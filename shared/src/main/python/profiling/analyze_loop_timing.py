"""Analyze robot-loop timing entries in one or more WPILOG files."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any

import wpiutil.log

SCHEDULER_PREFIX = "/Robot/Scheduler/"
BUILD_SHA_KEY = "/Robot/Metadata/GitSHA"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument(
        "--mode", choices=("any", "disabled", "auto", "teleop", "test"), default="any"
    )
    parser.add_argument("--discard-seconds", type=float, default=2.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--markdown", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--allow-mixed-builds", action="store_true")
    return parser.parse_args()


def percentile(sorted_values: list[float], fraction: float) -> float:
    if not sorted_values:
        return math.nan
    position = (len(sorted_values) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return sorted_values[lower]
    weight = position - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def summarize(values: list[float]) -> dict[str, float | int]:
    finite = sorted(value for value in values if math.isfinite(value) and value >= 0.0)
    if not finite:
        return {"count": 0}
    return {
        "count": len(finite),
        "mean": statistics.fmean(finite),
        "p50": percentile(finite, 0.50),
        "p95": percentile(finite, 0.95),
        "p99": percentile(finite, 0.99),
        "max": finite[-1],
        "overruns_20ms": sum(value > 0.020 for value in finite),
    }


def robot_mode(enabled: bool, autonomous: bool, test: bool) -> str:
    if not enabled:
        return "disabled"
    if test:
        return "test"
    return "auto" if autonomous else "teleop"


def read_log(
    path: Path, selected_mode: str, discard_seconds: float
) -> tuple[dict[str, list[float]], set[str]]:
    entries: dict[int, tuple[str, str]] = {}
    samples: dict[str, list[float]] = defaultdict(list)
    build_shas: set[str] = set()
    enabled = autonomous = test = False
    current_mode = "disabled"
    transition_timestamp = 0
    first_timestamp: int | None = None
    discard_micros = int(discard_seconds * 1_000_000)

    for record in wpiutil.log.DataLogReader(str(path)):
        timestamp = record.getTimestamp()
        if first_timestamp is None:
            first_timestamp = timestamp
            transition_timestamp = timestamp

        if record.isStart():
            start = record.getStartData()
            entries[start.entry] = (start.name, start.type)
            continue

        entry = entries.get(record.getEntry())
        if entry is None:
            continue
        name, entry_type = entry

        old_mode = current_mode
        if entry_type == "boolean":
            if name == "DS:enabled":
                enabled = record.getBoolean()
            elif name == "DS:autonomous":
                autonomous = record.getBoolean()
            elif name == "DS:test":
                test = record.getBoolean()
        current_mode = robot_mode(enabled, autonomous, test)
        if current_mode != old_mode:
            transition_timestamp = timestamp

        if name == BUILD_SHA_KEY and entry_type == "string":
            build_shas.add(record.getString())

        if not name.startswith(SCHEDULER_PREFIX) or entry_type != "double":
            continue
        if selected_mode != "any" and current_mode != selected_mode:
            continue
        if timestamp - transition_timestamp < discard_micros:
            continue
        samples[name].append(record.getDouble())

    return samples, build_shas


def analyze(paths: list[Path], mode: str, discard_seconds: float) -> dict[str, Any]:
    combined: dict[str, list[float]] = defaultdict(list)
    build_shas: set[str] = set()
    for path in paths:
        samples, log_build_shas = read_log(path, mode, discard_seconds)
        for key, values in samples.items():
            combined[key].extend(values)
        build_shas.update(sha for sha in log_build_shas if sha)

    metrics = {key: summarize(values) for key, values in sorted(combined.items())}
    total_key = SCHEDULER_PREFIX + "RobotPeriodicExecution"
    denominator = metrics.get(total_key, {}).get("mean")
    if not isinstance(denominator, float) or denominator <= 0.0:
        denominator = sum(
            metric.get("mean", 0.0)
            for key, metric in metrics.items()
            if "/Scheduler/Subsystems/" in key
        )
    if denominator > 0.0:
        for key, metric in metrics.items():
            if "/Scheduler/Subsystems/" in key or "/Scheduler/Inputs/" in key:
                metric["mean_contribution_percent"] = (
                    float(metric.get("mean", 0.0)) / denominator * 100.0
                )
        overhead = metrics.get(SCHEDULER_PREFIX + "ProfilerClockOverhead", {}).get(
            "mean"
        )
        if isinstance(overhead, float):
            metrics[SCHEDULER_PREFIX + "ProfilerClockOverhead"][
                "mean_contribution_percent"
            ] = overhead / denominator * 100.0

    report = {
        "schema_version": 1,
        "mode": mode,
        "discard_seconds": discard_seconds,
        "logs": [str(path) for path in paths],
        "build_shas": sorted(build_shas),
        "metrics": metrics,
    }
    overhead_metric = metrics.get(SCHEDULER_PREFIX + "ProfilerClockOverhead")
    total_metric = metrics.get(total_key)
    if overhead_metric and total_metric and total_metric.get("p50", 0.0) > 0.0:
        median_percent = overhead_metric["p50"] / total_metric["p50"] * 100.0
        report["instrumentation_overhead"] = {
            "median_percent": median_percent,
            "acceptable": median_percent < 1.0,
        }
    return report


def compare(report: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    comparisons: dict[str, dict[str, float]] = {}
    for key, candidate_metric in report["metrics"].items():
        baseline_metric = baseline.get("metrics", {}).get(key)
        if not baseline_metric:
            continue
        baseline_p99 = baseline_metric.get("p99")
        candidate_p99 = candidate_metric.get("p99")
        if not isinstance(baseline_p99, (int, float)) or not baseline_p99:
            continue
        if not isinstance(candidate_p99, (int, float)):
            continue
        comparisons[key] = {
            "baseline_p99": baseline_p99,
            "candidate_p99": candidate_p99,
            "p99_improvement_percent": (baseline_p99 - candidate_p99)
            / baseline_p99
            * 100.0,
        }
    return comparisons


def markdown(report: dict[str, Any]) -> str:
    lines = [
        "# Robot loop timing report",
        "",
        f"Mode: `{report['mode']}`  ",
        f"Build SHA(s): `{', '.join(report['build_shas']) or 'unknown'}`",
        "",
        "| Metric | Samples | Mean (ms) | p95 (ms) | p99 (ms) | Max (ms) | >20 ms |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for key, metric in report["metrics"].items():
        if not metric.get("count"):
            continue
        label = key.removeprefix(SCHEDULER_PREFIX)
        lines.append(
            f"| {label} | {metric['count']} | {metric['mean'] * 1000:.3f} | "
            f"{metric['p95'] * 1000:.3f} | {metric['p99'] * 1000:.3f} | "
            f"{metric['max'] * 1000:.3f} | {metric['overruns_20ms']} |"
        )
    if overhead := report.get("instrumentation_overhead"):
        lines.extend(
            [
                "",
                f"Instrumentation clock overhead: {overhead['median_percent']:.3f}% "
                f"(`acceptable={str(overhead['acceptable']).lower()}`).",
            ]
        )
    if comparisons := report.get("comparison"):
        lines.extend(
            [
                "",
                "## Baseline comparison",
                "",
                "| Metric | Baseline p99 (ms) | Candidate p99 (ms) | Improvement |",
                "|---|---:|---:|---:|",
            ]
        )
        for key, result in comparisons.items():
            lines.append(
                f"| {key.removeprefix(SCHEDULER_PREFIX)} | "
                f"{result['baseline_p99'] * 1000:.3f} | "
                f"{result['candidate_p99'] * 1000:.3f} | "
                f"{result['p99_improvement_percent']:.1f}% |"
            )
    lines.extend(
        [
            "",
            "> Laptop and historical-log measurements demonstrate relative changes; they do not "
            "prove absolute roboRIO execution time.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    report = analyze(args.logs, args.mode, args.discard_seconds)
    if len(report["build_shas"]) > 1 and not args.allow_mixed_builds:
        raise SystemExit(
            "Refusing to mix build SHAs; analyze each build separately or pass --allow-mixed-builds"
        )
    if args.baseline:
        report["comparison"] = compare(report, json.loads(args.baseline.read_text()))

    json_text = json.dumps(report, indent=2) + "\n"
    markdown_text = markdown(report)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json_text)
    else:
        print(json_text, end="")
    if args.markdown:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(markdown_text)


if __name__ == "__main__":
    main()
