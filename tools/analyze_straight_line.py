"""Compare drive commands and measured speeds from saved straight-line runs.

Reads drive-output.csv and summary.md from each run folder under root.
Writes acceleration metrics and a Markdown summary; --plot also requires matplotlib.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import re
import statistics
from pathlib import Path


def slope(times: list[float], values: list[float]) -> float:
    mean_time = statistics.mean(times)
    mean_value = statistics.mean(values)
    return sum((t - mean_time) * (v - mean_value) for t, v in zip(times, values)) / sum(
        (t - mean_time) ** 2 for t in times
    )


def acceleration_limit(summary: str) -> float:
    match = re.search(r"maxAcceleration=([\d.]+)", summary)
    if match is None:
        raise ValueError("Missing maxAcceleration in motion summary")
    return float(match.group(1))


def analyze(directory: Path, window: float) -> dict:
    with (directory / "drive-output.csv").open() as stream:
        rows = [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]
    summary = (directory / "summary.md").read_text()
    limit = acceleration_limit(summary)
    times = [row["time_s"] for row in rows]
    command = [
        math.hypot(row["commanded_vx_mps"], row["commanded_vy_mps"]) for row in rows
    ]
    measured = [
        math.hypot(row["measured_vx_mps"], row["measured_vy_mps"]) for row in rows
    ]
    first = next(i for i, value in enumerate(command) if value > 1e-6)
    # Stop at the first plateau or decrease; a later speed maximum may be in cruise.
    peak = next(
        (
            i - 1
            for i in range(first + 1, len(command))
            if command[i] <= command[i - 1] + 1e-6
        ),
        len(command) - 1,
    )
    rates = [
        (command[i] - command[i - 1]) / (times[i] - times[i - 1])
        for i in range(1, len(rows))
    ]
    raw = [
        (measured[i] - measured[i - 1]) / (times[i] - times[i - 1])
        for i in range(1, len(rows))
    ]
    # Fit the ramp after startup and before braking.
    ramp = [
        i for i, t in enumerate(times) if times[first] + 0.25 <= t <= times[peak] - 0.1
    ]
    fitted = None
    if len(ramp) >= 5 and times[ramp[-1]] - times[ramp[0]] >= 0.25:
        fitted = slope([times[i] for i in ramp], [measured[i] for i in ramp])
    rising = [
        rates[i - 1] for i in range(first + 1, len(rows)) if rates[i - 1] > limit * 0.1
    ]
    window_times, window_acceleration = [], []
    for i, t in enumerate(times):
        selected = [j for j in range(i + 1) if t - window <= times[j] <= t]
        if (
            len(selected) >= 4
            and times[selected[-1]] - times[selected[0]] >= window * 0.8
        ):
            window_times.append(statistics.mean(times[j] for j in selected))
            window_acceleration.append(
                slope([times[j] for j in selected], [measured[j] for j in selected])
            )
    result = {
        "run": directory.name,
        "acceleration_limit_mps2": limit,
        "endpoint_passed": "Result: **PASSED**" in summary,
        "max_sample_interval_seconds": max(b - a for a, b in itertools.pairwise(times)),
        "startup_command_step_mps": command[first] - command[first - 1],
        "median_command_ramp_mps2": statistics.median(rising) if rising else None,
        "fitted_measured_ramp_mps2": fitted,
        "peak_command_braking_mps2": max(0, -min(rates[first:])),
        "peak_raw_measured_abs_acceleration_mps2": max(map(abs, raw)),
        "peak_windowed_measured_abs_acceleration_mps2": max(
            map(abs, window_acceleration)
        ),
        "peak_measured_speed_mps": max(measured),
        "time_to_command_2_9_mps": next(
            (t - times[first] for t, v in zip(times, command) if v >= 2.9), None
        ),
        "time_to_measured_2_9_mps": next(
            (t - times[first] for t, v in zip(times, measured) if v >= 2.9), None
        ),
    }
    return {
        "metrics": result,
        "times": times,
        "command": command,
        "measured": measured,
        "raw": raw,
        "window_times": window_times,
        "window_acceleration": window_acceleration,
    }


def plot(runs: list[dict], output: Path, window: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # Omit runs named *-repeat from plots; keep them in the metrics table.
    originals = [run for run in runs if not run["metrics"]["run"].endswith("-repeat")]
    fig, axes = plt.subplots(
        math.ceil(len(originals) / 2), 2, figsize=(12, 10), constrained_layout=True
    )
    for axis, run in zip(axes.flat, originals):
        metric = run["metrics"]
        axis.plot(
            run["times"],
            run["command"],
            label="Trailblazer command",
            color="#185abc",
            linewidth=2,
        )
        axis.plot(
            run["times"],
            run["measured"],
            label="Measured simulation",
            color="#b45309",
            linewidth=1,
            alpha=0.85,
        )
        axis.set(
            title=f"Acceleration setting: {metric['acceleration_limit_mps2']:g} m/s²",
            xlabel="Time (s)",
            ylabel="Speed (m/s)",
        )
        axis.grid(alpha=0.2)
        axis.legend(fontsize=8)
    fig.suptitle("Straight-line speed: commanded and measured")
    fig.savefig(output / "velocity-sweep.png", dpi=160)
    plt.close(fig)

    selected = [
        run
        for run in originals
        if run["metrics"]["acceleration_limit_mps2"] in (0.25, 0.75)
    ]
    fig, axes = plt.subplots(
        len(selected), 1, figsize=(12, 6), constrained_layout=True, squeeze=False
    )
    for axis, run in zip(axes.flat, selected):
        limit = run["metrics"]["acceleration_limit_mps2"]
        axis.plot(
            run["times"][1:],
            run["raw"],
            color="#b45309",
            alpha=0.45,
            linewidth=1,
            label="Raw finite difference",
        )
        axis.plot(
            run["window_times"],
            run["window_acceleration"],
            color="#185abc",
            linewidth=1.5,
            label=f"{window * 1000:g} ms local linear fit",
        )
        for boundary in (-limit, limit):
            axis.axhline(boundary, color="black", linestyle="--", linewidth=0.8)
        axis.set(
            title=f"Acceleration setting: {limit:g} m/s²",
            xlabel="Time (s)",
            ylabel="Acceleration (m/s²)",
        )
        axis.grid(alpha=0.2)
        axis.legend(fontsize=8)
    fig.suptitle(
        "Measured acceleration: raw and smoothed\nSmoothing reduces spikes and spreads startup and braking changes"
    )
    fig.savefig(output / "acceleration-noise.png", dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="Parent directory of the run folders")
    parser.add_argument("--window-seconds", type=float, default=0.12)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.window_seconds) or args.window_seconds <= 0:
        parser.error("window must be positive")
    directories = sorted(
        (
            path.parent
            for path in args.root.glob("*/drive-output.csv")
            if (path.parent / "summary.md").is_file()
        ),
        key=lambda path: (
            acceleration_limit((path / "summary.md").read_text()),
            path.name,
        ),
    )
    if not directories:
        parser.error("no drive-output.csv files found")
    runs = [analyze(directory, args.window_seconds) for directory in directories]
    (args.root / "acceleration-metrics.json").write_text(
        json.dumps(
            {
                "window_seconds": args.window_seconds,
                "runs": [run["metrics"] for run in runs],
            },
            indent=2,
        )
        + "\n"
    )
    table = [
        "| Run | Setting | Startup Δv | Command ramp | Measured ramp fit | Peak command braking | Raw measured peak | Windowed measured peak |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for run in runs:
        m = run["metrics"]
        values = [
            m[key]
            for key in (
                "acceleration_limit_mps2",
                "startup_command_step_mps",
                "median_command_ramp_mps2",
                "fitted_measured_ramp_mps2",
                "peak_command_braking_mps2",
                "peak_raw_measured_abs_acceleration_mps2",
                "peak_windowed_measured_abs_acceleration_mps2",
            )
        ]
        table.append(
            "| "
            + m["run"]
            + " | "
            + " | ".join("—" if value is None else f"{value:.3f}" for value in values)
            + " |"
        )
    report = "\n".join(table) + "\n\nUnits: acceleration in m/s²; startup Δv in m/s.\n"
    report += "Ramp fits skip the first 250 ms and last 100 ms of the rising command and need at least 250 ms of data. A dash means the metric could not be calculated.\n"
    report += "Long sample intervals affect the fit. Check max_sample_interval_seconds in acceleration-metrics.json when a ramp looks slow.\n"
    report += f"Smoothed acceleration uses a {args.window_seconds * 1000:g} ms linear fit of measured speed. It does not measure changes in travel direction.\n"
    (args.root / "acceleration-summary.md").write_text(report)
    print(report)
    if args.plot:
        plot(runs, args.root, args.window_seconds)


if __name__ == "__main__":
    main()
