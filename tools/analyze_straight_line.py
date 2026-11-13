# /// script
# requires-python = ">=3.10"
# dependencies = ["robotpy-wpiutil>=2026.2.2,<2027", "matplotlib>=3.10,<4"]
# ///
"""Generate metrics, a Markdown summary, and plots from straight-line tests in normal robot WPILOGs.

Usage: uv run tools/analyze_straight_line.py <log.wpilog-or-directory>
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import statistics
from bisect import bisect_right
from pathlib import Path

import matplotlib
from wpiutil.log import DataLogReader

matplotlib.use("Agg")
import matplotlib.pyplot as plt

WINDOW_SECONDS = 0.12
STRAIGHT = "Tests/StraightLine/"


def read_runs(path: Path) -> list[dict]:
    reader = DataLogReader(str(path))
    if not reader.isValid():
        raise ValueError(f"Invalid WPILOG: {path}")
    entries = {}
    runs = []
    data = None
    current_run = None
    for record in reader:
        if record.isStart():
            entry = record.getStartData()
            name = entry.name.lstrip("/").removeprefix("Robot/")
            if name.startswith(STRAIGHT):
                entries[entry.entry] = (name, entry.type)
        elif record.isFinish():
            entries.pop(record.getFinishEntry(), None)
        elif not record.isControl() and record.getEntry() in entries:
            name, kind = entries[record.getEntry()]
            if kind == "double":
                value = record.getDouble()
            elif kind == "string":
                value = record.getString()
            else:
                continue
            if name == STRAIGHT + "RunId" and value != current_run:
                current_run = value
                data = {}
                runs.append(data)
            if data is not None:
                data.setdefault(name, {})[record.getTimestamp()] = value
    return runs


def slope(times: list[float], values: list[float]) -> float:
    mean_time = statistics.mean(times)
    mean_value = statistics.mean(values)
    return sum((t - mean_time) * (v - mean_value) for t, v in zip(times, values)) / sum(
        (t - mean_time) ** 2 for t in times
    )


def analyze(path: Path, data: dict, run_index: int) -> dict:
    def latest(name: str):
        values = data.get(name, {})
        return values[max(values)] if values else None

    # DogLog timestamps each call separately. Sample time is logged last in each loop.
    channels = [
        sorted(data.get(STRAIGHT + name + "SpeedMetersPerSecond", {}).items())
        for name in ("Commanded", "Measured")
    ]
    channel_times = [[t for t, _ in channel] for channel in channels]
    samples = {}
    for timestamp, elapsed in sorted(data.get(STRAIGHT + "ElapsedSeconds", {}).items()):
        indices = [bisect_right(times, timestamp) - 1 for times in channel_times]
        if all(index >= 0 for index in indices):
            samples[elapsed] = tuple(
                channel[index][1] for channel, index in zip(channels, indices)
            )
    times = sorted(samples)
    command = [samples[t][0] for t in times]
    measured = [samples[t][1] for t in times]
    if not all(math.isfinite(v) for v in command + measured):
        raise ValueError(f"Nonfinite drive sample in {path}")
    rates = [
        (command[i] - command[i - 1]) / (times[i] - times[i - 1])
        for i in range(1, len(times))
    ]
    raw = [
        (measured[i] - measured[i - 1]) / (times[i] - times[i - 1])
        for i in range(1, len(times))
    ]
    limit = latest(STRAIGHT + "MaxAcceleration")
    rising = [rate for rate in rates if rate > limit * 0.1]
    first = next((i for i, value in enumerate(command) if value > 1e-6), None)
    fitted = None
    if first is not None:
        # Fit after startup and before the first command plateau or braking.
        peak = next(
            (
                i - 1
                for i in range(first + 1, len(command))
                if command[i] <= command[i - 1] + 1e-6
            ),
            len(command) - 1,
        )
        ramp = [
            i
            for i, t in enumerate(times)
            if times[first] + 0.25 <= t <= times[peak] - 0.1
        ]
        if len(ramp) >= 5 and times[ramp[-1]] - times[ramp[0]] >= 0.25:
            fitted = slope([times[i] for i in ramp], [measured[i] for i in ramp])
    window_times, window_acceleration = [], []
    for i, t in enumerate(times):
        selected = [j for j in range(i + 1) if t - WINDOW_SECONDS <= times[j] <= t]
        if (
            len(selected) >= 4
            and times[selected[-1]] - times[selected[0]] >= WINDOW_SECONDS * 0.8
        ):
            window_times.append(statistics.mean(times[j] for j in selected))
            window_acceleration.append(
                slope([times[j] for j in selected], [measured[j] for j in selected])
            )
    status = latest(STRAIGHT + "Status")
    if status not in ("PASSED", "FAILED", "ABORTED"):
        status = "INCOMPLETE"
    metrics = {
        "log": str(path),
        "run_id": latest(STRAIGHT + "RunId"),
        "status": status,
        "reason": latest(STRAIGHT + "Reason") or "Log ended without a result",
        "battery_volts": latest(STRAIGHT + "BatteryVolts"),
        "initial_heading_degrees": latest(STRAIGHT + "InitialHeadingDegrees"),
        "distance_meters": latest(STRAIGHT + "DistanceMeters"),
        "max_velocity_mps": latest(STRAIGHT + "MaxVelocity"),
        "acceleration_limit_mps2": limit,
        "duration_seconds": latest(STRAIGHT + "ElapsedSeconds"),
        "final_position_error_meters": latest(STRAIGHT + "PositionErrorMeters"),
        "max_cross_track_error_meters": max(
            data.get(STRAIGHT + "CrossTrackErrorMeters", {}).values(), default=None
        ),
        "peak_measured_speed_mps": max(measured, default=None),
        "max_sample_interval_seconds": max(
            (b - a for a, b in itertools.pairwise(times)), default=None
        ),
        "startup_command_step_mps": command[first]
        - (command[first - 1] if first else 0)
        if first is not None
        else None,
        "median_command_ramp_mps2": statistics.median(rising) if rising else None,
        "fitted_measured_ramp_mps2": fitted,
        "peak_command_braking_mps2": max(0, -min(rates)) if rates else None,
        "peak_raw_measured_abs_acceleration_mps2": max(map(abs, raw), default=None),
        "peak_windowed_measured_abs_acceleration_mps2": max(
            map(abs, window_acceleration), default=None
        ),
    }
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)
    axes[0].plot(times, command, label="Applied command")
    axes[0].plot(times, measured, label="Measured")
    axes[0].set(
        ylabel="Speed (m/s)",
        title=f"{path.parent.name}/{path.name}: {metrics['status']}",
    )
    axes[1].plot(times[1:], raw, alpha=0.4, label="Raw finite difference")
    axes[1].plot(window_times, window_acceleration, label="120 ms linear fit")
    for boundary in (-limit, limit):
        axes[1].axhline(boundary, color="black", linestyle="--", linewidth=0.8)
    axes[1].set(xlabel="Time (s)", ylabel="Acceleration (m/s²)")
    for axis in axes:
        axis.grid(alpha=0.2)
        axis.legend()
    fig.savefig(path.with_name(f"{path.stem}-test-{run_index}.png"), dpi=160)
    plt.close(fig)
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", type=Path, help="WPILOG file or directory of saved runs"
    )
    args = parser.parse_args()
    paths = (
        sorted(args.input.rglob("*.wpilog")) if args.input.is_dir() else [args.input]
    )
    runs = []
    try:
        for path in paths:
            for index, data in enumerate(read_runs(path), 1):
                if STRAIGHT + "MaxAcceleration" in data:
                    runs.append(analyze(path, data, index))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    if not runs:
        parser.error("no straight-line test runs found in the robot WPILOGs")
    output = args.input if args.input.is_dir() else args.input.parent
    (output / "acceleration-metrics.json").write_text(
        json.dumps(runs, indent=2, allow_nan=False) + "\n"
    )
    report = ["# Straight-line tests", ""]
    for run in runs:
        report += [
            f"## {Path(run['log']).relative_to(output)} — {run['run_id']}",
            "",
            f"Result: **{run['status']}** — {run['reason']}",
            "",
            "| Measurement | Value |",
            "|---|---:|",
        ]
        for key, value in run.items():
            if key in ("log", "run_id", "status", "reason"):
                continue
            report.append(
                f"| {key.replace('_', ' ')} | {'—' if value is None else f'{value:.3f}'} |"
            )
        report.append("")
    report += [
        "Passing requires reaching the goal and settling; acceleration limits are not checked.",
        "Acceleration is calculated from measured speed and includes simulation timing noise. The 120 ms linear fit smooths spikes and changes in travel direction are not measured.",
        "Ramp fits skip the first 250 ms and last 100 ms of the rising command and need at least 250 ms of data. A dash means the metric could not be calculated.",
        "",
    ]
    summary = "\n".join(report)
    (output / "acceleration-summary.md").write_text(summary)
    print(summary)


if __name__ == "__main__":
    main()
