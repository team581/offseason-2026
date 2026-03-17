import sys
import os
import subprocess
import argparse
import wpiutil.log
import pandas as pd
import matplotlib.pyplot as plt
import json


def parse_args():
    parser = argparse.ArgumentParser(
        description="Analyze current draw from CTRE .hoot files"
    )
    parser.add_argument(
        "hoot_files", nargs="+", help="Path to one or more .hoot files to parse"
    )
    parser.add_argument(
        "--high-current-threshold",
        type=float,
        default=150.0,
        help="Threshold for total current to be considered 'high' (Amps)",
    )
    parser.add_argument(
        "--force-convert",
        action="store_true",
        help="Force reconversion of the .hoot file even if a .wpilog exists",
    )
    return parser.parse_args()


def convert_hoot_to_wpilog(hoot_path, force_convert):
    """Converts a .hoot file to .wpilog using owlet CLI, or uses cached if available."""
    if not hoot_path.endswith(".hoot"):
        print("Error: Input file must be a .hoot file.", file=sys.stderr)
        sys.exit(1)

    wpilog_path = hoot_path.rsplit(".", 1)[0] + "_temp.wpilog"

    if os.path.exists(wpilog_path) and not force_convert:
        print(f"Using cached converted log: {wpilog_path}")
        return wpilog_path

    print(f"Converting {hoot_path} to {wpilog_path} using owlet...")

    env = os.environ.copy()
    try:
        ps_command = f'$env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path","User"); owlet "{hoot_path}" "{wpilog_path}" -f wpilog'

        result = subprocess.run(
            ["powershell", "-Command", ps_command],
            check=True,
            capture_output=True,
            text=True,
            env=env,
        )
        print("Conversion successful.")
        return wpilog_path
    except subprocess.CalledProcessError as e:
        print(f"Error converting file with owlet: {e.stderr}", file=sys.stderr)
        sys.exit(1)


def load_log(filepaths):
    """Parses multiple WPILogs and returns a joined DataFrame of selected records."""
    all_data = []

    for path in filepaths:
        print(f"Loading generated log file: {path}")
        reader = wpiutil.log.DataLogReader(path)
        entries = {}
        entry_types = {}

        for record in reader:
            if record.isStart():
                start_data = record.getStartData()
                entries[start_data.entry] = start_data.name
                entry_types[start_data.entry] = start_data.type
            elif record.getEntry() in entries:
                entry_id = record.getEntry()
                entry_name = entries[entry_id]
                entry_type = entry_types.get(entry_id, "")

                is_motor = "Phoenix6/" in entry_name and "/SupplyCurrent" in entry_name
                is_time = entry_name in [
                    "/Robot/HubActivity/TimeSinceMatchStart",
                    "/Robot/HubActivity/TimeSinceTeleopEnable",
                ]

                if is_motor or is_time:
                    time_s = (
                        record.getTimestamp() / 1_000_000.0
                    )  # microseconds to seconds
                    value = None
                    if entry_type == "double":
                        value = record.getDouble()
                    elif entry_type == "int64":
                        value = record.getInteger()

                    if value is not None:
                        all_data.append(
                            {"timestamp": time_s, "name": entry_name, "value": value}
                        )

    if not all_data:
        print("No motor supply current data found in logs.")
        sys.exit(1)

    df = pd.DataFrame(all_data)
    df_pivot = df.pivot_table(
        index="timestamp", columns="name", values="value", aggfunc="mean"
    )
    df_pivot = df_pivot.sort_index().ffill().bfill()
    return df_pivot


def load_mapping():
    mapping_path = os.path.join(os.path.dirname(__file__), "motor_mapping.json")
    if os.path.exists(mapping_path):
        with open(mapping_path, "r") as f:
            return json.load(f)
    return {}


def process_motors(df):
    """Groups keys by Motor ID mapping."""
    mech_currents = pd.DataFrame(index=df.index)
    mapping = load_mapping()

    for col in df.columns:
        if "Phoenix6/" in col and "/SupplyCurrent" in col:
            parts = col.split("/")
            if len(parts) >= 3:
                # e.g., TalonFX-4 -> extracts 4
                motor_id_str = parts[1].replace("TalonFX-", "")

                # Fetch mapped name or fallback to original TalonFX-4
                motor_name = mapping.get(motor_id_str, parts[1])
                mech_currents[motor_name] = df[col].abs()

    mech_currents["Total_Supply_Current"] = mech_currents.sum(axis=1)
    return mech_currents


def analyze_incidents(mech_currents, threshold):
    """Finds periods where total current exceeded threshold."""
    is_high = mech_currents["Total_Supply_Current"] > threshold

    edges = is_high.astype(int).diff()
    starts = mech_currents.index[edges == 1].tolist()
    ends = mech_currents.index[edges == -1].tolist()

    if len(is_high) > 0 and pd.notna(is_high.iloc[0]):
        if is_high.iloc[0]:
            starts.insert(0, mech_currents.index[0])
        if is_high.iloc[-1]:
            ends.append(mech_currents.index[-1])

    incidents = []
    for s, e in zip(starts, ends):
        duration = e - s
        max_current = mech_currents.loc[s:e, "Total_Supply_Current"].max()
        incidents.append(
            {"start": s, "end": e, "duration": duration, "max_current": max_current}
        )

    incidents = sorted(incidents, key=lambda x: x["duration"], reverse=True)
    return incidents


def get_match_time_string(df, timestamp_s):
    """Formats the match time context (e.g., Auto - 10s remaining)."""
    if (
        "/Robot/HubActivity/TimeSinceTeleopEnable" in df.columns
        and "/Robot/HubActivity/TimeSinceMatchStart" in df.columns
    ):
        # Get the nearest temporal data if exact timestamp isn't aligned
        idx = df.index.get_indexer([timestamp_s], method="nearest")[0]
        closest_stamp = df.index[idx]

        tele_time = (
            df.loc[closest_stamp, "/Robot/HubActivity/TimeSinceTeleopEnable"]
            if "/Robot/HubActivity/TimeSinceTeleopEnable" in df.columns
            else -1
        )
        match_time = (
            df.loc[closest_stamp, "/Robot/HubActivity/TimeSinceMatchStart"]
            if "/Robot/HubActivity/TimeSinceMatchStart" in df.columns
            else -1
        )

        if tele_time > 0:
            time_left = max(0.0, 135.0 - tele_time)
            return f"Teleop - {time_left:.1f}s remaining"
        elif match_time > 0:
            time_left = max(0.0, 15.0 - match_time)
            return f"Auto - {time_left:.1f}s remaining"

    return "Unknown Period"


def generate_report_and_plots(df_raw, mech_currents, incidents):
    print("====== HOOT CURRENT ANALYSIS REPORT ======")

    print("\n1. Motors drawing the most current (Average Supply Current):")
    avg_currents = (
        mech_currents.drop(columns=["Total_Supply_Current"])
        .mean()
        .sort_values(ascending=False)
    )
    for sub, val in avg_currents.items():
        print(f"  - {sub}: {val:.2f} A")

    print("\n2. Aggregate Current Draw per Motor (1-second averages sum):")
    df_sec = mech_currents.drop(columns=["Total_Supply_Current"])
    sec_means = df_sec.groupby(df_sec.index // 1.0).mean()
    agg_totals = sec_means.sum().sort_values(ascending=False)
    for sub, val in agg_totals.items():
        print(f"  - {sub}: {val:.2f} A-sec")

    print("\n3. High Current Incidents (> 150A):")
    if not incidents:
        print("  - None found.")
    else:
        for idx, inc in enumerate(incidents[:5]):
            ctx = get_match_time_string(df_raw, inc["start"])
            print(
                f"  - Incident {idx + 1}: Duration {inc['duration']:.2f}s (Start: {inc['start']:.2f}s [{ctx}], Peak: {inc['max_current']:.2f}A)"
            )

    print("\n4. Point in match with most simultaneous current draw:")
    peak_stamp = mech_currents["Total_Supply_Current"].idxmax()
    peak_total = mech_currents.loc[peak_stamp, "Total_Supply_Current"]
    peak_ctx = get_match_time_string(df_raw, peak_stamp)
    print(f"  - Time: {peak_stamp:.2f}s [{peak_ctx}] with {peak_total:.2f}A peak draw.")
    print("  - Disaggregation at that instant:")
    peak_breakdown = (
        mech_currents.drop(columns=["Total_Supply_Current"])
        .loc[peak_stamp]
        .sort_values(ascending=False)
    )
    for sub, val in peak_breakdown.items():
        if val > 1.0:
            print(f"    - {sub}: {val:.2f} A")

    # Plotting
    plt.figure(figsize=(14, 8))

    plt.subplot(2, 1, 1)
    plt.plot(
        mech_currents.index,
        mech_currents["Total_Supply_Current"],
        label="Total Current",
        color="black",
        linewidth=1.5,
    )
    plt.axhline(150, color="red", linestyle="--", alpha=0.5, label="150A Threshold")
    plt.title("Total Motor Supply Current over Time (.hoot log)")
    plt.ylabel("Current (Amps)")
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    mech_only = mech_currents.drop(columns=["Total_Supply_Current"])
    plt.stackplot(
        mech_only.index,
        [mech_only[col] for col in mech_only.columns],
        labels=mech_only.columns,
        alpha=0.8,
    )
    plt.title("Current Draw per Motor (Stacked)")
    plt.xlabel("Log Time (Seconds)")
    plt.ylabel("Current (Amps)")
    plt.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    plt.grid(True)

    plt.tight_layout()
    plot_path = "hoot_current_analysis_graphs.png"
    plt.savefig(plot_path)
    print(f"\nGraphs saved to: {plot_path}")


def main():
    args = parse_args()
    wpilog_paths = []

    for hf in args.hoot_files:
        wpilog_paths.append(convert_hoot_to_wpilog(hf, args.force_convert))

    df_raw = load_log(wpilog_paths)
    mech_currents = process_motors(df_raw)
    incidents = analyze_incidents(mech_currents, args.high_current_threshold)
    generate_report_and_plots(df_raw, mech_currents, incidents)


if __name__ == "__main__":
    main()
