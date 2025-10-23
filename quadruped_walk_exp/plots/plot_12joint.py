#!/usr/bin/env python3
"""
Plot low_m0_q to low_m11_q for one or more all_topics.csv files.
Usage:
    python3 plot_12joint.py --csv /path/to/first/all_topics.csv --csv /path/to/second/all_topics.csv
Or, to plot the latest under ~/data/quadruped_walk_2:
    python3 plot_12joint.py
"""

import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import argparse

# ---------- CONFIG ---------------------------------------------------------
ROOT_DIR = Path.home() / "data" / "quadruped_walk_2"
TIMESTAMP_FMT = "%Y-%m-%d %H:%M:%S.%f"
JOINT_COLUMNS = [f"low_m{i}_q" for i in range(0, 12)]  # low_m0_q ... low_m11_q
# --------------------------------------------------------------------------


def find_csv_files(root_dir: Path, filename: str = "all_topics.csv"):
    """Recursively collect all CSV paths called `filename` under root_dir."""
    return list(root_dir.rglob(filename))


def load_and_prepare(csv_path: Path):
    """
    Read a single CSV and return a DataFrame with:
      - timestamp parsed to datetime
      - t_sec: time in seconds since first row
      - JOINT_COLUMNS: the 12 joint values
    """
    usecols = ["timestamp"] + JOINT_COLUMNS
    df = pd.read_csv(csv_path, usecols=usecols)

    df["timestamp"] = pd.to_datetime(df["timestamp"],
                                     format=TIMESTAMP_FMT,
                                     errors="coerce")
    df = df.dropna(subset=["timestamp"]).sort_values("timestamp")
    df["t_sec"] = (df["timestamp"] - df["timestamp"].iloc[0]).dt.total_seconds()
    return df


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', type=str, action='append',
                        help='Path to an all_topics.csv file (can be given multiple times)')
    args = parser.parse_args()

    if args.csv:
        csv_paths = [Path(p) for p in args.csv]
    else:
        # Default: plot the latest found under ROOT_DIR
        found = find_csv_files(ROOT_DIR)
        if not found:
            print(f"❌  No all_topics.csv files found under {ROOT_DIR}")
            return
        # Use the latest two if available, else just the latest
        found = sorted(found, key=os.path.getmtime, reverse=True)
        csv_paths = found[:2]

    if not csv_paths:
        print(f"❌  No all_topics.csv files to plot.")
        return

    fig, axs = plt.subplots(6, 2, figsize=(16, 18), sharex=True)
    axs = axs.flatten()

    for csv_path in csv_paths:
        label = csv_path.parent.name
        try:
            df = load_and_prepare(csv_path)
        except Exception as e:
            print(f"⚠️  {csv_path} could not be read: {e}")
            continue

        for i, col in enumerate(JOINT_COLUMNS):
            if col not in df.columns:
                print(f"⚠️  {csv_path} is missing the '{col}' column, skipped.")
                continue
            axs[i].plot(df["t_sec"], df[col], label=label, alpha=0.8)

    for i, col in enumerate(JOINT_COLUMNS):
        axs[i].set_title(col)
        axs[i].set_ylabel("Position (rad)")
        axs[i].grid(True)
        axs[i].legend(fontsize="x-small")

    axs[-2].set_xlabel("Time (s)")
    axs[-1].set_xlabel("Time (s)")
    plt.suptitle("Low-level Joint Positions (low_m0_q to low_m11_q) over time")
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show()


if __name__ == "__main__":
    main()
