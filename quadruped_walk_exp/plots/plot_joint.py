#!/usr/bin/env python3
"""
Plot low_m1_q for every `all_topics.csv` that lives under ~/data/quadruped_walk_2/*/.
Run from VS Code (or any terminal) with:  python3 plot_low_m1_q.py
"""

import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# 1) ---------- CONFIG ---------------------------------------------------------
ROOT_DIR = Path.home() / "data" / "quadruped_walk_2"      # expands the ~ for every OS
TIMESTAMP_FMT = "%Y-%m-%d %H:%M:%S.%f"                    # format used in your logger
COLUMN = "sport_pos0"                                       # the column we want to plot
# -----------------------------------------------------------------------------


def find_csv_files(root_dir: Path, filename: str = "all_topics.csv"):
    """Recursively collect all CSV paths called `filename` under root_dir."""
    return list(root_dir.rglob(filename))


def load_and_prepare(csv_path: Path):
    """
    Read a single CSV and return a DataFrame with:
      - timestamp parsed to datetime
      - t_sec: time in seconds since first row
      - COLUMN: the value of interest
    """
    # Only read the two columns we actually need -> saves memory & time
    df = pd.read_csv(csv_path, usecols=["timestamp", COLUMN])

    # Parse timestamps; drop rows where it fails (errors='coerce' gives NaT)
    df["timestamp"] = pd.to_datetime(df["timestamp"],
                                     format=TIMESTAMP_FMT,
                                     errors="coerce")
    df = df.dropna(subset=["timestamp"]).sort_values("timestamp")

    # Seconds since experiment start
    df["t_sec"] = (df["timestamp"] - df["timestamp"].iloc[0]).dt.total_seconds()
    return df


def main():
    csv_paths = find_csv_files(ROOT_DIR)

    if not csv_paths:
        print(f"❌  No all_topics.csv files found under {ROOT_DIR}")
        return

    plt.figure(figsize=(12, 6))

    for csv_path in sorted(csv_paths):
        label = csv_path.parent.name                # use folder name as legend label
        try:
            df = load_and_prepare(csv_path)
        except Exception as e:
            print(f"⚠️  {csv_path} could not be read: {e}")
            continue

        if COLUMN not in df.columns:
            print(f"⚠️  {csv_path} is missing the '{COLUMN}' column, skipped.")
            continue

        plt.plot(df["t_sec"], df[COLUMN], label=label)

    plt.xlabel("Time (s)")
    plt.ylabel(COLUMN)
    plt.title(f"{COLUMN} over time (all experiments)")
    plt.legend(fontsize="small", ncol=2, title="Experiment folder")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
