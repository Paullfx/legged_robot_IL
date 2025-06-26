#!/usr/bin/env python3
"""
Align curves by first significant change (>0.01 within 1 s) and plot.
"""

import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

ROOT_DIR = Path.home() / "data" / "quadruped_walk_2"
COL      = "low_m1_q"                   # value to track
THRESH   = 0.01                         # significant change
WIN_SEC  = pd.Timedelta(seconds=1)      # 1-second window
TS_FMT   = "%Y-%m-%d %H:%M:%S.%f"       # timestamp format written by logger


def first_significant_change(df: pd.DataFrame) -> pd.Timestamp | None:
    """
    Return the earliest timestamp ts₀ such that |q(t) - q(ts₀)| > THRESH
    for some t with 0 < t - ts₀ ≤ 1 s.  If not found, return None.
    """
    # Use merge_asof to match each row with the first row ≥ ts+WIN_SEC
    later = df[["timestamp", COL]].copy()
    later["timestamp"] += WIN_SEC
    merged = pd.merge_asof(df, later, on="timestamp", direction="forward",
                           suffixes=("", "_later"))
    diff = (merged[f"{COL}_later"] - merged[COL]).abs()
    mask = diff > THRESH
    if mask.any():
        return merged.loc[mask, "timestamp"].iloc[0]
    return None


def load_trim(csv_path: Path) -> pd.DataFrame | None:
    """Load CSV, find ts₀, trim & re-zero time. Return prepared DataFrame."""
    df = pd.read_csv(csv_path, usecols=["timestamp", COL])
    df["timestamp"] = pd.to_datetime(df["timestamp"], format=TS_FMT, errors="coerce")
    df = df.dropna(subset=["timestamp"]).sort_values("timestamp").reset_index(drop=True)

    ts0 = first_significant_change(df)
    if ts0 is None:
        print(f"⚠️  {csv_path} never exceeds ±{THRESH}; skipped.")
        return None

    align_start = ts0.floor("S")  # beginning of that second
    df = df[df["timestamp"] >= align_start].copy()
    df["t_sec"] = (df["timestamp"] - align_start).dt.total_seconds()
    return df


def main():
    csv_files = sorted(Path(ROOT_DIR).rglob("all_topics.csv"))
    if not csv_files:
        print(f"❌  No all_topics.csv files found under {ROOT_DIR}")
        return

    plt.figure(figsize=(12, 6))

    for csv in csv_files:
        label = csv.parent.name
        try:
            df = load_trim(csv)
        except Exception as e:
            print(f"⚠️  {csv} could not be processed: {e}")
            continue
        if df is None:
            continue

        plt.plot(df["t_sec"], df[COL], label=label)

    plt.title(f"{COL} aligned by first ≥{THRESH} change within 1 s")
    plt.xlabel("Aligned time (s)")
    plt.ylabel(COL)
    plt.legend(title="Experiment", fontsize="small", ncol=2)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
