#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# python3 plot_vision.py --csv ~/data/quadruped_walk_2/exp_50hz_2025xxxx_xxxxxx


def smart_pick(df, *cands):
    """Return the first existing column in df that also has non-all-NaN values."""
    for c in cands:
        if c in df.columns and not df[c].isna().all():
            return c
    # if none exist, return first for error clarity
    return cands[0]

def load_csv(csv_or_dir):
    # Accept either a folder produced by your logger or a direct .csv
    path = csv_or_dir
    if os.path.isdir(path):
        path = os.path.join(path, "all_topics.csv")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"CSV not found: {path}")
    # Keep empty strings to coerce later
    df = pd.read_csv(path, keep_default_na=False)
    # Normalize empty-string numeric fields to NaN now; we'll handle special cases like aruco_x below
    df = df.replace("", np.nan)
    return df

def to_seconds_since_start(ts_series):
    """ts_series is string like 'YYYY-mm-dd HH:MM:SS.sss'. Convert to seconds since first."""
    # If missing/NaN, fabricate a monotonic index in seconds
    if ts_series.isna().all():
        return np.arange(len(ts_series), dtype=float)
    t = pd.to_datetime(ts_series, errors="coerce")
    if t.isna().all():
        return np.arange(len(ts_series), dtype=float)
    t0 = t.dropna().iloc[0]
    secs = (t - t0).dt.total_seconds()
    return secs.fillna(method="ffill").fillna(method="bfill").to_numpy()

def main():
    ap = argparse.ArgumentParser(description="Plot quadruped logs with signed Y and aruco_x.")
    ap.add_argument("--csv", required=True,
                    help="Path to logger folder (exp_50hz_xxx) or to all_topics.csv")
    ap.add_argument("--save", default=None, help="Optional path to save figure PNG (no extension change).")
    args = ap.parse_args()

    df = load_csv(args.csv)

    # --- Time axis ---
    t = to_seconds_since_start(df.get("timestamp", pd.Series([np.nan]*len(df))))

    # --- Columns (prefer 'sport_*', fallback to 'lf_sport_*') ---
    x_col = smart_pick(df, "sport_pos0", "lf_sport_pos0")
    y_col = smart_pick(df, "sport_pos1", "lf_sport_pos1")
    yaw_col = smart_pick(df, "sport_yaw_speed", "lf_sport_yaw_speed")

    # ArUco: treat blanks/NaN as 0, as requested
    aruco_col = "aruco_x"
    if aruco_col not in df.columns:
        # keep a zero series if not present at all
        aruco_x = np.zeros(len(df), dtype=float)
    else:
        # values may be strings/NaN—coerce; missing -> 0
        aruco_x = pd.to_numeric(df[aruco_col], errors="coerce").fillna(0.0).to_numpy()

    # Numeric series for plotting; IMPORTANT: DO NOT take abs() on Y anywhere
    x = pd.to_numeric(df[x_col], errors="coerce").to_numpy()
    y = pd.to_numeric(df[y_col], errors="coerce").to_numpy()
    yaw = pd.to_numeric(df[yaw_col], errors="coerce").to_numpy()

    # Mask where both x and y are finite for the trajectory plot
    valid_xy = np.isfinite(x) & np.isfinite(y)
    valid_t_y = np.isfinite(t) & np.isfinite(y)
    valid_t_yaw = np.isfinite(t) & np.isfinite(yaw)

    # --- Figure layout: Left = trajectory (x vs y); Right = 3 subplots (top=aruco_x, mid=yaw, bottom=y) ---
    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(ncols=2, nrows=3, width_ratios=[1.3, 1.0], height_ratios=[1, 1, 1], wspace=0.3, hspace=0.35)

    # Left big trajectory spanning 3 rows
    ax_traj = fig.add_subplot(gs[:, 0])
    ax_traj.plot(y[valid_xy], x[valid_xy], linewidth=1.5)  # x forward vs lateral y: conventional is x vs y; if you want x on vertical, swap back
    ax_traj.set_xlabel("Lateral position y (m)")
    ax_traj.set_ylabel("Forward position x (m)")
    ax_traj.set_title("Trajectory (signed y)")
    ax_traj.set_aspect("equal", adjustable="box")  # keep true aspect
    ax_traj.grid(True, linewidth=0.5, alpha=0.6)

    # Top-right: aruco_x over time (zeros where missing)
    ax_arx = fig.add_subplot(gs[0, 1])
    ax_arx.plot(t, aruco_x, linewidth=1.2)
    ax_arx.set_xlabel("Time (s)")
    ax_arx.set_ylabel("aruco_x (m)")
    ax_arx.set_title("ArUco X (missing → 0)")
    ax_arx.grid(True, linewidth=0.5, alpha=0.6)

    # Mid-right: yaw angular velocity
    ax_yaw = fig.add_subplot(gs[1, 1])
    ax_yaw.plot(t[valid_t_yaw], yaw[valid_t_yaw], linewidth=1.2)
    ax_yaw.set_xlabel("Time (s)")
    ax_yaw.set_ylabel("Yaw rate (rad/s)")
    ax_yaw.set_title("Yaw Angular Velocity")
    ax_yaw.grid(True, linewidth=0.5, alpha=0.6)

    # Bottom-right: y position vs time (SIGNED; no abs)
    ax_y = fig.add_subplot(gs[2, 1])
    ax_y.plot(t[valid_t_y], y[valid_t_y], linewidth=1.2)
    ax_y.set_xlabel("Time (s)")
    ax_y.set_ylabel("Lateral y (m)")
    ax_y.set_title("Lateral Position y (signed)")
    ax_y.grid(True, linewidth=0.5, alpha=0.6)

    # Optional: tighten limits a bit if data present
    if valid_xy.any():
        ax_traj.relim(); ax_traj.autoscale()
    if np.isfinite(aruco_x).any():
        ax_arx.relim(); ax_arx.autoscale()
    if valid_t_yaw.any():
        ax_yaw.relim(); ax_yaw.autoscale()
    if valid_t_y.any():
        ax_y.relim(); ax_y.autoscale()

    fig.suptitle("Quadruped Run — Trajectory & Signals", y=0.98)
    if args.save:
        out = args.save if args.save.lower().endswith(".png") else f"{args.save}.png"
        plt.savefig(out, dpi=200, bbox_inches="tight")
        print(f"Saved figure to: {out}")
    else:
        plt.show()

if __name__ == "__main__":
    main()
