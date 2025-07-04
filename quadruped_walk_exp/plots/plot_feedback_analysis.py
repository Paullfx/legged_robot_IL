import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# -----------------------------------------------------------------------------
# User‑tunable parameters
CSV_FILES = [
    # Path.home() / "data/quadruped_walk_2/exp_50hz_20250625_111809/all_topics.csv",
    Path.home() / "data/quadruped_walk_2/exp_50hz_20250625_112636/all_topics.csv",
    # Path.home() / "data/quadruped_walk_2/exp_50hz_20250625_114212/all_topics.csv",
    # Path.home() / "data/quadruped_feedback_test/exp_50hz_20250702_184152/all_topics.csv",
    # Path.home() / "data/quadruped_feedback_test/exp_50hz_20250702_184339/all_topics.csv",
    # Path.home() / "data/quadruped_feedback_test/exp_50hz_20250702_201312/all_topics.csv",
    Path.home() / "data/quadruped_feedback_test/exp_50hz_20250702_203443/all_topics.csv",
]

COLS_TO_PLOT = [
    "sport_pos0", "sport_vel0", "sport_vel1",
    "low_imu_gyro0", "low_imu_gyro1", "low_imu_gyro2",
    "low_imu_acc0", "low_imu_acc1", "low_imu_acc2",
    "low_imu_rpy0", "low_imu_rpy1", "low_imu_rpy2"
    # "low_foot_force0", "low_foot_force1", "low_foot_force2", "low_foot_force3"
    # "low_foot_force_est0", "low_foot_force_est1", "low_foot_force_est2", "low_foot_force_est3"
    # "sport_foot_force0", "sport_foot_force2", "sport_foot_force3", "sport_foot_force1",
]
THRESH   = 0.005                                   # change magnitude
WIN      = pd.Timedelta(milliseconds=100)         # 0.1‑second window
TS_FMT   = "%Y-%m-%d %H:%M:%S.%f"                  # timestamp format in CSV
# -----------------------------------------------------------------------------


def find_first_event_time(df: pd.DataFrame) -> pd.Timestamp | None:
    """
    Find the timestamp of the first significant change in 'sport_pos0'.
    """
    col = "sport_pos0"
    if col not in df.columns:
        print(f"⚠️  Column '{col}' not found for threshold check.")
        return None

    # Shift the timeline forward by 0.1 s and compare
    later = df[["timestamp", col]].copy()
    later["timestamp"] += WIN
    merged = pd.merge_asof(df, later, on="timestamp",
                           direction="forward", suffixes=("", "_later"))
    diff = (merged[f"{col}_later"] - merged[col]).abs()
    mask = diff > THRESH
    if mask.any():
        return merged.loc[mask, "timestamp"].iloc[0]
    return None


def load_and_trim(csv_path: Path) -> pd.DataFrame | None:
    """Load data and trim it to start from the first significant event."""
    try:
        df = pd.read_csv(csv_path, usecols=["timestamp"] + COLS_TO_PLOT)
    except ValueError: # Some columns might not exist
        df = pd.read_csv(csv_path)
        df = df[[col for col in ["timestamp"] + COLS_TO_PLOT if col in df.columns]]

    df["timestamp"] = pd.to_datetime(df["timestamp"], format=TS_FMT, errors="coerce")
    df = df.dropna(subset=["timestamp"]).sort_values("timestamp").reset_index(drop=True)

    t0 = find_first_event_time(df)
    if t0 is None:
        print(f"⚠️  {csv_path.parent.name} never exceeds ±{THRESH} within {WIN.total_seconds()}s — skipped.")
        return None

    # Align to the beginning of that 0.1‑second bin
    align_start = (t0.floor("100ms"))
    df = df[df["timestamp"] >= align_start].copy()
    df["t_sec"] = (df["timestamp"] - align_start).dt.total_seconds()
    return df


# Create subplots
n_cols = 3
n_rows = int(np.ceil(len(COLS_TO_PLOT) / n_cols))
fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, n_rows * 3), sharex=True)
axes = axes.flatten()
found_any = False

for csv_file in CSV_FILES:
    if not csv_file.exists():
        print(f"⚠️  {csv_file} not found.")
        continue
    try:
        df = load_and_trim(csv_file)
    except Exception as e:
        print(f"⚠️  Error processing {csv_file}: {e}")
        continue
    if df is None:
        continue

    found_any = True
    label = f"{csv_file.parent.parent.name}/{csv_file.parent.name}"
    for i, col in enumerate(COLS_TO_PLOT):
        if col in df.columns:
            axes[i].plot(df["t_sec"], df[col], label=label)

if found_any:
    for i, col in enumerate(COLS_TO_PLOT):
        axes[i].set_title(col)
        axes[i].set_ylabel("Value")
        axes[i].grid(True, linestyle='--', alpha=0.6)

    for i in range(n_rows * n_cols - len(COLS_TO_PLOT)):
        fig.delaxes(axes[-(i+1)])

    # Common settings
    plt.suptitle(f"Aligned at first ≥{THRESH} change in 'sport_pos0' within {WIN.total_seconds()}s", fontsize=16)
    plt.xlabel("Aligned time (s)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right', fontsize='small', title="Experiment")
    plt.tight_layout(rect=[0, 0, 0.9, 0.96]) # Adjust layout to make space for suptitle and legend
    plt.show()
else:
    print("❌  Nothing was plotted. Check file paths or data validity.")
