import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# -----------------------------------------------------------------------------
# User‑tunable parameters
ROOT_DIR = Path.home() / "data" / "quadruped_walk_2"
FOLDERS_OF_INTEREST = {
    "exp_50hz_20250625_111809",
    "exp_50hz_20250625_112636",
    "exp_50hz_20250625_114212",
}
COL      = "sport_pos0" # "low_m1_q" # "sport_pos0" # "sport_vel0"
THRESH   = 0.005                                   # change magnitude
WIN      = pd.Timedelta(milliseconds=100)         # 0.1‑second window
TS_FMT   = "%Y-%m-%d %H:%M:%S.%f"                  # timestamp format in CSV
# -----------------------------------------------------------------------------


def first_sig_change(df: pd.DataFrame) -> pd.Timestamp | None:
    """
    Return the earliest timestamp ts₀ such that |q(t) − q(ts₀)| > THRESH
    for some t with 0 < t − ts₀ ≤ 0.1 s.  If none, return None.
    """
    # Shift the timeline forward by 0.1 s and compare
    later = df[["timestamp", COL]].copy()
    later["timestamp"] += WIN
    merged = pd.merge_asof(df, later, on="timestamp",
                           direction="forward", suffixes=("", "_later"))
    diff = (merged[f"{COL}_later"] - merged[COL]).abs()
    mask = diff > THRESH
    return merged.loc[mask, "timestamp"].iloc[0] if mask.any() else None


def load_and_trim(csv_path: Path) -> pd.DataFrame | None:
    df = pd.read_csv(csv_path, usecols=["timestamp", COL])
    df["timestamp"] = pd.to_datetime(df["timestamp"], format=TS_FMT, errors="coerce")
    df = df.dropna(subset=["timestamp"]).sort_values("timestamp").reset_index(drop=True)

    t0 = first_sig_change(df)
    if t0 is None:
        print(f"⚠️  {csv_path} never exceeds ±{THRESH} within 0.1 s — skipped.")
        return None

    # Align to the beginning of that 0.1‑second bin
    align_start = (t0.floor("100ms"))
    df = df[df["timestamp"] >= align_start].copy()
    df["t_sec"] = (df["timestamp"] - align_start).dt.total_seconds()
    return df


plt.figure(figsize=(12, 6))
found_any = False

for folder in sorted(FOLDERS_OF_INTEREST):
    csv_file = ROOT_DIR / folder / "all_topics.csv"
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

    plt.plot(df["t_sec"], df[COL], label=folder)
    found_any = True

if found_any:
    plt.title(f"{COL}: aligned at first ≥{THRESH} change within 0.1 s")
    plt.xlabel("Aligned time (s)")
    plt.ylabel(COL)
    plt.legend(fontsize="small")
    plt.tight_layout()
    plt.show()
else:
    print("❌  Nothing was plotted. Check folder names, ROOT_DIR, or data validity.")
