#!/usr/bin/env python3
"""
Plot sport_pos0 for each `all_topics.csv` whose parent folder包含20250625，
并在 **曲线起点** 上方标注 HHMMSS。
"""

import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

# ---------- CONFIG -----------------------------------------------------------
ROOT_DIR       = Path.home() / "data" / "quadruped_walk_2" # "quadruped_walk_2" "quadruped_walk"
TARGET_DAY_STR = "20250625_111809" # "20250625" # "20250615" #"20250625_111809"
TIMESTAMP_FMT  = "%Y-%m-%d %H:%M:%S.%f"
COLUMN         = "sport_vel0" #"sport_pos0" "sport_vel0"
Y_OFFSET       = 0.02        # 垂直向上偏移量，可按数据量级调整
# -----------------------------------------------------------------------------


def find_csv_files(root_dir: Path, filename="all_topics.csv"):
    return [p for p in root_dir.rglob(filename) if TARGET_DAY_STR in p.parent.name]


def load_and_prepare(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path, usecols=["timestamp", COLUMN])
    df["timestamp"] = pd.to_datetime(df["timestamp"], format=TIMESTAMP_FMT,
                                     errors="coerce")
    df = df.dropna(subset=["timestamp"]).sort_values("timestamp")
    df["t_sec"] = (df["timestamp"] - df["timestamp"].iloc[0]).dt.total_seconds()
    return df


def main():
    csv_paths = find_csv_files(ROOT_DIR)
    if not csv_paths:
        print(f"❌  未找到包含 {TARGET_DAY_STR} 的实验文件。")
        return

    plt.figure(figsize=(12, 6))

    for csv_path in sorted(csv_paths):
        folder_name = csv_path.parent.name
        hhmmss      = folder_name.split('_')[-1]

        try:
            df = load_and_prepare(csv_path)
        except Exception as e:
            print(f"⚠️  读取 {csv_path} 失败: {e}")
            continue
        if COLUMN not in df.columns:
            print(f"⚠️  缺少列 {COLUMN}，跳过 {csv_path}")
            continue

        # 画曲线
        plt.plot(df["t_sec"], df[COLUMN], label=hhmmss)

        # 起点坐标
        x0, y0 = df["t_sec"].iloc[0], df[COLUMN].iloc[0]

        # 上移一点点再标注
        plt.text(x0, y0 + Y_OFFSET, hhmmss,
                 fontsize=8, va="bottom", ha="center")

    plt.xlabel("Time (s)")
    plt.ylabel(COLUMN)
    plt.title(f"{COLUMN} over time – experiments on {TARGET_DAY_STR}")
    plt.legend(title="HHMMSS", fontsize="small", ncol=3)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
