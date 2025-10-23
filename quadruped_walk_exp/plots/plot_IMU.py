import pandas as pd
import matplotlib.pyplot as plt
import argparse, os

# Usage:
# python plot_IMU.py /path/to/all_topics.csv

parser = argparse.ArgumentParser()
parser.add_argument('csv_path', help='all_topics.csv 路径')
args = parser.parse_args()

assert os.path.isfile(args.csv_path), "CSV 不存在"

df = pd.read_csv(args.csv_path)
time = pd.to_datetime(df['timestamp'])
t_sec = (time - time.iloc[0]).dt.total_seconds()

# IMU相关传感器
sensor_groups = {
    'imu_quat': 4,
    'imu_gyro': 3,
    'imu_acc': 3,
    'imu_rpy': 3,
}

prefixes = ['low', 'sport']

# 计算总subplot数
total_subplots = sum(sensor_groups.values())
fig, axs = plt.subplots(total_subplots, 1, figsize=(10, 2.5 * total_subplots), sharex=True)

subplot_idx = 0
for sensor, n in sensor_groups.items():
    for i in range(n):
        ax = axs[subplot_idx]
        for prefix in prefixes:
            col = f'{prefix}_{sensor}{i}'
            if col in df.columns:
                ax.plot(t_sec, df[col], label=prefix)
        ax.set_ylabel(f'{sensor}{i}')
        ax.legend()
        ax.grid(True)
        subplot_idx += 1

fig.suptitle('IMU Sensors Comparison (low vs sport)', fontsize=16)
plt.xlabel('time (s)')
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.show()
