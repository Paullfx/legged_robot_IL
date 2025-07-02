import pandas as pd
import matplotlib.pyplot as plt
import argparse, os

# python plot_IMU.py ~/data/quadruped_walk/exp_50hz_20250615_160244/all_topics.csv
# /home/fuxiao/data/quadruped_walk/exp_50hz_20250615_160244/all_topics.csv

parser = argparse.ArgumentParser()
parser.add_argument('csv_path', help='imu_foot_force.csv 路径')
args = parser.parse_args()

assert os.path.isfile(args.csv_path), "CSV 不存在"

# 读入
df = pd.read_csv(args.csv_path)
time = pd.to_datetime(df['timestamp'])
# 将时间戳转化为相对秒
t_sec = (time - time.iloc[0]).dt.total_seconds()

# ==== 绘图 ====
# 1. IMU quaternion
plt.figure()
for i in range(4):
    plt.plot(t_sec, df[f'imu_quat{i}'], label=f'q{i}')
plt.title('IMU Quaternion')
plt.xlabel('time (s)'); plt.legend(); plt.tight_layout()

# 2. Gyro
plt.figure()
for i in range(3):
    plt.plot(t_sec, df[f'imu_gyro{i}'], label=f'gyro{i}')
plt.title('Gyroscope (rad/s)')
plt.xlabel('time (s)'); plt.legend(); plt.tight_layout()

# 3. Acc
plt.figure()
for i in range(3):
    plt.plot(t_sec, df[f'imu_acc{i}'], label=f'acc{i}')
plt.title('Accelerometer (m/s²)')
plt.xlabel('time (s)'); plt.legend(); plt.tight_layout()

# 4. RPY
plt.figure()
for i in range(3):
    plt.plot(t_sec, df[f'imu_rpy{i}'], label=f'rpy{i}')
plt.title('IMU RPY (rad)')
plt.xlabel('time (s)'); plt.legend(); plt.tight_layout()

# 5. Foot Force
plt.figure()
for i in range(4):
    plt.plot(t_sec, df[f'foot_force{i}'], label=f'FF{i}')
plt.title('Foot Force (N)')
plt.xlabel('time (s)'); plt.legend(); plt.tight_layout()

# 6. Foot Force Est
plt.figure()
for i in range(4):
    plt.plot(t_sec, df[f'foot_force_est{i}'], label=f'FF_est{i}')
plt.title('Foot Force Estimated (N)')
plt.xlabel('time (s)'); plt.legend(); plt.tight_layout()

plt.show()
