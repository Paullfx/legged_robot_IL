#!/usr/bin/env python3
import argparse
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
# python3 odom_trj_time_clip.py --csv your.csv --clip 60

def find_latest_csv(search_dir=None):
    if search_dir is None:
        search_dir = os.path.expanduser('~/data/quadruped_walk_2')
    pattern = os.path.join(search_dir, '**', '*.csv')
    matches = glob.glob(pattern, recursive=True)
    if not matches:
        return None
    matches.sort(key=os.path.getmtime, reverse=True)
    return matches[0]

def load_positions(csv_path, prefix='sport'):
    df = pd.read_csv(csv_path, low_memory=False)
    # Use original mapping: pos0 for x (forward), pos1 for y (lateral)
    col_x = f'{prefix}_pos0'
    col_y = f'{prefix}_pos1'
    col_time = 'timestamp' if 'timestamp' in df.columns else None
    if col_x not in df.columns or col_y not in df.columns:
        for p in ['sport', 'lf_sport']:
            if f'{p}_pos0' in df.columns and f'{p}_pos1' in df.columns:
                col_x, col_y = f'{p}_pos0', f'{p}_pos1'
                break
        else:
            raise ValueError(f'Could not find position columns like \"{prefix}_pos0\" and \"{prefix}_pos1\" in {csv_path}')
    x = pd.to_numeric(df[col_x], errors='coerce').to_numpy()
    y = pd.to_numeric(df[col_y], errors='coerce').to_numpy()
    mask = np.isfinite(x) & np.isfinite(y)
    # Try to get time if available
    first_timestamp_str = None
    if col_time:
        # Try to parse timestamp as seconds (assume format: "YYYY-MM-DD HH:MM:SS.sss")
        try:
            t_full = pd.to_datetime(df[col_time], errors='coerce')
            t = (t_full - t_full.iloc[0]).dt.total_seconds().to_numpy()
            # Get first valid timestamp string
            first_valid_idx = t_full.first_valid_index()
            if first_valid_idx is not None:
                ts = t_full.iloc[first_valid_idx]
                first_timestamp_str = ts.strftime('%Y-%m-%d %H:%M:%S') if hasattr(ts, 'strftime') else str(ts)
        except Exception:
            t = np.arange(len(x))
        t = t[mask]
    else:
        t = np.arange(len(x))[mask]
    return x[mask], y[mask], t, col_x, col_y, first_timestamp_str

def load_extra(csv_path, prefix='sport'):
    df = pd.read_csv(csv_path, low_memory=False)
    # IMU rpy0
    col_rpy0 = f'{prefix}_imu_rpy0'
    # Yaw velocity
    col_yaw_vel = f'{prefix}_yaw_speed'
    col_time = 'timestamp' if 'timestamp' in df.columns else None
    if col_rpy0 not in df.columns or col_yaw_vel not in df.columns:
        for p in ['sport', 'lf_sport']:
            if f'{p}_imu_rpy0' in df.columns and f'{p}_yaw_speed' in df.columns:
                col_rpy0 = f'{p}_imu_rpy0'
                col_yaw_vel = f'{p}_yaw_speed'
                break
        else:
            raise ValueError(f'Could not find columns \"{prefix}_imu_rpy0\" and \"{prefix}_yaw_speed\" in {csv_path}')
    rpy0 = pd.to_numeric(df[col_rpy0], errors='coerce').to_numpy()
    yaw_vel = pd.to_numeric(df[col_yaw_vel], errors='coerce').to_numpy()
    mask_rpy0 = np.isfinite(rpy0)
    mask_yaw = np.isfinite(yaw_vel)
    # Use time if available
    if col_time:
        try:
            t_all = pd.to_datetime(df[col_time], errors='coerce')
            t_all = (t_all - t_all.iloc[0]).dt.total_seconds().to_numpy()
        except Exception:
            t_all = np.arange(len(rpy0))
        t_rpy0 = t_all[mask_rpy0]
        t_yaw = t_all[mask_yaw]
    else:
        t_rpy0 = np.arange(len(rpy0))[mask_rpy0]
        t_yaw = np.arange(len(yaw_vel))[mask_yaw]
    return t_rpy0, rpy0[mask_rpy0], t_yaw, yaw_vel[mask_yaw], col_rpy0, col_yaw_vel

def plot_traj(x, y, t, csv_path, out_path=None, title=None, timestamp_str=None, clip_right=None):
    # Shift so that the start point is always at (0,0)
    if len(x):
        x_shifted = x - x[0]
        y_shifted = y - y[0]
    else:
        x_shifted = x
        y_shifted = y

    # Load extra data for subplots
    try:
        t_rpy0, rpy0, t_yaw, yaw_vel, col_rpy0, col_yaw_vel = load_extra(csv_path)
    except Exception as e:
        print('Warning:', e)
        t_rpy0, rpy0, t_yaw, yaw_vel = [], [], [], []

    # Clip right-side subplots to first N seconds if requested
    if clip_right is not None:
        mask_rpy0 = t_rpy0 <= clip_right
        mask_yaw = t_yaw <= clip_right
        mask_y = t <= clip_right
        t_rpy0 = t_rpy0[mask_rpy0]
        rpy0 = rpy0[mask_rpy0]
        t_yaw = t_yaw[mask_yaw]
        yaw_vel = yaw_vel[mask_yaw]
        t_y = t[mask_y]
        y_shifted_time = (y - y[0])[mask_y]
    else:
        t_y = t
        y_shifted_time = y - y[0] if len(y) else y

    from matplotlib.gridspec import GridSpec

    fig = plt.figure(figsize=(20, 8))
    gs = GridSpec(3, 3, figure=fig, width_ratios=[1, 2, 2])

    # Trajectory subplot (left, spanning all rows)
    ax_traj = fig.add_subplot(gs[:, 0])
    # Plot with y on x-axis and x on y-axis to show horizontal motion better
    ax_traj.plot(y_shifted, x_shifted, '-o', markersize=2, linewidth=1)
    if len(x_shifted):
        ax_traj.scatter([0], [0], c='green', s=40, label='start')
        ax_traj.scatter([y_shifted[-1]], [x_shifted[-1]], c='red', s=40, label='end')
        
        # Set limits without forcing equal aspect ratio
        margin = 0.1  # 10% margin
        y_range = np.ptp(y_shifted)
        x_range = np.ptp(x_shifted)
        ax_traj.set_xlim(
            np.min(y_shifted) - margin * y_range,
            np.max(y_shifted) + margin * y_range
        )
        ax_traj.set_ylim(
            np.min(x_shifted) - margin * x_range,
            np.max(x_shifted) + margin * x_range
        )
    
    # Remove equal aspect ratio to allow natural scaling
    ax_traj.set_aspect('equal', adjustable='box')  # Comment out or remove this line
    
    # Update labels to reflect axis swap
    ax_traj.set_xlabel('y (pos0, m)')
    ax_traj.set_ylabel('x (pos1, m)')
    ax_traj.set_title('Trajectory')
    ax_traj.legend()
    ax_traj.grid(True)

    # IMU rpy0 subplot (top right)
    ax_rpy0 = fig.add_subplot(gs[0, 1:])
    ax_rpy0.plot(t_rpy0, rpy0, label='imu_rpy0')
    ax_rpy0.axhline(0.070, color='r', linestyle='--', label='+0.070')
    ax_rpy0.axhline(-0.070, color='b', linestyle='--', label='-0.070')
    ax_rpy0.set_ylabel('imu_rpy0 (rad)')
    ax_rpy0.set_title('IMU rpy0')
    ax_rpy0.legend()
    ax_rpy0.grid(True)

    # Yaw velocity subplot (middle right)
    ax_yaw = fig.add_subplot(gs[1, 1:])
    ax_yaw.plot(t_yaw, yaw_vel, label='yaw_velocity')
    ax_yaw.set_ylabel('yaw_velocity (rad/s)')
    ax_yaw.set_title('Yaw Velocity')
    ax_yaw.legend()
    ax_yaw.grid(True)

    # y (pos0) subplot (bottom right)
    ax_y = fig.add_subplot(gs[2, 1:])
    ax_y.plot(t_y, y_shifted_time, label='y (pos0) - y[0]')
    if len(y_shifted_time):
        ax_y.axhline(0, color='g', linestyle='--', label='y[0] (start, shifted to 0)')
    ax_y.set_xlabel('Time (s)')
    ax_y.set_ylabel('y (pos0, m, shifted)')
    ax_y.set_title('y (pos0) over time (start at 0)')
    ax_y.legend()
    ax_y.grid(True)

    if title is None:
        title = os.path.basename(csv_path)
    if timestamp_str:
        fig.suptitle(f'Trajectory and IMU/Yaw: {title}\nStart timestamp: {timestamp_str}', fontsize=16)
    else:
        fig.suptitle(f'Trajectory and IMU/Yaw: {title}', fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if out_path is None:
        out_path = os.path.splitext(csv_path)[0] + '_traj.png'
    plt.savefig(out_path, dpi=150)
    print('Saved plot to', out_path)
    try:
        plt.show()
    except Exception:
        pass

def main():
    p = argparse.ArgumentParser(description='Plot 2D trajectory from CSV (sportmodestate pos0,pos1).')
    p.add_argument('--csv', '-c', help='path to CSV (defaults to latest under ~/data/quadruped_walk_2)')
    p.add_argument('--out', '-o', help='output image path (optional)')
    p.add_argument('--clip', type=float, default=None, help='Only plot the first N seconds on right subplots (e.g. --clip 60)')
    args = p.parse_args()

    csv_path = args.csv or find_latest_csv()
    if csv_path is None:
        raise SystemExit('No CSV found. Provide one with --csv.')

    if os.path.isdir(csv_path):
        possible = os.path.join(csv_path, 'all_topics.csv')
        if os.path.exists(possible):
            csv_path = possible

    x, y, t, colx, coly, first_timestamp_str = load_positions(csv_path)
    if x.size == 0:
        raise SystemExit(f'No valid position data found in columns {colx},{coly} of {csv_path}')

    plot_traj(x, y, t, csv_path, out_path=args.out, title=None, timestamp_str=first_timestamp_str, clip_right=args.clip)

if __name__ == '__main__':
    main()