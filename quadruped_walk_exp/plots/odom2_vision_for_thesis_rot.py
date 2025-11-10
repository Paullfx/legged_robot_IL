#!/usr/bin/env python3
import argparse
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# python3 odom2_vision.py --clip 45 --csv ~/data/quadruped_walk_2/exp_50hz_20251104_171821/
# python3 odom2_vision.py --clip 60 --csv ~/data/quadruped_walk_2/exp_50hz_20251104_173020/
# -------------------- 文件与数据载入 --------------------

def find_latest_csv(search_dir=None):
    if search_dir is None:
        search_dir = os.path.expanduser('~/data/quadruped_walk_2')
    pattern = os.path.join(search_dir, '**', '*.csv')
    matches = glob.glob(pattern, recursive=True)
    if not matches:
        return None
    matches.sort(key=os.path.getmtime, reverse=True)
    return matches[0]

def load_positions(df, prefix='sport'):
    # Use original mapping: pos0 for x (forward), pos1 for y (lateral)
    col_x = f'{prefix}_pos0'
    col_y = f'{prefix}_pos1'
    if col_x not in df.columns or col_y not in df.columns:
        for p in ['sport', 'lf_sport']:
            if f'{p}_pos0' in df.columns and f'{p}_pos1' in df.columns:
                col_x, col_y = f'{p}_pos0', f'{p}_pos1'
                break
        else:
            raise ValueError(f'Could not find position columns like "{prefix}_pos0" and "{prefix}_pos1"')
    x = pd.to_numeric(df[col_x], errors='coerce').to_numpy()
    y = pd.to_numeric(df[col_y], errors='coerce').to_numpy()
    mask = np.isfinite(x) & np.isfinite(y)

    # time
    if 'timestamp' in df.columns:
        try:
            t_full = pd.to_datetime(df['timestamp'], errors='coerce')
            t = (t_full - t_full.iloc[0]).dt.total_seconds().to_numpy()
        except Exception:
            t = np.arange(len(x))
    else:
        t = np.arange(len(x))
    return x[mask], y[mask], t[mask], col_x, col_y

def load_extra(df, prefix='sport'):
    col_rpy0 = f'{prefix}_imu_rpy0'
    col_yaw_vel = f'{prefix}_yaw_speed'
    if col_rpy0 not in df.columns or col_yaw_vel not in df.columns:
        for p in ['sport', 'lf_sport']:
            if f'{p}_imu_rpy0' in df.columns and f'{p}_yaw_speed' in df.columns:
                col_rpy0 = f'{p}_imu_rpy0'
                col_yaw_vel = f'{p}_yaw_speed'
                break
        else:
            raise ValueError(f'Could not find columns "{prefix}_imu_rpy0" and "{prefix}_yaw_speed"')

    rpy0 = pd.to_numeric(df[col_rpy0], errors='coerce').to_numpy()
    yaw_vel = pd.to_numeric(df[col_yaw_vel], errors='coerce').to_numpy()
    mask_rpy0 = np.isfinite(rpy0)
    mask_yaw  = np.isfinite(yaw_vel)

    if 'timestamp' in df.columns:
        try:
            t_all = pd.to_datetime(df['timestamp'], errors='coerce')
            t_all = (t_all - t_all.iloc[0]).dt.total_seconds().to_numpy()
        except Exception:
            t_all = np.arange(len(rpy0))
    else:
        t_all = np.arange(len(rpy0))
    return (
        t_all[mask_rpy0], rpy0[mask_rpy0],
        t_all[mask_yaw],  yaw_vel[mask_yaw]
    )

def load_vision_data(df):
    """Load ArUco vision data from CSV."""
    aruco_col = "aruco_x"
    if aruco_col not in df.columns:
        # Return zero series if not present
        aruco_x = np.zeros(len(df), dtype=float)
    else:
        # Values may be strings/NaN—coerce; missing -> 0
        aruco_x = pd.to_numeric(df[aruco_col], errors="coerce").fillna(0.0).to_numpy()
    
    # Get time for aruco data
    if 'timestamp' in df.columns:
        try:
            t_full = pd.to_datetime(df['timestamp'], errors='coerce')
            t_aruco = (t_full - t_full.iloc[0]).dt.total_seconds().to_numpy()
        except Exception:
            t_aruco = np.arange(len(aruco_x))
    else:
        t_aruco = np.arange(len(aruco_x))
    
    return t_aruco, aruco_x

def load_cmd_vel_data(df):
    """Load command velocity data from CSV."""
    cmd_vel_cols = ["cmd_vel_lin_x", "cmd_vel_lin_y", "cmd_vel_ang_z"]
    cmd_vel_data = {}
    
    for col in cmd_vel_cols:
        if col not in df.columns:
            # Return zero series if not present
            cmd_vel_data[col] = np.zeros(len(df), dtype=float)
        else:
            # Values may be strings/NaN—coerce; missing -> 0
            cmd_vel_data[col] = pd.to_numeric(df[col], errors="coerce").fillna(0.0).to_numpy()
    
    # Get time for cmd_vel data
    if 'timestamp' in df.columns:
        try:
            t_full = pd.to_datetime(df['timestamp'], errors='coerce')
            t_cmd_vel = (t_full - t_full.iloc[0]).dt.total_seconds().to_numpy()
        except Exception:
            t_cmd_vel = np.arange(len(df))
    else:
        t_cmd_vel = np.arange(len(df))
    
    return t_cmd_vel, cmd_vel_data["cmd_vel_lin_x"], cmd_vel_data["cmd_vel_lin_y"], cmd_vel_data["cmd_vel_ang_z"]

# -------------------- 作图：图1（单独的轨迹图） --------------------

def plot_figure1_traj(x, y, t, out_path, a4=False, dpi=300):
    # 平移使起点为(0,0)
    if len(x):
        x_shifted = x - x[0]
        y_shifted = y - y[0]
    else:
        x_shifted, y_shifted = x, y

    # Rotate coordinates by 15 degrees clockwise
    if len(x_shifted):
        angle = 19.47 * np.pi / 180  # 15 degrees clockwise (negative angle)
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        
        # Apply rotation matrix
        x_rotated = x_shifted * cos_angle - y_shifted * sin_angle
        y_rotated = x_shifted * sin_angle + y_shifted * cos_angle
        
        # Flip y values from positive to negative
        y_rotated = -y_rotated
    else:
        x_rotated, y_rotated = x_shifted, y_shifted

    # Use the actual y_rotated values (not absolute values) to show negative values

    # 版式与字体（全局 1.2x 放大）
    import matplotlib as mpl
    if a4:
        figsize = (11.69, 8.27*0.8)  # A4 landscape
        base_fontsize = 10
    else:
        figsize = (10, 8)
        base_fontsize = 16

    scale_factor = 1.2
    mpl.rcParams.update({
        'font.size': base_fontsize * scale_factor,
        'axes.titlesize': base_fontsize * scale_factor * 1.2,
        'axes.labelsize': base_fontsize * scale_factor,
        'legend.fontsize': base_fontsize * scale_factor,
        'xtick.labelsize': base_fontsize * 0.4 ,
        'ytick.labelsize': base_fontsize 
    })

    fig, ax = plt.subplots(figsize=figsize, constrained_layout=True)
    ax.plot(y_rotated, x_rotated, '-o', markersize=2, linewidth=2.5)
    if len(x_rotated):
        ax.scatter([0], [0], c='green', s=100, label='start', zorder=3)
        ax.scatter([y_rotated[-1]], [x_rotated[-1]], c='red', s=100, label='end', zorder=3)

        # x=0.25 m 的水平参考线（纵轴是 x）
        ax.axhline(0.25, color='gray', linestyle='--', linewidth=1)

        # 自适应轴限，留一点边距
        pad_x = max(0.05, 0.05*np.nanmax(np.abs(y_rotated)) if len(y_rotated) else 0.05)
        pad_y = max(0.1, 0.05*np.nanmax(np.abs(x_rotated)) if len(x_rotated) else 0.1)
        ax.set_xlim((np.nanmin(y_rotated) if len(y_rotated) else -0.75) - pad_x,
                    (np.nanmax(y_rotated) if len(y_rotated) else 0.0) + pad_x)
        ax.set_ylim(min(0.0, np.nanmin(x_rotated) - pad_y),
                    (np.nanmax(x_rotated) if len(x_rotated) else 7.0) + pad_y)

    ax.set_aspect('equal', adjustable='box')  # 真实比例
    ax.set_xlabel('y (pos1, m)')
    ax.set_ylabel('x (pos0, m)')
    ax.set_title('Trajectory')
    ax.grid(True)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=1, frameon=False)

    # 不显示时间戳（按你的要求去掉）
    fig.savefig(out_path, dpi=dpi, bbox_inches='tight')
    print('Saved Figure 1 (trajectory) to', out_path)
    try:
        plt.show()
    except Exception:
        pass

# -------------------- 作图：图2（四个子图，单列布局） --------------------

def plot_figure2_metrics(t_yaw, yaw_vel, t_all, y_shifted_signed, t_aruco, aruco_x, 
                         t_cmd_vel, cmd_vel_lin_x, cmd_vel_lin_y, cmd_vel_ang_z,
                         out_path, clip_time=60.0, a4=False, dpi=300):
    # 版式与字体（全局 1.2x 放大，尽量铺满 A4）
    import matplotlib as mpl
    from matplotlib.gridspec import GridSpec

    if a4:
        figsize = (8.27, 11.69)  # A4 portrait
        base_fontsize = 14
    else:
        figsize = (12, 16)
        base_fontsize = 16

    scale_factor = 1.2
    mpl.rcParams.update({
        'font.size': base_fontsize * scale_factor,
        'axes.titlesize': base_fontsize * scale_factor * 1.2,
        'axes.labelsize': base_fontsize * scale_factor,
        'legend.fontsize': base_fontsize * scale_factor,
        'xtick.labelsize': base_fontsize * scale_factor,
        'ytick.labelsize': base_fontsize * scale_factor
    })

    # 裁剪所有数据到指定时间窗口
    mask_yaw  = t_yaw  <= clip_time
    mask_aruco = t_aruco <= clip_time
    mask_cmd_vel = t_cmd_vel <= clip_time

    t_yaw_c  = t_yaw [mask_yaw ];  yaw_c    = yaw_vel[mask_yaw]
    t_aruco_c = t_aruco[mask_aruco]; aruco_c = aruco_x[mask_aruco]
    t_cmd_vel_c = t_cmd_vel[mask_cmd_vel]
    cmd_vel_lin_x_c = cmd_vel_lin_x[mask_cmd_vel]
    cmd_vel_ang_z_c = cmd_vel_ang_z[mask_cmd_vel]

    # Center ArUco X around first non-zero value
    if len(aruco_c) > 0:
        # Find first non-zero value to use as center
        non_zero_indices = np.where(aruco_c != 0)[0]
        if len(non_zero_indices) > 0:
            aruco_center = aruco_c[non_zero_indices[0]]
        else:
            aruco_center = 0
        aruco_c = aruco_c - aruco_center
    
    fig = plt.figure(figsize=figsize, constrained_layout=True)
    gs  = GridSpec(4, 1, figure=fig, height_ratios=[1,1,1,1])

    # 1) ArUco X position (top)
    ax_aruco = fig.add_subplot(gs[0, 0])
    ax_aruco.plot(t_aruco_c, aruco_c, linewidth=2.5, label='aruco_x')
    ax_aruco.axhline(0, color='gray', linestyle='--', linewidth=1, alpha=0.7)
    ax_aruco.set_xlim(0, clip_time)
    ax_aruco.set_ylabel('aruco_x (m)')
    ax_aruco.set_title('ArUco X Position (centered)')
    ax_aruco.grid(True, alpha=0.4)
    ax_aruco.legend(loc='upper center', bbox_to_anchor=(0.5, -0.10), ncol=1, frameon=False)

    # 2) Command Angular Velocity Z (second)
    ax_cmd_ang = fig.add_subplot(gs[1, 0])
    ax_cmd_ang.plot(t_cmd_vel_c, cmd_vel_ang_z_c, linewidth=2.5, label='cmd_vel_ang_z')
    ax_cmd_ang.set_xlim(0, clip_time)
    ax_cmd_ang.set_ylabel('ang_z (rad/s)')
    ax_cmd_ang.set_title('Command Angular Velocity Z')
    ax_cmd_ang.grid(True, alpha=0.4)
    ax_cmd_ang.legend(loc='upper center', bbox_to_anchor=(0.5, -0.10), ncol=1, frameon=False)

    # 3) Yaw velocity (third)
    ax_yaw = fig.add_subplot(gs[2, 0])
    ax_yaw.plot(t_yaw_c, yaw_c, linewidth=2.5, label='yaw_velocity')
    ax_yaw.set_xlim(0, clip_time)
    ax_yaw.set_ylabel('yaw (rad/s)')
    ax_yaw.set_title('Yaw Velocity')
    ax_yaw.grid(True, alpha=0.4)
    ax_yaw.legend(loc='upper center', bbox_to_anchor=(0.5, -0.10), ncol=1, frameon=False)

    # 4) Command Linear Velocity X (bottom)
    ax_cmd_x = fig.add_subplot(gs[3, 0])
    ax_cmd_x.plot(t_cmd_vel_c, cmd_vel_lin_x_c, linewidth=2.5, label='cmd_vel_lin_x')
    ax_cmd_x.set_xlim(0, clip_time)
    ax_cmd_x.set_xlabel('Time (s)', labelpad=40)  # Added labelpad to move Time(s) further down
    ax_cmd_x.set_ylabel('lin_x (m/s)')
    ax_cmd_x.set_title('Command Linear Velocity X')
    ax_cmd_x.grid(True, alpha=0.4)
    ax_cmd_x.legend(loc='upper center', bbox_to_anchor=(0.5, -0.10), ncol=1, frameon=False)

    fig.savefig(out_path, dpi=dpi, bbox_inches='tight')
    print('Saved Figure 2 (four subplots single column) to', out_path)
    try:
        plt.show()
    except Exception:
        pass

# -------------------- 主流程 --------------------

def main():
    p = argparse.ArgumentParser(description='Make two figures: (1) trajectory-only, (2) four subplots in single column layout.')
    p.add_argument('--csv', '-c', help='path to CSV (defaults to latest under ~/data/quadruped_walk_2)')
    p.add_argument('--out1', help='output path for Figure 1 (trajectory). Default: <csv>_traj.png')
    p.add_argument('--out2', help='output path for Figure 2 (metrics with vision). Default: <csv>_vision_metrics.png')
    p.add_argument('--clip', type=float, default=45.0, help='Time window (s) for metrics figure. Default 45.')
    p.add_argument('--a4', action='store_true', help='Use A4 portrait size and print-friendly layout.')
    p.add_argument('--dpi', type=int, default=300, help='DPI for saved figures. Default 300.')
    args = p.parse_args()

    csv_path = args.csv or find_latest_csv()
    if csv_path is None:
        raise SystemExit('No CSV found. Provide one with --csv.')

    if os.path.isdir(csv_path):
        possible = os.path.join(csv_path, 'all_topics.csv')
        if os.path.exists(possible):
            csv_path = possible

    # 只读一次 CSV
    df = pd.read_csv(csv_path, low_memory=False)

    # 位置数据
    x, y, t, colx, coly = load_positions(df)
    if x.size == 0:
        raise SystemExit(f'No valid position data found in columns {colx},{coly} of {csv_path}')

    # 额外传感器数据（移除IMU roll）
    try:
        t_rpy0, rpy0, t_yaw, yaw_vel = load_extra(df)
    except Exception as e:
        print('Warning:', e)
        t_yaw, yaw_vel = np.array([]), np.array([])

    # 视觉数据 (ArUco)
    try:
        t_aruco, aruco_x = load_vision_data(df)
        print(f'Loaded ArUco data: {len(aruco_x)} points')
    except Exception as e:
        print('Warning loading vision data:', e)
        t_aruco, aruco_x = np.array([]), np.array([])

    # 命令速度数据
    try:
        t_cmd_vel, cmd_vel_lin_x, cmd_vel_lin_y, cmd_vel_ang_z = load_cmd_vel_data(df)
        print(f'Loaded cmd_vel data: {len(cmd_vel_lin_x)} points')
    except Exception as e:
        print('Warning loading cmd_vel data:', e)
        t_cmd_vel, cmd_vel_lin_x, cmd_vel_lin_y, cmd_vel_ang_z = np.array([]), np.array([]), np.array([]), np.array([])

    # y 的有符号位移（与右下子图标题一致）
    y_shifted_signed = y - y[0] if len(y) else y

    # 输出路径
    base = os.path.splitext(csv_path)[0]
    out1 = args.out1 or base + '_traj_rot.png'
    out2 = args.out2 or base + '_vision_for_thesis.png'

    # 图1：单独轨迹（旋转45度顺时针）
    plot_figure1_traj(x, y, t, out1, a4=args.a4, dpi=args.dpi)
    
    # 图2：四子图（单列布局）
    clip_time = args.clip if args.clip is not None else 45.0
    plot_figure2_metrics(t_yaw, yaw_vel, t, y_shifted_signed, t_aruco, aruco_x,
                         t_cmd_vel, cmd_vel_lin_x, cmd_vel_lin_y, cmd_vel_ang_z,
                         out2, clip_time=clip_time, a4=args.a4, dpi=args.dpi)

if __name__ == '__main__':
    main()
