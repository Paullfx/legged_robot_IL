#!/usr/bin/env python3
import argparse
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

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
    col_x = f'{prefix}_pos1'
    col_y = f'{prefix}_pos0'
    if col_x not in df.columns or col_y not in df.columns:
        for p in ['sport', 'lf_sport']:
            if f'{p}_pos0' in df.columns and f'{p}_pos1' in df.columns:
                col_x, col_y = f'{p}_pos1', f'{p}_pos0'
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

# -------------------- 作图：图1（单独的轨迹图） --------------------

def plot_figure1_traj(x, y, t, out_path, a4=False, dpi=300):
    # 平移使起点为(0,0)
    if len(x):
        x_shifted = x - x[0]
        y_shifted = y - y[0]
    else:
        x_shifted, y_shifted = x, y


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
        'xtick.labelsize': base_fontsize * scale_factor,
        'ytick.labelsize': base_fontsize * scale_factor
    })

    fig, ax = plt.subplots(figsize=figsize, constrained_layout=True)
    ax.plot(y_shifted, x_shifted, '-o', markersize=2, linewidth=2.5)
    if len(x_shifted):
        ax.scatter([0], [0], c='green', s=100, label='start', zorder=3)
        ax.scatter([y_shifted[-1]], [x_shifted[-1]], c='red', s=100, label='end', zorder=3)

        # x=0.25 m 的水平参考线（纵轴是 x）
        ax.axhline(0.25, color='gray', linestyle='--', linewidth=1)

        # 自适应轴限，留一点边距
        pad_x = max(0.05, 0.05*np.nanmax(y_shifted) if len(y_shifted) else 0.05)
        pad_y = max(0.1, 0.05*np.nanmax(np.abs(x_shifted)) if len(x_shifted) else 0.1)
        ax.set_xlim(0.0, (np.nanmax(y_shifted) if len(y_shifted) else 0.75) + pad_x)
        ax.set_ylim(min(0.0, np.nanmin(x_shifted) - pad_y),
                    (np.nanmax(x_shifted) if len(x_shifted) else 7.0) + pad_y)

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

# -------------------- 作图：图2（三个子图合在一页） --------------------

def plot_figure2_metrics(t_rpy0, rpy0, t_yaw, yaw_vel, t_all, y_shifted_signed,
                         out_path, clip_time=60.0, a4=False, dpi=300):
    # 版式与字体（全局 1.2x 放大，尽量铺满 A4）
    import matplotlib as mpl
    from matplotlib.gridspec import GridSpec

    if a4:
        figsize = (11.69*0.8, 8.27)  # A4 landscape
        base_fontsize = 14
    else:
        figsize = (16, 10)
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

    # 裁剪
    mask_rpy0 = t_rpy0 <= clip_time
    mask_yaw  = t_yaw  <= clip_time
    mask_y    = t_all  <= clip_time

    t_rpy0_c = t_rpy0[mask_rpy0];  rpy0_c   = rpy0[mask_rpy0]
    t_yaw_c  = t_yaw [mask_yaw ];  yaw_c    = yaw_vel[mask_yaw]
    t_y_c    = t_all [mask_y   ];  y_c      = y_shifted_signed[mask_y]

    fig = plt.figure(figsize=figsize, constrained_layout=True)
    gs  = GridSpec(3, 1, figure=fig, height_ratios=[1,1,1])

    # 1) IMU rpy0
    ax_rpy0 = fig.add_subplot(gs[0, 0])
    ax_rpy0.plot(t_rpy0_c, rpy0_c, linewidth=2.5, label='imu_rpy0')
    ax_rpy0.axhline( 0.070, color='r', linestyle='--', linewidth=1.5, label='+0.070')
    ax_rpy0.axhline(-0.070, color='b', linestyle='--', linewidth=1.5, label='-0.070')
    ax_rpy0.set_xlim(0, clip_time)
    ax_rpy0.set_ylabel('imu_rpy0 (rad)')
    ax_rpy0.set_title('IMU roll angle (rpy0)')
    ax_rpy0.grid(True, alpha=0.4)
    ax_rpy0.legend(loc='upper center', bbox_to_anchor=(0.5, -0.18), ncol=3, frameon=False)

    # 2) Yaw velocity
    ax_yaw = fig.add_subplot(gs[1, 0])
    ax_yaw.plot(t_yaw_c, yaw_c, linewidth=2.5, label='yaw_velocity')
    ax_yaw.set_xlim(0, clip_time)
    ax_yaw.set_ylabel('yaw (rad/s)')
    ax_yaw.set_title('Yaw Velocity')
    ax_yaw.grid(True, alpha=0.4)
    ax_yaw.legend(loc='upper center', bbox_to_anchor=(0.5, -0.18), ncol=2, frameon=False)

    # 3) y (pos1) vs time（有符号 y）
    ax_y = fig.add_subplot(gs[2, 0])
    ax_y.plot(t_y_c, np.abs(y_c), linewidth=2.5, label='y (pos1) - y[0]')
    ax_y.axhline(0, color='g', linestyle='--', linewidth=1.2, label='start y[0]')
    ax_y.set_xlim(0, clip_time)
    ax_y.set_xlabel('Time (s)')
    ax_y.set_ylabel('y (m)')
    ax_y.set_title('y position (pos1) over time (start at 0)')
    ax_y.grid(True, alpha=0.4)
    ax_y.legend(loc='upper center', bbox_to_anchor=(0.5, -0.25), ncol=2, frameon=False)

    fig.savefig(out_path, dpi=dpi, bbox_inches='tight')
    print('Saved Figure 2 (three subplots) to', out_path)
    try:
        plt.show()
    except Exception:
        pass

# -------------------- 主流程 --------------------

def main():
    p = argparse.ArgumentParser(description='Make two figures: (1) trajectory-only, (2) three subplots on A4.')
    p.add_argument('--csv', '-c', help='path to CSV (defaults to latest under ~/data/quadruped_walk_2)')
    p.add_argument('--out1', help='output path for Figure 1 (trajectory). Default: <csv>_traj.png')
    p.add_argument('--out2', help='output path for Figure 2 (metrics). Default: <csv>_metrics.png')
    p.add_argument('--clip', type=float, default=60.0, help='Time window (s) for metrics figure. Default 60.')
    p.add_argument('--a4', action='store_true', help='Use A4 landscape size and print-friendly layout.')
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

    # 额外传感器
    try:
        t_rpy0, rpy0, t_yaw, yaw_vel = load_extra(df)
    except Exception as e:
        print('Warning:', e)
        t_rpy0, rpy0, t_yaw, yaw_vel = np.array([]), np.array([]), np.array([]), np.array([])

    # y 的有符号位移（与右下子图标题一致）
    y_shifted_signed = y - y[0] if len(y) else y

    # 输出路径
    base = os.path.splitext(csv_path)[0]
    out1 = args.out1 or base + '_traj.png'
    out2 = args.out2 or base + '_metrics.png'

    # 图1：单独轨迹（无时间戳）
    plot_figure1_traj(x, y, t, out1, a4=args.a4, dpi=args.dpi)
    # 图2：三子图
    clip_time = args.clip if args.clip is not None else 60.0
    plot_figure2_metrics(t_rpy0, rpy0, t_yaw, yaw_vel, t, y_shifted_signed,
                         out2, clip_time=clip_time, a4=args.a4, dpi=args.dpi)

if __name__ == '__main__':
    main()
