#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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

def _get_time_array(df):
    if 'timestamp' in df.columns:
        try:
            t = pd.to_datetime(df['timestamp'], errors='coerce')
            return (t - t.iloc[0]).dt.total_seconds().to_numpy()
        except Exception:
            return np.arange(len(df))
    return np.arange(len(df))

def load_dp_cmd_series(df):
    """
    读取 dp/cmd 相关列：
      dp_ang_z, cmd_vel_ang_z, dp_lin_x, dp_lin_y
    返回：t, dict(series_name -> (t_masked, y_masked))
    """
    t_all = _get_time_array(df)

    out = {}
    def fetch(col):
        if col in df.columns:
            y = pd.to_numeric(df[col], errors='coerce').to_numpy()
            mask = np.isfinite(y)
            return t_all[mask], y[mask]
        else:
            return np.array([]), np.array([])

    out['dp_ang_z']      = fetch('dp_ang_z')
    out['cmd_vel_ang_z'] = fetch('cmd_vel_ang_z')
    out['dp_lin_x']      = fetch('dp_lin_x')
    out['dp_lin_y']      = fetch('dp_lin_y')
    return out

# -------------------- 作图：图1（单独的轨迹图） --------------------

def plot_figure1_traj(x, y, t, out_path, a4=False, dpi=300):
    # 平移使起点为(0,0)
    if len(x):
        x_shifted = x - x[0]
        y_shifted = y - y[0]
    else:
        x_shifted, y_shifted = x, y

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
        'xtick.labelsize': base_fontsize * 0.6,
        'ytick.labelsize': base_fontsize 
    })

    fig, ax = plt.subplots(figsize=figsize, constrained_layout=True)
    ax.plot(y_shifted, x_shifted, '-o', markersize=2, linewidth=2.5)

    if len(x_shifted):
        # 起点/终点
        ax.scatter([0], [0], s=100, label='start', zorder=3)
        ax.scatter([y_shifted[-1]], [x_shifted[-1]], s=100, label='end', zorder=3)

        # --- 关键修复：两侧留白的自适应轴限（支持负值） ---
        xs = np.asarray(y_shifted, dtype=float)  # 横轴是 y_shifted
        ys = np.asarray(x_shifted, dtype=float)  # 纵轴是 x_shifted
        x_min, x_max = np.nanmin(xs), np.nanmax(xs)
        y_min, y_max = np.nanmin(ys), np.nanmax(ys)

        # 避免 min==max 的退化情况
        if not np.isfinite(x_min) or not np.isfinite(x_max) or x_min == x_max:
            x_min, x_max = -0.1, 0.1
        if not np.isfinite(y_min) or not np.isfinite(y_max) or y_min == y_max:
            y_min, y_max = -0.1, 0.1

        # 留白按范围的 5%
        x_pad = 0.05 * (x_max - x_min)
        y_pad = 0.05 * (y_max - y_min)
        ax.set_xlim(x_min - x_pad, x_max + x_pad)
        ax.set_ylim(y_min - y_pad, y_max + y_pad)

        # 参考线：目标前进 0.25 m（纵轴是 x_shifted）
        ax.axhline(0.25, linestyle='--', linewidth=1)

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('y (m)  [shifted]')
    ax.set_ylabel('x (m)  [shifted]')
    ax.set_title('Trajectory')
    ax.grid(True)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=2, frameon=False)

    fig.savefig(out_path, dpi=dpi, bbox_inches='tight')
    print('Saved Figure 1 (trajectory) to', out_path)
    try:
        plt.show()
    except Exception:
        pass


# -------------------- 作图：图2（左 3 图 + 右 2 图） --------------------

def plot_figure2_metrics(t_rpy0, rpy0, t_yaw, yaw_vel,
                         t_all, y_shifted_signed,
                         dp_cmd_dict,
                         out_path, clip_time=60.0, a4=False, dpi=300):
    import matplotlib as mpl
    from matplotlib.gridspec import GridSpec

    # --- layout & typography ---
    if a4:
        # A4 portrait works better for a tall single column
        figsize = (8.27, 11.69)   # A4 portrait
        base_fontsize = 12
    else:
        figsize = (10, 14)        # tall single column
        base_fontsize = 16

    scale_factor = 1.05
    mpl.rcParams.update({
        'font.size': base_fontsize * scale_factor,
        'axes.titlesize': base_fontsize * scale_factor * 1.05,
        'axes.labelsize': base_fontsize * scale_factor,
        'legend.fontsize': base_fontsize * scale_factor * 0.95,
        'xtick.labelsize': base_fontsize * scale_factor * 0.9,
        'ytick.labelsize': base_fontsize * scale_factor * 0.9
    })

    # --- clip series by time window ---
    def clip_by_time(tt, yy):
        if tt.size == 0 or yy.size == 0:
            return tt, yy
        m = tt <= clip_time
        return tt[m], yy[m]

    # sensor series
    t_rpy0_c, rpy0_c = clip_by_time(t_rpy0, rpy0)
    t_yaw_c,  yaw_c  = clip_by_time(t_yaw,  yaw_vel)
    t_y_c,    y_c    = clip_by_time(t_all,  y_shifted_signed)

    # dp/cmd series
    def fetch(name):
        return dp_cmd_dict.get(name, (np.array([]), np.array([])))
    t_dp_ang,  dp_ang  = fetch('dp_ang_z')
    t_cmd_ang, cmd_ang = fetch('cmd_vel_ang_z')
    t_dp_x,    dp_x    = fetch('dp_lin_x')
    t_dp_y,    dp_y    = fetch('dp_lin_y')

    t_dp_ang_c,  dp_ang_c  = clip_by_time(t_dp_ang,  dp_ang)
    t_cmd_ang_c, cmd_ang_c = clip_by_time(t_cmd_ang, cmd_ang)
    t_dp_x_c,    dp_x_c    = clip_by_time(t_dp_x,    dp_x)
    t_dp_y_c,    dp_y_c    = clip_by_time(t_dp_y,    dp_y)

    # --- figure: single column of 5 stacked axes ---
    fig = plt.figure(figsize=figsize, constrained_layout=True)
    gs = GridSpec(nrows=5, ncols=1, figure=fig, hspace=0.35)

    # 1) IMU roll (rpy0)
    ax1 = fig.add_subplot(gs[0, 0], sharex=None)
    if t_rpy0_c.size and rpy0_c.size:
        ax1.plot(t_rpy0_c, rpy0_c, linewidth=2.0, label='imu_rpy0')
    ax1.axhline( 0.070, linestyle='--', linewidth=1.0, label='+0.070')
    ax1.axhline(-0.070, linestyle='--', linewidth=1.0, label='-0.070')
    ax1.set_xlim(0, clip_time)
    ax1.set_ylabel('rpy0 (rad)')
    ax1.set_title('IMU roll angle (rpy0)')
    ax1.grid(True, alpha=0.4)
    ax1.legend(loc='upper center', bbox_to_anchor=(0.5, -0.22), ncol=3, frameon=False)

    # 2) Yaw velocity (sensor vs DP vs cmd)
    ax2 = fig.add_subplot(gs[1, 0], sharex=None)
    handles, labels = [], []
    if t_yaw_c.size and yaw_c.size:
        h1 = ax2.plot(t_yaw_c, yaw_c, linewidth=2.0, label='yaw_velocity (sensor)')[0]
        handles.append(h1); labels.append(h1.get_label())
    if t_dp_ang_c.size and dp_ang_c.size:
        h2 = ax2.plot(t_dp_ang_c, dp_ang_c, linewidth=1.8, label='dp_ang_z')[0]
        handles.append(h2); labels.append(h2.get_label())
    if t_cmd_ang_c.size and cmd_ang_c.size:
        h3 = ax2.plot(t_cmd_ang_c, cmd_ang_c, linewidth=1.8, label='cmd_vel_ang_z')[0]
        handles.append(h3); labels.append(h3.get_label())
    ax2.set_xlim(0, clip_time)
    ax2.set_ylabel('yaw (rad/s)')
    ax2.set_title('Yaw Velocity (sensor vs DP vs cmd)')
    ax2.grid(True, alpha=0.4)
    if handles:
        ax2.legend(handles, labels, loc='upper center',
                   bbox_to_anchor=(0.5, -0.22), ncol=3, frameon=False)

    # 3) |y - y0| over time
    ax3 = fig.add_subplot(gs[2, 0], sharex=None)
    if t_y_c.size and y_c.size:
        ax3.plot(t_y_c, np.abs(y_c), linewidth=2.0, label='|y - y0|')
    ax3.axhline(0, linestyle='--', linewidth=1.0, label='start y0')
    ax3.set_xlim(0, clip_time)
    ax3.set_ylabel('y (m)')
    ax3.set_title('y position over time (start at 0)')
    ax3.grid(True, alpha=0.4)
    ax3.legend(loc='upper center', bbox_to_anchor=(0.5, -0.28), ncol=2, frameon=False)

    # 4) dp_lin_x
    ax4 = fig.add_subplot(gs[3, 0], sharex=None)
    if t_dp_x_c.size and dp_x_c.size:
        ax4.plot(t_dp_x_c, dp_x_c, linewidth=2.0, label='dp_lin_x')
    ax4.set_xlim(0, clip_time)
    ax4.set_ylabel('m/s')
    ax4.set_title('DP linear x')
    ax4.grid(True, alpha=0.4)
    ax4.legend(loc='upper center', bbox_to_anchor=(0.5, -0.22), ncol=1, frameon=False)
    # ax4.set_visible(False)

    # 5) dp_lin_y
    ax5 = fig.add_subplot(gs[4, 0], sharex=None)
    if t_dp_y_c.size and dp_y_c.size:
        ax5.plot(t_dp_y_c, dp_y_c, linewidth=2.0, label='dp_lin_y')
    ax5.set_xlim(0, clip_time)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('m/s')
    ax5.set_title('DP linear y')
    ax5.grid(True, alpha=0.4)
    ax5.legend(loc='upper center', bbox_to_anchor=(0.5, -0.22), ncol=1, frameon=False)

    fig.savefig(out_path, dpi=dpi, bbox_inches='tight')
    print('Saved Figure 2 (metrics, single-column x5) to', out_path)
    try:
        plt.show()
    except Exception:
        pass


# -------------------- 主流程 --------------------

def main():
    p = argparse.ArgumentParser(description='Make two figures: (1) trajectory-only, (2) metrics with left-3 + right-2 subplots.')
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

    # dp/cmd 系列
    dp_cmd = load_dp_cmd_series(df)

    # y 的有符号位移（与图注一致）
    y_shifted_signed = y - y[0] if len(y) else y

    # 输出路径
    base = os.path.splitext(csv_path)[0]
    out1 = args.out1 or base + '_traj.png'
    out2 = args.out2 or base + '_metrics.png'

    # 图1：单独轨迹（无时间戳）
    plot_figure1_traj(x, y, t, out1, a4=args.a4, dpi=args.dpi)

    # 图2：左 3 + 右 2
    clip_time = args.clip if args.clip is not None else 60.0
    plot_figure2_metrics(t_rpy0, rpy0, t_yaw, yaw_vel,
                         t, y_shifted_signed,
                         dp_cmd,
                         out2, clip_time=clip_time, a4=args.a4, dpi=args.dpi)

if __name__ == '__main__':
    main()
