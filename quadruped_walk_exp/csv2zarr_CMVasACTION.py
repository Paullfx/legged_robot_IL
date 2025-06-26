import os
import numpy as np
import pandas as pd
import zarr
from termcolor import cprint   

ROOT_DIR = "/home/fuxiao/data/quadruped_walk" #/home/hamilton/data/quadruped_walk #/home/fuxiao/data/quadruped_walk
ZARR_PATH = os.path.join(ROOT_DIR, "quadruped_walk_5hz_CMVasACTION.zarr")  # file path for the zarr results
MOTOR_IDX = list(range(12))
Q_COLS = [f'low_m{i}_q' for i in MOTOR_IDX]
ACTION_COLS = ["sport_vel0", "sport_vel1", "sport_vel2", "sport_yaw_speed"]

def find_demo_folders(root_dir):
    """
    找到所有以 'exp', 'exp_50hz' 开头，且包含 'all_topics.csv' 的目录
    """
    folders = []
    for folder in os.listdir(root_dir):
        folder_path = os.path.join(root_dir, folder)
        if not os.path.isdir(folder_path):
            continue
        if folder.startswith("exp_50hz") or folder.startswith("exp"):
            csv_path = os.path.join(folder_path, 'all_topics.csv')
            if os.path.isfile(csv_path):
                folders.append((folder, folder_path, csv_path))
    return folders

def infer_sampling_rate(foldername):
    """
    从文件夹名推断采样率
    """
    if foldername.startswith("exp_50hz"):
        return 50
    elif foldername.startswith("exp_"):
        return 20
    else:
        return None

def main():
    folders = find_demo_folders(ROOT_DIR)
    print(f"共找到{len(folders)}个有效数据文件夹。")

    all_agent_pose = []
    all_action = []
    episode_ends = []
    total_count = 0

    for foldername, folder_path, csv_path in folders:
        sampling_rate = infer_sampling_rate(foldername)
        if sampling_rate is None:
            print(f"[Warning] 未识别采样率，跳过: {foldername}")
            continue
        stride = int(np.round(sampling_rate / 5)) # 4Hz或5Hz
        print(f"处理 {csv_path}，原采样率: {sampling_rate} Hz，降采样步长: {stride}")

        try:
            df = pd.read_csv(csv_path, engine="python")
            df.columns = [c.strip() for c in df.columns]
        except Exception as e:
            print(f"[Error] 读取csv失败: {csv_path}, 原因: {e}")
            continue

        missing = [col for col in Q_COLS if col not in df.columns]
        if missing:
            print(f"[Error] 缺少关节列: {missing}，跳过。")
            continue

        df_down = df.iloc[::stride].reset_index(drop=True)
        q_arr = df_down[Q_COLS].values.astype(np.float32)
        if len(q_arr) < 2:
            print(f"[Warning] {csv_path} 降采样后数据太少，跳过。")
            continue

        action_arr = df_down[ACTION_COLS].values.astype(np.float32)    




        agent_pose = q_arr[:-1]  # (N-1, 12)
        action = action_arr[1:]       # (N-1, 4)
        all_agent_pose.append(agent_pose)
        all_action.append(action)
        total_count += agent_pose.shape[0]
        episode_ends.append(total_count)  # 累加，每条轨迹的结尾index（不含）

    # 合并全部数据
    if len(all_agent_pose) == 0:
        print("无有效数据，未生成zarr。")
        return

    all_agent_pose = np.concatenate(all_agent_pose, axis=0)
    all_action = np.concatenate(all_action, axis=0)
    episode_ends = np.array(episode_ends, dtype=np.int64)

    print(f"合并后总样本数: {all_agent_pose.shape[0]}，轨迹数: {len(episode_ends)}")

    # 写入zarr
    zarr_root = zarr.open(ZARR_PATH, mode='w')

    # ==== 创建 data group ====
    zarr_data = zarr_root.create_group('data')
    zarr_data.create_dataset('state', data=all_agent_pose, chunks=(100, all_agent_pose.shape[1]), dtype='float32', overwrite=True)
    zarr_data.create_dataset('action', data=all_action, chunks=(100, all_action.shape[1]), dtype='float32', overwrite=True)

    # ==== 创建 meta group ====
    zarr_meta = zarr_root.create_group('meta')
    zarr_meta.create_dataset('episode_ends', data=episode_ends, chunks=(100,), dtype='int64', overwrite=True)

    # 如有需要还可以加其它meta属性
    zarr_meta.attrs['sampling_rate'] = 4


    print(f"已写入：{ZARR_PATH}")

    # ==== 结果打印 ====
    cprint(f'agent_pose shape: {all_agent_pose.shape}, range: [{np.min(all_agent_pose)}, {np.max(all_agent_pose)}]', 'green')
    cprint(f'action shape: {all_action.shape}, range: [{np.min(all_action)}, {np.max(all_action)}]', 'green')
    cprint(f'episode_ends shape: {episode_ends.shape}, range: [{np.min(episode_ends)}, {np.max(episode_ends)}]', 'green')
    cprint(f'total_count: {total_count}', 'green')
    cprint(f'Saved zarr file to {ZARR_PATH}', 'green')


if __name__ == "__main__":
    main()
