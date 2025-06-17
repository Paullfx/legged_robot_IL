import os
import numpy as np
import pandas as pd
import zarr

root_dir = r"C:\data\quadruped_walk"

motor_idx = list(range(12))
q_cols = [f'low_m{i}_q' for i in motor_idx]

for folder in os.listdir(root_dir):
    folder_path = os.path.join(root_dir, folder)
    if not os.path.isdir(folder_path):
        continue
    csv_path = os.path.join(folder_path, 'all_topics.csv')
    if not os.path.isfile(csv_path):
        print(f"[Warning] {csv_path} 不存在，跳过。")
        continue

    # 采样率判断
    if folder.startswith("exp_50hz"):
        sampling_rate = 50
    elif folder.startswith("exp_"):
        sampling_rate = 20
    else:
        print(f"[Warning] 未知采样率，跳过: {folder}")
        continue
    stride = int(np.round(sampling_rate / 4))

    print(f"处理 {csv_path}，原采样率: {sampling_rate} Hz，降采样步长: {stride}")

    # 读取csv，自动适配分隔符
    try:
        df = pd.read_csv(csv_path, engine="python")
        # 自动去除列名空格
        df.columns = [c.strip() for c in df.columns]
    except Exception as e:
        print(f"[Error] 读取csv失败: {csv_path}, 原因: {e}")
        continue

    # 检查所有q字段是否存在
    missing = [col for col in q_cols if col not in df.columns]
    if missing:
        print(f"[Error] 缺少关节列: {missing}，跳过。")
        continue

    # 降采样
    df_down = df.iloc[::stride].reset_index(drop=True)
    q_arr = df_down[q_cols].values.astype(np.float32)
    if len(q_arr) < 2:
        print(f"[Warning] {csv_path} 降采样后数据太少，跳过。")
        continue
    agent_pose = q_arr[:-1]
    action = q_arr[1:]

    zarr_path = os.path.join(folder_path, "sa_pairs_q_to_q_4hz.zarr")
    zarr_root = zarr.open(zarr_path, mode='w')
    
    z_agent = zarr_root.create_array('agent_pose', shape=agent_pose.shape, chunks=(1000, agent_pose.shape[1]), dtype='float32')
    z_agent[:] = agent_pose

    z_action = zarr_root.create_array('action', shape=action.shape, chunks=(1000, action.shape[1]), dtype='float32')
    z_action[:] = action

    print(f"已保存: {zarr_path} | agent_pose: {agent_pose.shape}, action: {action.shape}")

print("全部处理完成！")
