import os
import zarr
import pickle
import tqdm
import numpy as np
import torch
import pytorch3d.ops as torch3d_ops
import torchvision
from termcolor import cprint
import re
import time
import socket
import pickle
import open3d as o3d
import cv2
def visualize_point_cloud(points, title="Point Cloud"):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([pcd], window_name=title)

def farthest_point_sampling(points, num_points=2048, use_cuda=True):
    K = [num_points]
    if use_cuda:
        points = torch.from_numpy(points).cuda()
        sampled_points, indices = torch3d_ops.sample_farthest_points(points=points.unsqueeze(0), K=K)
        sampled_points = sampled_points.squeeze(0)
        sampled_points = sampled_points.cpu().numpy()
    else:
        points = torch.from_numpy(points)
        sampled_points, indices = torch3d_ops.sample_farthest_points(points=points.unsqueeze(0), K=K)
        sampled_points = sampled_points.squeeze(0)
        sampled_points = sampled_points.numpy()

    return sampled_points, indices

def preprocess_point_cloud(points, use_cuda=True):
    num_points = 2048
    # d455
    extrinsics_matrix = np.array([[0.04931362,  0.38340706, -0.922262,  1.096588],
                                    [ 0.99757704,  0.02646174, 0.06434153,  -0.10391138],
                                    [0.04907365, -0.9232003, -0.38117316,   0.38142048],
                                    [ 0.,          0. ,         0.,          1.        ]])

    point_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
    point_homogeneous = np.dot(extrinsics_matrix, point_homogeneous.T).T
    points = point_homogeneous[..., :-1]
    points, sample_indices = farthest_point_sampling(points, num_points, use_cuda)
    return points

def add_gaussian_noise_to_pointcloud(pointcloud, robot_state, std_dev=0.01):
    extrinsics_matrix = np.array([[0.04931362,  0.38340706, -0.922262,  1.096588],
                                    [ 0.99757704,  0.02646174, 0.06434153,  -0.10391138],
                                    [0.04907365, -0.9232003, -0.38117316,   0.38142048],
                                    [ 0.,          0. ,         0.,          1.        ]])
    point_homogeneous = np.hstack((pointcloud, np.ones((pointcloud.shape[0], 1))))
    point_homogeneous = np.dot(extrinsics_matrix, point_homogeneous.T).T
    pointcloud = point_homogeneous[..., :-1]
    # Define the cylindrical region
    z_min = robot_state[2] - 0.17
    z_max = robot_state[2]
    center_x = robot_state[0]
    center_y = robot_state[1]
    radius = 0.11  # Define the radius of the cylinder
    # Identify the points within the cylindrical region
    mask = (
        (pointcloud[:, 2] >= z_min) &
        (pointcloud[:, 2] <= z_max) &
        (np.sqrt((pointcloud[:, 0] - center_x) ** 2 + (pointcloud[:, 1] - center_y) ** 2) <= radius)
    )
    noise = np.random.normal(0, std_dev, pointcloud[mask].shape)
    pointcloud[mask] += noise
    return pointcloud


def down_sample_point_cloud(points, num_points=2048, use_cuda = True):
    points, sample_indices = farthest_point_sampling(points, num_points, use_cuda)
    return points

def preproces_image(image):
    img_size = 84
    image = image.astype(np.float32)
    image = torch.from_numpy(image).cuda()
    image = image.permute(2, 0, 1)
    image = torchvision.transforms.functional.resize(image, (img_size, img_size))
    image = image.permute(1, 2, 0)
    image = image.cpu().numpy()
    return image

def visualize_point_cloud(points, title="Point Cloud"):
    # Create an Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd], window_name=title)

expert_data_path = '/home/yxt/thesis/yirui/imitation/3D-Diffusion-Policy/3D-Diffusion-Policy/data/6-2'
save_data_path = '/home/yxt/thesis/yirui/imitation/3D-Diffusion-Policy/3D-Diffusion-Policy/data/6-2/pick-place-l515-tea.zarr'
demo_dirs = [os.path.join(expert_data_path, f) for f in os.listdir(expert_data_path) if f.endswith('.pkl')]

print(f"demo_dirs: {len(demo_dirs)}")

total_count = 0
point_cloud_arrays = []
state_arrays = []
action_arrays = []
episode_ends_arrays = []

mask_pc = False

if os.path.exists(save_data_path):
    cprint('Data already exists at {}'.format(save_data_path), 'red')
    cprint("If you want to overwrite, delete the existing directory first.", "red")
    cprint("Do you want to overwrite? (y/n)", "red")
    user_input = 'y'
    if user_input == 'y':
        cprint('Overwriting {}'.format(save_data_path), 'red')
        os.system('rm -rf {}'.format(save_data_path))
    else:
        cprint('Exiting', 'red')
        exit()
os.makedirs(save_data_path, exist_ok=True)

for demo_dir in demo_dirs:
    dir_name = os.path.dirname(demo_dir)

    cprint('Processing {}'.format(demo_dir), 'green')
    with open(demo_dir, 'rb') as f:
        demo = pickle.load(f)

    pcd_dirs = os.path.join(dir_name, 'pcd')
    if not os.path.exists(pcd_dirs):
           os.makedirs(pcd_dirs)
        
    demo_length = len(demo['point_cloud'])
    for step_idx in range(demo_length):
       
        total_count += 1
        
        robot_state = demo['agent_pose'][step_idx]
        obs_pointcloud = demo['point_cloud'][step_idx]

        action = demo['action'][step_idx]
        if action[6] == 1:
            action = action[:6]
            action = np.append(action, [1, 0])
        elif action[6] == 0:
            action = action[:6]
            action = np.append(action, [0, 1])
        if mask_pc:
            obs_pointcloud = add_gaussian_noise_to_pointcloud(obs_pointcloud, robot_state)
            obs_pointcloud = down_sample_point_cloud(obs_pointcloud, num_points=2048, use_cuda=True)
        else:
            obs_pointcloud = preprocess_point_cloud(obs_pointcloud, use_cuda=True)
        action_arrays.append(action)
        point_cloud_arrays.append(obs_pointcloud)
        state_arrays.append(robot_state)
    cv2.destroyAllWindows()
    episode_ends_arrays.append(total_count)

# create zarr file
zarr_root = zarr.group(save_data_path)
zarr_data = zarr_root.create_group('data')
zarr_meta = zarr_root.create_group('meta')

point_cloud_arrays = np.stack(point_cloud_arrays, axis=0)
action_arrays = np.stack(action_arrays, axis=0)
state_arrays = np.stack(state_arrays, axis=0)
episode_ends_arrays = np.array(episode_ends_arrays)

compressor = zarr.Blosc(cname='zstd', clevel=3, shuffle=1)
point_cloud_chunk_size = (100, point_cloud_arrays.shape[1], point_cloud_arrays.shape[2])
if len(action_arrays.shape) == 2:
    action_chunk_size = (100, action_arrays.shape[1])
elif len(action_arrays.shape) == 3:
    action_chunk_size = (100, action_arrays.shape[1], action_arrays.shape[2])
else:
    raise NotImplementedError
zarr_data.create_dataset('point_cloud', data=point_cloud_arrays, chunks=point_cloud_chunk_size, dtype='float64', overwrite=True, compressor=compressor)
zarr_data.create_dataset('action', data=action_arrays, chunks=action_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
zarr_data.create_dataset('state', data=state_arrays, chunks=(100, state_arrays.shape[1]), dtype='float32', overwrite=True, compressor=compressor)
zarr_meta.create_dataset('episode_ends', data=episode_ends_arrays, chunks=(100,), dtype='int64', overwrite=True, compressor=compressor)

# print shape
cprint(f'point_cloud shape: {point_cloud_arrays.shape}, range: [{np.min(point_cloud_arrays)}, {np.max(point_cloud_arrays)}]', 'green')
cprint(f'action shape: {action_arrays.shape}, range: [{np.min(action_arrays)}, {np.max(action_arrays)}]', 'green')
cprint(f'state shape: {state_arrays.shape}, range: [{np.min(state_arrays)}, {np.max(state_arrays)}]', 'green')
cprint(f'episode_ends shape: {episode_ends_arrays.shape}, range: [{np.min(episode_ends_arrays)}, {np.max(episode_ends_arrays)}]', 'green')
cprint(f'total_count: {total_count}', 'green')
cprint(f'Saved zarr file to {save_data_path}', 'green')