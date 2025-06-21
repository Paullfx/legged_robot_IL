#!/usr/bin/env python3
#coding=utf-8
import rospy
from std_msgs.msg import Float32MultiArray, Int16
from action_infer.msg import PoseAction
import pyrealsense2 as rs
import pickle
import numpy as np
import open3d as o3d
import copy
import os
def depth_to_pointcloud_wo_color(depth_frame, workspace, pc):
    points = pc.calculate(depth_frame)
    points = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
    points = points[np.where((points[..., 0] > workspace[0][0]) & (points[..., 0] < workspace[0][1]) &
                    (points[..., 1] > workspace[1][0]) & (points[..., 1] < workspace[1][1]) &
                    (points[..., 2] > workspace[2][0]) & (points[..., 2] < workspace[2][1]))]
    return points
    
def visualize_pointcloud(pointcloud_3d):
    """
    Visualize a 3D point cloud (XYZ) using Open3D.
    :param pointcloud_3d: A numpy array of shape (N, 3), where each row is [X, Y, Z].
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud_3d[:, :3])  # XYZ coordinates
    o3d.visualization.draw_geometries([pcd])

class RecordRobotData:
    def __init__(self):
        rospy.init_node('record_robot_data')

        ctx = rs.context()
        if len(ctx.devices) > 0:
            for dev in ctx.devices:
                print('Found device:', dev.get_info(rs.camera_info.name), dev.get_info(rs.camera_info.serial_number))
        else:
            print("No Intel Device connected")
        devices = ctx.query_devices()
        for dev in devices:
            dev.hardware_reset()
            print('Reset device:', dev.get_info(rs.camera_info.name), dev.get_info(rs.camera_info.serial_number))

        rospy.Subscriber('/franka_pose_action', PoseAction, self.obs_callback, queue_size=1)
        rospy.Subscriber('/stop_command', Int16, self.stop_callback, queue_size=1) 

        self.pose = None
        self.action = None
        self.data_cache = {'agent_pose': [], 'action': [], 'point_cloud': [], 'rs_img': []}

        self.workspace = [(-0.30, 0.30), (-0.15, 0.2), (0.5, 0.85)]
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.flag = False
        
        # first frame is not used
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        print("First frame captured")
    
    def depth_callback(self, msg):
        self.Image_depth_buff = msg

    def rgb_callback(self, msg):
        self.Image_rgb_buff = msg

    def CInfo_callback(self, msg):
        self.CamInfo_buff = msg
    
    def obs_callback(self, msg):
        if self.flag:
            return
        self.pose = np.array(msg.pose_data.data, dtype=np.float32)
        self.action = np.array(msg.action_data.data, dtype=np.float32)
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        rs_img = np.asanyarray(color_frame.get_data())
        pc = rs.pointcloud()  
        points = depth_to_pointcloud_wo_color(depth_frame, self.workspace,pc)
        print("point cloud shape:", points.shape)

        self.data_cache['point_cloud'].append(copy.deepcopy(points))
        self.data_cache['action'].append(copy.deepcopy(self.action))
        self.data_cache['agent_pose'].append(copy.deepcopy(self.pose)) 
        self.data_cache['rs_img'].append(copy.deepcopy(rs_img))
    
    def save_data(self, directory = '/media/yuan/Seagate 2TB/diffusion-data/6-4/'):
        cnt = len(os.listdir(directory))
        path = os.path.join(directory, 'traj'+str(cnt)+'.pkl')
        print(f"Saving data to: {path}")        
        with open(path, 'wb') as f:
            pickle.dump(self.data_cache, f)
    
    def stop_callback(self, msg):
        if msg.data == 0:
            rospy.loginfo("stop command received, saving data")
            self.pipeline.stop()
            
            self.save_data()
            self.flag = True
            rospy.signal_shutdown("Stop command received, shutting down node")
    
    def run(self):
        rospy.spin()
    
def main():
    recorder = RecordRobotData()  
    recorder.run()   

if __name__ == '__main__':
    main()  