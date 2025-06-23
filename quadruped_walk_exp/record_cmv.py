#!/usr/bin/env python3  # 指定运行脚本所使用的 Python 解释器
# coding=utf-8             # 指定源码文件的编码格式
"""record_robot_data_ros2.py
===========================
**功能概述**
-----------
本脚本是对最初 `record_script.py` 的 ROS 1 实现的重构：

1. **完全去除** Realsense 点云与图像相关代码，避免对 Open3D / pyrealsense2 的依赖。
2. **迁移到 ROS 2 (rclpy)**，并**订阅**与 `all_topics_scripts.py` 同名的 4 个话题：
   * `/lowstate` → `unitree_go/msg/LowState`
   * `/lf/lowstate` → `unitree_go/msg/LowState`
   * `/sportmodestate` → `unitree_go/msg/SportModeState`
   * `/lf/sportmodestate` → `unitree_go/msg/SportModeState`
3. 仍沿用 **`/stop_command`** (`std_msgs/Int16`) 作为保存数据并安全退出的触发器。
4. 以 **50 Hz (0.02 s)** 定时采样，将最新各话题消息与时间戳缓存于内存，最终保存为 **pickle** 文件。

出于演示目的，脚本直接将 ROS 2 消息对象放入 pickle（这些消息是 *picklable* 的）。若后续希望与非 Python 语言共享数据，可再添加自定义序列化逻辑（JSON/CSV/Parquet 等）。
"""

# ------------------ 依赖库导入 ------------------
import os
import pickle
from datetime import datetime, timezone, timedelta

import rclpy                        # ROS 2 Python 客户端库
from rclpy.node import Node         # 节点基类
from std_msgs.msg import Int16      # 停止命令
from unitree_go.msg import (
    LowState,
    SportModeState,
)

# ------------------ 保存目录 ------------------
ROOT_SAVE_DIR = os.path.expanduser('~/data/quadruped_walk_2_pkl')

# ------------------ 工具函数 ------------------

def timestamp_str() -> str:
    """返回当前 ROS 时间（单调时钟）的人类可读字符串，精度毫秒，时区为 UTC+2。"""
    now = rclpy.clock.Clock().now().to_msg()           # `builtin_interfaces/msg/Time`
    seconds = now.sec + now.nanosec * 1e-9
    return (
        datetime.fromtimestamp(seconds, tz=timezone(timedelta(hours=2)))
        .strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]        # 去掉微秒后三位，保留毫秒
    )


def make_save_folder() -> str:
    """创建带时间戳的实验文件夹，如 ros2_record_20250621_154012。"""
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    folder = os.path.join(ROOT_SAVE_DIR, f'ros2_record_{ts}')
    os.makedirs(folder, exist_ok=True)
    return folder

# ------------------ 主节点 ------------------

class RecordRobotData(Node):
    """订阅多个话题并以 50 Hz 缓存数据的节点, 可以改成4Hz。"""

    def __init__(self):
        super().__init__('record_robot_data_ros2')

        # ---- 保存目录与缓存结构 ----
        self.save_dir = make_save_folder()
        self.cache = {
            'timestamp': [],          # str
            'low': [],               # unitree_go/msg/LowState | None
            'lf_low': [],            # unitree_go/msg/LowState | None
            'sport': [],             # unitree_go/msg/SportModeState | None
            'lf_sport': []           # unitree_go/msg/SportModeState | None
        }
        # 最新消息缓冲区；键名同上但不含 timestamp
        self.latest = {k: None for k in self.cache if k != 'timestamp'}

        # ---- 订阅话题 ----
        self.create_subscription(LowState,       '/lowstate',          self.cb_lowstate, 10)
        self.create_subscription(LowState,       '/lf/lowstate',       self.cb_lflow, 10)
        self.create_subscription(SportModeState, '/sportmodestate',    self.cb_sport, 10)
        self.create_subscription(SportModeState, '/lf/sportmodestate', self.cb_lfsport, 10)
        # 停止命令
        self.create_subscription(Int16, '/stop_command', self.stop_callback, 10)

        # ---- 定时器：4 Hz —— 每次调用 record_tick ----
        self.create_timer(0.25, self.record_tick)

        self._stopped = False  # 一旦保存后置 True，忽略后续定时器

    # ---------- 各话题回调 ----------
    def cb_lowstate(self, msg: LowState):
        self.latest['low'] = msg

    def cb_lflow(self, msg: LowState):
        self.latest['lf_low'] = msg

    def cb_sport(self, msg: SportModeState):
        self.latest['sport'] = msg

    def cb_lfsport(self, msg: SportModeState):
        self.latest['lf_sport'] = msg

    # ---------- 定时缓存 ----------
    def record_tick(self):
        """每 0.02 s 触发一次，将最新数据写入 `self.cache`。"""
        if self._stopped:
            return  # 已经保存过，直接忽略

        # 若尚未收到任何话题，则无需记录（避免一堆空行）
        if not any(self.latest.values()):
            return

        # 追加时间戳
        self.cache['timestamp'].append(timestamp_str())
        # 追加各消息（可为 None，保持对齐）
        for key in self.latest:
            self.cache[key].append(self.latest[key])

    # ---------- 数据保存 ----------
    def _save_cache(self):
        path = os.path.join(self.save_dir, 'ros2_record.pkl')
        with open(path, 'wb') as f:
            pickle.dump(self.cache, f)
        self.get_logger().info(f'Data saved to {path}')

    # ---------- 停止回调 ----------
    def stop_callback(self, msg: Int16):
        if msg.data == 0 and not self._stopped:
            self._stopped = True
            self._save_cache()
            self.get_logger().info('Stop command received — shutting down node.')
            # 优雅关闭：先销毁节点，再结束 rclpy.spin
            self.destroy_node()
            rclpy.shutdown()

# ------------------ main ------------------

def main():
    rclpy.init()
    node = RecordRobotData()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl‑C 退出时也尝试保存
        if not node._stopped:
            node._save_cache()
    # 若没有被 stop_callback 销毁，则在此销毁
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
