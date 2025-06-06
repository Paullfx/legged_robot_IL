# Guideline

Read this guideline and follow the steps.

---

**Step 1: 机器狗开机**

Run the following commands:

按一瞬间开机键，再长按开机键直到听到电机启动的声音


---
**Step 2: 手机unitree APP**

让自己的手机被授权使用这支 Go2

- a.关闭手机流量，连接狗子的 WiFi，名叫 `"家人"`
- b.点按钮 **Go!**，点击右上角打开的菜单里面的 “卧倒”；更多可选项是关闭：“避障”,“雷达”
- c.点主菜单的右上角三个点 -> “设备”，服务状态：sport_model：点一下关闭
- d.最终状态是：狗子能连接，Go 按钮变灰

---
**Step 3: Open Terminal**

- There are in totoal 7 terminals
  - Terminal 1: ssh
  - Terminal 2: slam
  - Terminal 3: Rviz
  - Terminal 4: 走起来，cd go2, python script
  - Terminal 5: 起立
  - Terminal 6: launch controller
  - Terminal 7: sdk

---
- **Step 3.A**: cd

In **Terminal 2**
```bash
cd rosworkspace
```

In **Terminal 3,4,5,6,7**
```bash
cd catkin_ws
```

---

- **Step 3.B**: 

In **Terminal 2-7**
```bash
source devel/setup.bash
```

In **Terminal 1**
```bash
ssh unitree@162.168.123.18
```
在terminal提示后输入“2”，选择Noetic系统

```bash
cd hesai_lidar_ros1_ws
```
设置主节点，source
```bash
export ROS_MASTER_URI=http://192.168.123.222:11311
source devel/setup.bash
```

---

- **Step 3.C**: Source

In **Terminal 2**
```bash
source devel/setup.bash
```

In **Terminal 3,4,5,6,7**

```bash
source ~/catkin_ws/devel/setup.bash
export ROBOT_TYPE=go2
```

---

- **Step 3.D**: 启动主节点

In **Terminal 7**, sdk
```bash
roslaunch legged_unitree_hw legged_unitree_hw.launch
```

In **Terminal 1**, 启动激光雷达
```bash
roslaunch hesai_ros_driver start.launch
```
此时进入**Terminal 2**,检查rostopic是否出现lidar相关的
```bash
rostopic list
```
在Terminal 2运行SLAM
```bash
roslaunch hesai_hector_slam hesai_slam.launch
```
此时进入**Terminal 3**,检查rostopic是否出现slam相关的
```bash
rostopic list
rostopic echo /slam_out_pose
```

---

- **Step 3.E**: 准备让狗子站起来
准备工作，在**Terminal 3**, 打开RViz
```bash
roslaunch my_marker_pkg display_case0.launch
```

在**Terminal 6**, launch controller
```bash
roslaunch legged_controllers load_controller.launch cheater:=false
```

在**Terminal 5**, 起立
```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0" && rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.2
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
---
在**Terminal 4**, 走起来
```bash
cd catkin_ws/src/SA_Simulation/go2
python3 go2_test.py
```
---


---

**Step last: 机器狗关机**

按一瞬间开机键，再长按开机键直到听到关闭

---