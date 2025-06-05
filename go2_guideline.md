# Guideline

Read this guideline and follow the steps.

---

**Step 1: 机器狗开机**

Run the following commands:

按一瞬间开机键，再长按开机键直到听到电机启动的声音


---
**Step 2: 手机unitree APP**

让自己的手机被授权使用这支Go2
step a: 关闭手机流量，连接狗子的wifi, 名叫"家人“
step b: 点按钮"Go!", 点击右上角打开的菜单里面的“卧倒”, gengduo可选项是关闭“避障“”雷达”
step c: 点主菜单的，在右上的三个点，“设备”， 服务状态“ - sport_model - 点一下关闭
step d: 最终状态是，狗子能连接,Go按钮灰掉
---

**Step 3: Open Terminal**
There are in totoal 7 terminals

step a: 

```bash
export ROS_MASTER_URI=http://192.168.123.222:11311
source devel/setup.bash
roslaunch hesai_ros_driver start.launch
```

---

**Step 2: Open Terminal 2**

```bash
cd hesai_ws
ro
```

**Step 2: Open Terminal 2**

```bash
cd catkin_ws
ro
```

**Step 2: Open Terminal 2**
```bash
cd rosworkspace
ro
```

**Step 2: Open Terminal 2**

(Your next steps here...)

**Step 2: Open Terminal 2**

(Your next steps here...)

**Step 2: Open Terminal 2**

(Your next steps here...)

**Step 2: Open Terminal 2**

(Your next steps here...)

source ~/catkin_ws/devel/setup.bash
export ROBOT_TYPE=go2

运行：
(in work space catkin_ws)
Terminal1（run simulation）： roslaunch legged_unitree_description empty_world.launch
Terminal1 (on board): roslaunch legged_unitree_hw legged_unitree_hw.launch

Terminal2 （load controller）：roslaunch legged_controllers load_controller.launch cheater:=false

Terminal3 （start）： rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
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

Terminal3 （publish velocity）：  rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once     
Terminal3 （publish velocity）：  rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once  

Terminal4 (draw map and trajectory): roslaunch my_marker_pkg display_case0.launch

Terminal5 (give Control Input): python3 go2_test.py

//////////////////////////////////////////////////////////////////////////////////////



Get lidar data:
(in go2 computer): export ROS_MASTER_URI=http://192.168.123.222:11311
source devel/setup.bash
roslaunch hesai_ros_driver start.launch

(in own conputer): export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.123.222


(in work space rosworkspace)
roslaunch hesai_hector_slam hesai_slam.launch
//////////////////////////////////////////////////////////////////////////////////////

---

**Step last: 机器狗关机**

按一瞬间开机键，再长按开机键直到听到关闭

---