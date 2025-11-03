# 🌍 TurtleBot3 World — Simulation, SLAM, and Navigation2

This document guides you through running the **TurtleBot3 World** simulation in Gazebo, performing **SLAM mapping**, **navigation**, and **map saving** using ROS 2 Humble.

---

## 🎯 Objectives

By the end of this tutorial, you will:
- Launch the TurtleBot3 World in **Gazebo**
- Use **SLAM Toolbox** to map the environment
- Visualize everything in **RViz2**
- Move the robot manually
- Save the generated map

---

## ⚙️ Prerequisites

Before starting, ensure your ROS2 and TurtleBot3 environment is set up properly.  
👉 Follow [docs/01_setup.md](01_setup.md) for installation and workspace configuration.

Now, set your TurtleBot3 model permanently:

```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```
This ensures the model is correctly loaded every time you open a new terminal.

## 🚀 Step 1: Launch Gazebo Simulation
Run the TurtleBot3 World environment in Gazebo:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Expected:
A simulated TurtleBot3 appears in a world.
Gazebo GUI opens showing walls, obstacles, and the robot.

## 🧭 Step 2: Launch Navigation Stack
Launch Navigation2 with simulation time enabled:
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
This starts:
Planner, controller, and recovery nodes
Map server and behavior tree executors
You can verify active nodes:
```bash
ros2 node list
rqt_graph
```

## 🗺️ Step 3: Launch SLAM Toolbox
```bash
ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=True
```
Expected:
1. A live map starts forming in RViz.
2. As the robot moves, unexplored zones appear gray, free space white, and walls black.

## 🧠 Step 4: Open RViz with Default Navigation View
Launch RViz with Nav2’s default configuration:
```bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```
You should see:
1. Robot model
2. Laser scan
3. SLAM-generated map updating in real time

## 🎮 Step 5: Move the Robot (Teleoperation)
Control the robot manually using keyboard teleoperation:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
Use the arrow keys to move the robot and explore the map.
1. w - Move forward
2. x - Move backward
3. a - Turn left
4. d - Turn right
5. s - Stop

## 💾 Step 6: Save the Generated Map
Once mapping is complete, save your map:
```bash
mkdir maps
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```
This creates:
```bash
maps/my_map.yaml
maps/my_map.pgm
```
You can later reuse this map for Navigation2.

