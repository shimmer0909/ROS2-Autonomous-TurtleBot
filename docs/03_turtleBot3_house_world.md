# 🏠 TurtleBot3 House World — SLAM & Navigation

This guide explains how to **launch the TurtleBot3 House World in Gazebo**, perform **SLAM mapping**, visualize the robot in **Rviz**, and enable **autonomous navigation** using **Nav2**.

---

## 🎯 Objective

Create and save a 2D occupancy grid map of the TurtleBot3 House World using SLAM Toolbox, then enable autonomous navigation using Nav2.

---

## 🧩 Prerequisites


Before starting, ensure your ROS2 and TurtleBot3 environment is set up properly.
👉 Follow docs/01_setup.md for installation and workspace configuration.

Now, set your TurtleBot3 model permanently:

echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
This ensures the model is correctly loaded every time you open a new terminal.

## 🚀 Step 1: Launch Gazebo Simulation
Launch the TurtleBot3 House World:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
You should see a simulated house environment with your TurtleBot3 spawned in Gazebo.

## ⚙️ Step 2: Install Required Packages
Ensure all essential ROS 2 packages are installed:
```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-turtlebot3* ros-humble-nav2-bringup
```
## 🗺️ Step 3: Launch SLAM (Mapping)
You can use either of the following commands to launch SLAM Toolbox for real-time mapping.
Option 1 — TurtleBot3 SLAM Launch
```bash
ros2 launch turtlebot3_slam slam_toolbox.launch.py use_sim_time:=True
```
Option 2 — Direct SLAM Toolbox Launch
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
Once running, your robot will start generating the occupancy grid map as it moves.

## 👁️ Step 4: Launch Rviz Visualization
Visualize the robot, laser scans, and map in Rviz2:
```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```
In Rviz, enable:
1. Map
2. LaserScan
3. TF
4. RobotModel
5. Odometry
You should now see your TurtleBot3 exploring and mapping the house.

## 🤖 Step 5: Launch Nav2 for Autonomous Navigation
you can launch the Navigation2 (Nav2) stack:
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
This enables autonomous navigation — you can click “2D Goal Pose” in Rviz to send the robot to a target point.

## 🎮 Step 6: Move the Robot (Teleoperation)
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

## 💾 Step 7: Save the Map
After finishing mapping, save the map to your workspace:
```bash
ros2 run nav2_map_server map_saver_cli -f maps/my_house_map
```
This will generate two files:
```bash
maps/my_house_map.pgm
maps/my_house_map.yaml
```

## 🗺️ Step 8: View the Saved Map
View the map image
```bash
eog maps/my_house_map.pgm
```
View the map configuration (YAML)
```bash
gedit maps/my_house_map.yaml
```

