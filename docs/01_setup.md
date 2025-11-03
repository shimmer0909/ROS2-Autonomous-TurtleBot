# ⚙️ 01 — Environment Setup and Installation Guide

This document covers the complete setup for the **ROS2 Autonomous TurtleBot Project**, including:
- Installing and validating Gazebo, SLAM Toolbox, and Nav2
- Running the TurtleBot3 simulation and Rviz2 visualization
- Preparing your environment variables and workspace

---

## 🧩 Prerequisites

Before starting, ensure that you have:

- **Ubuntu 22.04 LTS**
- **ROS 2 Humble Hawksbill** installed  
  👉 [Official ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- A working internet connection

---

## 🧠 Step 1: Setup TurtleBot3 Packages

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* -y
```
✅ Validation:- 
```bash
ros2 pkg list | grep turtlebot3
```
Expected output includes:
```bash
turtlebot3_bringup
turtlebot3_cartographer
turtlebot3_gazebo
turtlebot3_navigation2
turtlebot3_description
```

## 🧱 Step 2: Set the TurtleBot3 Model

The TurtleBot3 model must be defined for simulation and navigation packages to work.
```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```
✅ Validation:- 
```bash
echo $TURTLEBOT3_MODEL
```
Expected output :
```nginx
waffle
```

## 🌍 Step 3: Install Gazebo for Simulation

Reference: [Setup TurtleBot3 Simulation in ROS2 Humble](https://medium.com/@nilutpolkashyap/setting-up-turtlebot3-simulation-in-ros-2-humble-hawksbill-70a6fcdaf5de)
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros -y
```
✅ Validation:- 
```bash
gazebo --version
```
Expected output (example):
```nginx
Gazebo multi-robot simulator, version 11.x
```

## 🚀 Step 4: Launch Gazebo Simulation
To verify that TurtleBot3 simulation runs correctly:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
This will open the Gazebo simulator with the TurtleBot3 World.
You should see your robot placed in the simulated environment.
Note - It can take a few minutes for Gazebo to load the environment first time.

## 🧭 Step 5: Install SLAM Toolbox

For simultaneous localization and mapping (SLAM):
```bash
sudo apt install ros-humble-slam-toolbox -y
```
✅ Validation:- 
```bash
ros2 pkg list | grep slam_toolbox
```
Expected output:
```nginx
slam_toolbox
```

## 🗺️ Step 6: Launch SLAM Toolbox (Mapping)
This starts SLAM mapping using simulated time.
You can visualize real-time mapping progress in Rviz2 (see next step).
```bash
ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=True
```

## 🛰️ Step 7: Run Rviz2 Visualization
This launches Rviz2, a visualization tool that shows:
1. Laser scans
2. Robot trajectory
3. Real-time map being built
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

## 🧭 Step 8: Install Navigation2 (Nav2) Stack

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
```

✅ Validation:- 
```bash
ros2 pkg list | grep nav2
```
Expected output:
```nginx
nav2_bringup
nav2_map_server
nav2_planner
nav2_controller
```

## 🛰️ Step 9: Run Navigation2 (Nav2)
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

## 🧹If you encounter build or dependency errors later:

```bash
sudo apt update && sudo apt upgrade -y
rosdep update
```

### 💡 Summary of What This Doc Adds

- Step-by-step **installation**, **validation**, and **test launch** commands.  
- Clean structure compatible with your `docs/` folder naming convention.  
- Direct verification steps to ensure each component works before moving forward.



