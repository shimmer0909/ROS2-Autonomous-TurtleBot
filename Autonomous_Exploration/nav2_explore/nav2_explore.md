# 🧭 Fully Automatic Exploration with Nav2 Explore Server

Unlike explore_lite, the Explore Server is fully integrated with Nav2’s planner, controller, costmaps, and Behavior Tree — providing stable and complete automatic mapping of indoor environments.

## 📌 nav2_explore_server is not part of Humble, Iron, or any officially maintained Nav2 release.

So we can drop that approach completely.

## 🚀 Step 1: Launch Simulation and Navigation Stack

### 1️⃣ Launch Gazebo (TurtleBot3 House World)

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

### 2️⃣ Launch SLAM Toolbox

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

3️⃣ Launch Nav2 Navigation Stack

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

4️⃣ Launch RViz (optional, for visualization)

```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## 🧩 Step 2: Launch the Nav2 Explore Server (Fully Automatic)

Install the exploration server if not already installed:
```bash
cd ~/ros2_ws/src
git clone https://github.com/ros-planning/navigation2.git -b humble
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

Then launch it:
```bash
ros2 launch nav2_explore_server explore_launch.py use_sim_time:=True
```

### Possible Installation Errors

1. CMake Error at CMakeLists.txt:20 (find_package):
  By not providing "Findnanoflann.cmake"...
  Could not find a package configuration file provided by "nanoflann"

```bash
sudo apt-get update
sudo apt-get install -y libnanoflann-dev

cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

2. Terminal Crash Due to Memory/CPU Exhaustion.

Terminate all the all simulations and stacks before running colcon build. 

Open System Monitor or Task Manager and keep checking the resources.

## 📌 Inspite of a successful build I could not launch Nav2 Explore

```bash
Package 'nav2_explore_server' not found: "package 'nav2_explore_server' not found, searching: ['/opt/ros/humble']"
```

Once running, the Explore Server will:

1. Automatically identify frontiers

2. Select reachable exploration goals

3. Send them to Nav2’s planner

4. Drive the robot until the entire map is explored

5. Stop automatically when no valid frontiers remain

## 🗺️ Step 3: Monitor Mapping Progress (Optional)

In RViz, ensure:

1. Map (OccupancyGrid) is visible

2. LaserScan (/scan) is visible

3. TF tree is connected (map → odom → base_link)

You will see the map expanding as the robot automatically explores the house.

## 💾 Step 4: Save the Completed Map

When exploration finishes (or whenever you're satisfied with the map):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/nav2_explore_map
```

This produces:

1. nav2_explore_map.yaml

2. nav2_explore_map.pgm

## ✅ Expected Result

1. The robot explores the entire environment automatically.

2. Mapping is consistent, frontier-based, and deterministic.

3. Works best for indoor environments like houses, labs, and office spaces.

4. No human supervision or waypoint placement required.

## ✅ Actual Results

1. 📌 nav2_explore_server is not part of Humble, Iron, or any officially maintained Nav2 release.

2. Some older ROS2 blogs and forks still reference an “explore server”

3. It used to exist briefly as an experimental frontier-exploration module in early Nav2 (pre-Foxy)

4. People assume it still exists because tutorials copy/paste from that era