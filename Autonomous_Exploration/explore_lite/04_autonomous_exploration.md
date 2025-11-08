# 🧭 Autonomous Exploration using Explore Lite (m-explore-ros2)

This section explains how to enable **autonomous exploration** for the TurtleBot3 in Gazebo using the [`m-explore-ros2`](https://github.com/robo-friends/m-explore-ros2) package.  
With this, the robot can **automatically explore unknown environments** and build maps without manual teleoperation.

---

## 🚀 Objective

- Add the `explore_lite` package to your ROS2 workspace  
- Build and verify the package  
- Launch autonomous exploration in TurtleBot3 simulation worlds

---

## ⚙️ Prerequisites

Before continuing, ensure you have:

- A working TurtleBot3 simulation setup  
- ROS2 environment sourced (e.g. Humble, Iron, or Jazzy)
- `slam_toolbox` or `cartographer` installed for mapping  
- Workspace located at `~/turtlebot3_ws`

---

## 🧩 Step-by-Step Setup

### 1️⃣ Create (or go to) your workspace

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
```
### 2️⃣ Clone the Explore Lite package
```bash
git clone https://github.com/robo-friends/m-explore-ros2.git explore_lite
```
This package provides frontier-based exploration — it detects unexplored regions of the map and commands the robot to move there automatically.

### 3️⃣ Build the workspace
```bash
cd ~/turtlebot3_ws
colcon build --symlink-install
```
 ### ⚠️ If Build Fails
 Sometimes the build may fail due to leftover build artifacts or missing dependencies.
Clean and rebuild with these steps:
```bash
# Remove previous build logs and artifacts
rm -rf build/ install/ log/

# (Optional) Remove old or conflicting custom interfaces if any exist
rm -rf ~/turtlebot3_ws/src/dynamixel_sdk_custom_interfaces

# Rebuild specific packages if full build fails
colcon build --symlink-install --packages-select explore_lite multirobot_map_merge
```

### 4️⃣ Source the workspace
```bash
source install/setup.bash
```
To make this permanent, add it to your .bashrc:
```bash
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
```

### 5️⃣ Verify Installation
Confirm that explore_lite was successfully built and recognized by ROS2:
```bash
ros2 pkg list | grep explore
```
Expected output:
```nginx
explore_lite
```

## 🧠 What’s Next?
Once installed, you can use the explore_lite package to make your TurtleBot3 autonomously explore the simulated environment.

Example launch sequence:
```bash
# Launch Gazebo with TurtleBot3 House or World
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

# Run SLAM mapping (e.g., Cartographer)
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
#or
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Run Rviz for map observation
ros2 launch turtlebot3_bringup rviz2.launch.py


# Run Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True


# Start Explore Lite
ros2 launch explore_lite explore.launch.py
```

## 🧩 Troubleshooting Tips
1. If the robot doesn’t move, ensure the map topic /map is being published (from SLAM).
2. Check that /move_base or nav2_bt_navigator nodes are active.
3. Adjust parameters in explore.launch.py (like minimum frontier size or cost scaling) if the robot gets stuck.
4. Visualization: use Rviz2 to watch the robot’s exploration progress and the map growth.

## 🗺️ Expected Behavior
1. The robot starts exploring autonomously, identifying “frontiers” (unmapped areas).
2. SLAM continuously updates the map as the robot moves.
3. The exploration continues until all reachable areas are mapped.

## Final Observations
Explore_lite chooses random “frontier” points at the boundary between known and unknown space.
But in TurtleBot3 house world, there’s a wall around the house and lots of obstacles outside it.

So what happens:

1. explore_lite picks a frontier outside the walls (because it “sees” the gray unexplored area outside).
2. Nav2 tries to plan to it.
3. The global planner finds no valid path or repeatedly replans.
4. The controller keeps rotating the robot trying to face a new path.
5. The result = endless “rotation loops” near walls or outside the house.

You can try playing aroung with configurations in ~/turtlebot3_ws/src/explore_lite/config/params.yaml

```yaml
explore_lite:
  ros__parameters:
    planner_frequency: 1.0
    progress_timeout: 30.0
    potential_scale: 1.0
    orientation_scale: 0.0
    gain_scale: 1.0
    transform_tolerance: 0.3
    min_frontier_size: 0.5
    visualize: true
    # Restrict exploration region (house size ~8x8 m)
    bounding_box_min_x: -4.0
    bounding_box_min_y: -4.0
    bounding_box_max_x: 4.0
    bounding_box_max_y: 4.0
```
### Video
[Final Demo](https://drive.google.com/file/d/1o_qBauySpDh73BDDz3AANedQ5nK5jrYH/view?usp=drive_link)



