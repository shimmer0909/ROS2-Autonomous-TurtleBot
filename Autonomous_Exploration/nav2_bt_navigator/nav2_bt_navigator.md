# 🧭 Autonomous Navigation with Nav2 BT Navigator + Waypoint Follower

> ✅ **Recommended Approach:** Use `nav2_bt_navigator` with the **Waypoint Follower** plugin for structured, deterministic exploration.

Unlike `explore_lite`, which performs random frontier exploration, this approach allows you to **manually define waypoints** for the robot to visit automatically — ensuring a stable and complete map of indoor environments.

---

## 🚀 Step 1: Launch the Simulation and Navigation Stack

### 1️⃣ Launch Gazebo (TurtleBot3 House World)
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
### 2️⃣ Launch SLAM Toolbox
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
### 3️⃣ Launch Nav2 Navigation Stack
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

### 4️⃣ Launch RViz
```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## 🧩 Step 2: Use the Nav2 Waypoint Follower in RViz

1. Open RViz — you should see the Nav2 panels (Goal, Navigation2, etc.).

2. If you don’t see the “Waypoint Follower” panel, add it manually:
Go to Panels → Add New Panel → Nav2 Waypoint Follower

3. Click “Waypoint/ Nav Through Poses Mode” and place 5–10 waypoints across different rooms or corridors using 'Nav2 Goal' button in the top panel.

4. After adding, click “Start Following Waypoints.”

The robot will autonomously move through each waypoint in sequence, updating the map as it goes.

## 🗺️ Step 3: Observe and Save the Map

Once the robot has visited all waypoints and the map looks complete:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/nav2_bt_navigator_map
```
This saves:
1. nav2_bt_navigator_map.yaml
2. nav2_bt_navigator_map.pgm

## ✅ Result

1. The robot autonomously navigates to all your waypoints.
2. Mapping is deterministic — no random rotations or missed corners.
3. Ideal for indoor structured spaces (houses, labs, corridors).

## 🧠 Tips

1. You can adjust waypoint order or edit them in RViz at any time.
2. To repeat the same exploration automatically, save and reload your waypoints as a .yaml file (advanced usage).
3. Works seamlessly with TurtleBot3, SLAM Toolbox, and the default Nav2 Behavior Tree.

## Final Observations

1. Adding waypoints in RViz to guide the robot helps generate a high-quality map easily, but it still requires human intervention to place the waypoints manually.

2. The Global Planner / Planner Server computes the optimal path for the robot to reach each waypoint.

3. The Controller Server provides reactiveness to the plan by handling unforeseen circumstances (e.g., obstacles or sudden changes), enabling reliable autonomous navigation and mapping.

### Video
[Final Demo](https://drive.google.com/file/d/1XnZgQ08SguBevJObjuK7yJiCiJ3NJSEs/view?usp=drive_link)