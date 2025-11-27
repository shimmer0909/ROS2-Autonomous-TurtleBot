## 🧭 Goal

Create a ROS2 node that:
1. Reads the SLAM map (/map)
2. Finds unexplored regions
3. Generates large, stable waypoints
4. Sends these waypoints sequentially to Nav2 (NavigateToPose)
5. Keeps doing this until no unexplored space remains

## 📌 Architecture Overview

```bash
SLAM → /map → Waypoint Generator → /next_waypoint → Nav2 → Robot moves → 
Map updates → Repeat
```

## 🧱 You Will Build 3 Components

### 1️⃣ Map Listener (get occupancy grid)

Subscribes to /map and keeps the latest copy.

### 2️⃣ Waypoint Generator (grid clustering)

1. scan map
2. find unexplored big regions (-1)
3. cluster them
4. compute centroids
5. publish waypoint list

### 3️⃣ Waypoint Executor

1. listen to /next_waypoint
2. call Nav2 action /navigate_to_pose
3. wait until goal completed
4. ask generator for next waypoint

## 🚀 STEP - Refer to my [Auto-Waypoint-Navigation Repository](https://github.com/shimmer0909/Auto-Waypoint-Navigation) for detailed steps and code.

## ✅ Result



## 🧠 Tips



## Final Observations

