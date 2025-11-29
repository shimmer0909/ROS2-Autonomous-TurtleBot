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

## 🧱 Complete working tutorial and code
Refer - [Auto-Waypoint-Navigation Repository](https://github.com/shimmer0909/Auto-Waypoint-Navigation) for detailed steps and code.

## ✅ Result
A map of turtleBot3 House world was created successfully.
[Final Demo Video](https://drive.google.com/file/d/1Wd6d5arcSVy20p_EnRnbW0kVb2GHAEV1/view?usp=drive_link)
