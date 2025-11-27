# 🤖 Project 1: ROS2 Autonomous TurtleBot

A complete exploration of **robot autonomy using ROS2 and TurtleBot3** — performing **mapping (SLAM)** and **navigation (Nav2)** inside Gazebo simulation and visualized with Rviz.  
This is my **first full integration project** after learning core ROS2 concepts from my [Robotics Basics](https://github.com/shimmer0909/Robotics-basics) repository.

---

## 🧭 Project Overview

This project demonstrates how a **TurtleBot3 robot** can:
1. Map an unknown environment using **SLAM**.
2. Navigate autonomously using the **Nav2 stack**.
3. Simulate in **Gazebo** and visualize data in **Rviz2**.

Two simulated worlds were used:

| Map | Description |
|------|--------------|
| 🗺️ **TurtleBot3 World** | A clean, open environment for initial testing |
| 🏠 **TurtleBot3 House World** | A realistic indoor environment for navigation challenges |

---

## 🧩 Learning Objectives

- Setup and run TurtleBot3 simulation in Gazebo  
- Generate maps using SLAM (`slam_toolbox` / `cartographer`)  
- Navigate using the ROS2 Nav2 stack  
- Visualize robot pose, laser scans, and map building in Rviz  
- Understand the overall flow from perception → localization → planning → control  

---

## 🗂️ Repository Structure

```text
ROS2-Autonomous-TurtleBot/
│
├── Autonomous_Exploration/                      # Ways to automate map generation
│   ├── auto_waypoint_generator            # Automate waypoint generation
│   ├── explore_lite         # Inbuild package (Doesn't work good for indoor maps)
│   ├── nav2_bt_navigator        # Use Rviz to set waypoints for exploration and mapping (Not fully autonated)
│   ├── nav2_explore        # Experimental package (Didn't work for me)
│
├── Setup/                    # Pre-requisites and setup for Turtlebot3, Gazebo, Nav2, Slam, Rviz
│   ├── setup.md
│
├── Manual_Exploration/               # Manual map creation steps and samples
│   ├── TurtleBot3_House_World
│   ├── TurtleBot3_World
│
└── README.md
```

## ⚙️ Software Stack

| Component | Version / Tool |
|------------|----------------|
| **ROS2 Distribution** | Humble Hawksbill (recommended) |
| **Robot** | TurtleBot3 Burger / Waffle Pi |
| **Simulation** | Gazebo Classic |
| **Mapping** | SLAM Toolbox |
| **Navigation** | Nav2 Stack |
| **Visualization** | Rviz2 |

---
