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

## 🔍 Exploration & Navigation – Practical Observations

This repository includes experiments with multiple exploration and navigation approaches.
Below is a summary of what worked reliably, what showed limitations, and what is still under active development, based on hands-on testing in simulated indoor environments.

### 🟢 Auto Waypoint Navigation
**Status:** Stable and predictable

**What worked well**
- Reliable execution of predefined and auto-generated waypoints using Nav2
- Smooth and predictable navigation while building a map online via SLAM
- Effective handling of local obstacles through Nav2 planners and controllers
- Easier debugging due to deterministic goal sequencing
- Well-suited for structured indoor environments and coverage-style tasks

**Where it showed limitations**
- Waypoint selection logic is decoupled from map uncertainty or information gain
- Exploration efficiency depends heavily on the waypoint generation strategy
- Without frontier-aware heuristics, the robot may revisit known areas unnecessarily

**Key takeaway**

Waypoint-based navigation is stable and predictable, and works well with online SLAM, but exploration quality depends on how intelligently waypoints are generated rather than on navigation itself.

### 🟢 Frontier-Based Exploration (Custom Logic)
**Status:** Stable/Actively evolving

**What worked well**
- Autonomous discovery of unexplored regions without prior map knowledge
- Effective frontier detection from occupancy grid data
- Natural expansion of explored space compared to waypoint methods
- Demonstrates true exploration behavior rather than scripted motion

**Where it showed limitations**
- Frontier selection quality significantly impacts exploration efficiency
- Noisy or closely spaced frontiers can cause oscillations or goal switching
- Requires careful filtering and clustering to avoid redundant navigation
- Navigation failures compound quickly if frontier goals are poorly chosen

**Key takeaway**

Frontier-based exploration enables true autonomy, but shifts complexity from navigation to decision-making and goal selection.

```md
⚠️ Note:
The `full_frontier_based_exploration` module is currently under active development.
This section focuses on building a more robust frontier detection, clustering,
and goal selection pipeline based on lessons learned from earlier approaches.

The current implementation reflects ongoing experimentation rather than a
finalized solution.
```

### ⚠️ Explore-lite / Nav2 Explore Plugins
**Status:** Tested, not used as final solution

**Outcome:** Inconsistent exploration behavior in complex indoor maps

**Observed issues:**
- Limited control over frontier selection logic
- Difficulty handling narrow passages and room transitions
- Less predictable behavior compared to custom frontier logic

**Key takeaway:**

Useful for quick demos, but insufficient for fine-grained exploration control

## 🧠 Overall Insights
- Navigation stability is rarely the limiting factor — goal selection logic is
- Exploration is fundamentally a decision-making problem, not just a planning problem
- Simple approaches (waypoints) outperform complex ones in constrained environments
- Autonomous exploration benefits more from good heuristics than complex planners
