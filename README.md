# Local PRM Replanner

## Overview
This repository contains a **local PRM-based replanner** for **sudden obstacle avoidance** in mobile robot navigation.

The system integrates:

- **Global path planning** through graph search (A*)
- **Local probabilistic replanning** when a collision is detected on the current path segment
- **EKF-based localization** using range measurements from fixed beacons
- **ROS/RViz visualization** of graphs, paths, detected obstacles, and state estimation

The objective is to demonstrate a **reactive navigation strategy** capable of safely reconnecting the robot to the global path in the presence of unexpected obstacles.

---

## Project Context
This project was developed within the  
**Master’s Degree in Computer Engineering and Robotics**  
at the **University of Perugia**.

A detailed description of the methodology, algorithms, and experimental results is provided in the **Tesina_AutonomousRoboticsProject**.

---

## Main Components

### Global Planner
Builds a navigation graph from a YAML map and computes the optimal path using **A\***.

### EKF Localization
Estimates the robot pose and covariance from control inputs and **distance measurements to fixed beacons**.

### Obstacle Monitor
Detects whether a dynamic obstacle **intersects the currently traversed path segment**.

### Local PRM Replanner
Generates a **random local graph** around the detected obstacle and computes a **collision-free reconnection path** toward the next global node.


## Build and Run

Clone the repository inside a ROS workspace and compile:

```bash
cd ~/catkin_ws/src
git clone https://github.com/ArmandoPalermo/Local_PRM_Replanner.git
cd ..
catkin_make
source devel/setup.bash

roslaunch graph_navigation launch.launch


