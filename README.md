<h2>Collaborative SLAM with Multi-UAV System (ROS 2 + PX4)</h2>
This repository contains a full workflow for setting up, simulating, and experimenting with multi‑UAV collaborative SLAM (CSLAM) using ROS 2, PX4, Gazebo, and RTAB‑Map. It is designed to support both autonomous and manual teleoperation modes, with each drone running onboard SLAM.

<img width="496" height="512" alt="image" src="https://github.com/user-attachments/assets/4febfa29-4065-4ca6-926e-83b466667bc9" />

---
1️⃣ Prerequisites
System Requirements

Ubuntu 24.04 LTS

ROS 2 Jazzy

PX4 v1.16

Gazebo Harmonic 

Install ROS2 Jazzy and standard dependencies.

---
2️⃣ PX4 Setup (Firmware + Gazebo Models)

```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout release/1.16
```
---
3️⃣ Workspace Setup

```bash
mkdir -p ~/cslam_ws/src
cd ~/cslam_ws/src
git clone https://github.com/Himanshu069/CSLAM-UAV.git .
git submodule update --init --recursive
cd ~/cslam_ws
colcon build --packages_select px4_msgs px4_ros2_cpp
source install/setup.bash
colcon build
```
---
4️⃣ SLAM Simulation
```bash
ros2 launch drone_slam_pkg px4_gazebo.launch.py
```
---
5️⃣ Manual Teleoperation Mode <br>
In one terminal , run: <br>
```bash
ros2 run manual_ros2_px4_multi teleop
```
And in another terminal, run: <br>
```bash
ros2 run manual_ros2_px4_multi teleop_keyboard
```
---
6️⃣ Project Goals: <br>
Individual RTAB-Map SLAM implementation on TWO DRONES<br>
Individual Exploration Planner in both drones using RRT* and Potential Field Functions
Online Map fusion from two drones(ON a Centralized Computer in hardware)<br>


