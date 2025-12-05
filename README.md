<h2> Collaborative SLAM with Multi-UAV System (ROS 2 + PX4) </h2>
---
This repository contains a full workflow for setting up, simulating, and experimenting with multi‑UAV collaborative SLAM (CSLAM) using ROS 2, PX4, Gazebo, and RTAB‑Map. It is designed to support both autonomous and manual‑teleoperation modes, with each drone running onboard SLAM.
Current Progess: Working RTAB-MAP simulation on a single drone & single drone manual control with Teleoperation mode
<img width="1920" height="1054" alt="Screenshot from 2025-09-17 15-49-12" src="https://github.com/user-attachments/assets/14b705d6-ab86-469a-b09b-158b18de4767" />
---
1️⃣ Prerequisites
System Requirements

```
Ubuntu 24.04 LTS

ROS 2  Jazzy & Standard Dependecies

PX4 v1.16

Gazebo Harmonic 
```
---
2️⃣ PX4 Setup (Firmware + Gazebo Models)

Clone the firmware and switch to the correct release:
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout release/1.16
```
---
3️⃣ Workspace Setup
Source your main ROS2 installation , then:
```
cd ~
mkdir -p cslam_ws/src
git clone git@github.com:Himanshu069/CSLAM-UAV.git
git submodule update --init --recursive
cd ~/cslam_ws
colcon build --packages-select px4_ros2_cpp
echo 'export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:$HOME/cslam_ws/src/px4-ros2-interface-lib/px4_ros2_cpp"' >> ~/.bashrc
source install/setup.bash
colcon build
```
---
4️⃣ Multi‑Drone Simulation
To run the main simulation :
```bash
ros2 launch drone_slam_pkg px4_gazebo.launch.py
```
---
5️⃣ Manual Teleoperation Node
In one terminal :
```bash
ros2 run manual_ros2_px4_multi teleop
```
In another Terminal:
```bash
ros2 run manual_ros2_px4_multi teleop_keyboard
```
You can use the commands displayed on this terminal to guide the Drone after the drone takes off and activating the Teleoperation Node in QGC
Note: For another drone , just repeat these steps by adding an instance/namespace for teleop and teleop_keyboard nodes of /px4_1, still under 
development.

