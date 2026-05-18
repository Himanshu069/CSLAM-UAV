<h1>Autonomous Multi-UAV Exploration System (ROS 2 + PX4(uXRCE-DDS)  + RTAB-Map)</h1>

This repository implements a full **multi-agent autonomous exploration framework** for UAVs using **ROS 2, PX4, Gazebo, and RTAB-Map**. The system is designed for both **simulation and real-world deployment**, maintaining identical architecture across both environments.

It supports:
- Multi-UAV visual-inertial SLAM
- Centralized map fusion (2D occupancy grids)
- Autonomous frontier-based exploration
- RRT* global planning + APF local control
- Real-time safety filtering using Control Barrier Functions (CBF)
- Decentralized multi-drone coordination without explicit task allocation

---

## System Overview
<img width="616" height="471" alt="image" src="https://github.com/user-attachments/assets/3b06858a-7ee0-4479-a2a5-8e7ef223f889" />

Each UAV runs an independent SLAM and exploration stack inside its own ROS 2 namespace:

- `/x500_drone_0`
- `/x500_drone_1`

Each agent performs:
- Visual-Inertial Odometry (RGB-D + IMU fusion)
- RTAB-Map based local SLAM
- Independent map generation (Occupancy Grid + OctoMap)

A centralized ground-station node performs:
- 2D occupancy grid merging (real-time, 4 Hz)
- 3D point cloud fusion (simulation only)
- Multi-agent map alignment using:
  - Known initial poses (rigid transform)
  - OR feature-based estimation (AKAZE-based registration)

---

## Core Architecture

### 1. SLAM Pipeline (Per UAV)
- RGB-D + IMU time synchronization
- Visual-Inertial Odometry fusion
- RTAB-Map graph optimization with loop closure
- Independent local map database per agent
- Odometry feedback integrated into PX4 EKF (GPS-denied navigation)

---

### 2. Map Merging System (Central Node)

#### 2D Occupancy Grid Fusion
- Dynamic ROS 2 topic discovery
- Namespace-based multi-robot map subscription
- Dual-mode alignment:
  - Known pose alignment (fixed transforms)

---

### 3. Autonomous Exploration Stack
<img width="313" height="546" alt="Screenshot from 2026-05-06 19-54-59" src="https://github.com/user-attachments/assets/ff9b3390-ec31-40be-a2e9-36e407474d2d" />
<img width="243" height="310" alt="Screenshot from 2026-05-06 20-10-46" src="https://github.com/user-attachments/assets/2ce14ea6-abe6-43fa-b3dc-acbb5f8a798e" />


https://github.com/user-attachments/assets/bf80fb55-6486-4027-b883-12a1031e768e


Each UAV independently runs:

#### Frontier-Based Exploration
- Frontier detection via unknown-cell adjacency
- Cluster filtering using morphological operations
- Scoring function combining:
  - Information gain (cluster size)
  - Distance penalty
  - Approach feasibility score

#### Global Planning (RRT*)
- Collision-aware planning on inflated costmap
- Rewiring-based optimality improvement
- Anytime replanning with stuck detection recovery

#### Local Control (APF)
- Attractive force (goal following)
- Repulsive force (obstacles via EDT)
- Inter-drone repulsive force (decentralized avoidance)
- Smooth velocity generation using tanh scaling

#### Safety Layer (CBF QP Filter)
- Dual control barrier functions:
  - Obstacle avoidance constraint
  - Frontier safety constraint
- Closed-form KKT-based QP solver (no external optimizer)
- Always-feasible soft fallback with slack variable

---

### 4. Multi-UAV Coordination (Decentralized)

No centralized task assignment is used.

Instead:
- Workspace partitioning via perpendicular bisector
- Adaptive boundary rotation to avoid deadlock
- Reactive inter-drone repulsion in APF layer
- Fully decentralized execution using only pose sharing

---

## Simulation Environment

- PX4 SITL + Gazebo (x500 depth quadrotor)
- Micro-XRCE-DDS bridge (PX4 ↔ ROS 2)
- Two UAV instances in shared environment
- Namespace-isolated sensor streams
- Identical architecture to hardware deployment

---

## Hardware Deployment




https://github.com/user-attachments/assets/60eb4398-de8c-4890-864f-8d0ada58b992



### Flight Stack
- PX4 (CUAV Nano 7, Pixhawk 4 flight controllers)
- EKF tuned for vision-based navigation (GPS/mag/barometer disabled)

### Companion Computers
- Raspberry Pi 4 (Drone 0)
- Raspberry Pi 5 (Drone 1)

### Sensors
- Intel RealSense D435i
- Luxonis OAK-D Pro W
- TFMINI PRO
- VL53l1x, Vl53l0x

### Communication
- ROS 2 DDS over shared LAN
- Micro-XRCE-DDS Agent/Client bridge
- Fully synchronized namespace-based topic routing

---

## Key Features

-  Multi-UAV RTAB-Map SLAM
-  Centralized real-time map fusion
- GPS-denied navigation
-  Frontier-based autonomous exploration
-  RRT* + APF hybrid planning
-  Dual-CBF safety filter (closed-form QP solver)
-  Decentralized multi-robot coordination
-  Simulation-to-hardware consistency

---

## Exploration Metrics Logging

Each run logs:

### Coverage Metrics
- Coverage %
- Explored area
- Coverage rate

### Planning Metrics
- RRT* solve time
- Iteration count
- Frontier count

### Safety Metrics
- Minimum obstacle clearance
- CBF activation rate
- QP infeasible events
- APF force ratio

### Multi-UAV Metrics
- Inter-drone distance
- Repulsion forces
- Safety violations

Outputs:
- CSV logs (time-series)
- JSON summary statistics
- Post-processing plots

---

## Launch Instructions

### 1. Build Workspace
```bash
mkdir -p ~/cslam_ws/src
cd ~/cslam_ws/src
git clone https://github.com/Himanshu069/CSLAM-UAV.git .
git submodule update --init --recursive

cd ~/cslam_ws
colcon build 
source install/setup.bash
colcon build
```
### 2. Run RTAB-MAP SLAM Simulation
```bash
ros2 launch drone_slam_pkg px4_gazebo.launch.py
```
This launches the RTABMAP SLAM simulation which takes in the imu data from PX4,and rgb and depth image from the camera and the odometry data is published to PX4 which handles low level control and it also builds a 3d pointcloud map which is projected to 2d needed for autonomous exploration planning .

For hardware, checkout to rasppi branch and run an equivalent launch file for RTAB-MAP SLAM and odometry , though you will need to configure camera specific drivers and setup the companion computer accordingly.
### 3. Run Exploration-Planner
In one terminal run
```bash
ros2 launch drone_slam_pkg offboard_control.launch.py
```
After the drone settles at a height. In another terminal, run

```bash
ros2 launch drone_slam_pkg exploration_planner.launch.py
```
The launch file handles the entire exploration planner stack and logges the required data.




