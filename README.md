# Development and Integration of a CSLAM Framework for Multiple UAV Platforms

Current Progress(Image):
<img width="1600" height="878" alt="image" src="https://github.com/user-attachments/assets/deb67b75-063e-4999-afe2-a9c78b208136" />



## Objectives
- Implementation of RTABMap in Simulation and Hardware for one drone
- Implementation of FAST-LIO2 in Simulation and Hardware for another drone
- Map Fusion of maps from two drones
- Navigation Policies from map fusion (if feasible)

#  Clone PX4-Autopilot (official repo) and checkout branch v1.16
```
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.16

```

# Modify the OakD-Lite Model (Gazebo)
```
sudo apt install xmlstarlet
cd Tools/simulation/gz/models/OakD-Lite
xmlstarlet ed -L \
  -u "/sdf/model/link/sensor[@name='IMX214']/camera/image/width" -v "640" \
  -u "/sdf/model/link/sensor[@name='IMX214']/camera/image/height" -v "480" \
  model.sdf
```

# Install ROS2 and Gazebo harmonic as per their official docs and install RTAB-MAP

```sudo apt install ros-$ROS_DISTRO-rtabmap-ros```

# Create ROS 2 Workspace
```
mkdir -p ~/cslam_ws/src
cd ~/cslam_ws/src
git clone https://github.com/Himanshu069/CSLAM-UAV.git
```
# Build Required Packages
Note: You need to build the px4_ros2_cpp and px4_msgs and source them before building the teleop packages,
      The teleop package here is only applicable for single drone, it is under work to be changed to make it work for multi drones.
```
cd ~/cslam_ws
colcon build --packages-select px4_msgs px4_ros2_cpp
source install/setup.bash
colcon build
source install/setup.bash
```

# Launch Simulation
```ros2 launch drone_slam_pkg px4_gazebo.launch.py```
You can view this launch file and make some changes.
#  Run Teleoperation (in a new terminal)
```ros2 run teleop teleop```

# Run Keyboard Teleoperation (in another terminal)
```
ros2 run teleop_twist_rpyt_keyboard teleop_twist_rpyt_keyboard
```
You need to change the mode of drone to Teleoperation from QGC to operate it using Teleop from terminal

Note: If PX4 doesn't get ready for takeoff , it could be a resource issue, ensure your gpu is being used. You can change the PX4_SIM_SPEED_FACTOR to a lower value like 0.1.
```
echo "export PX4_SIM_SPEED_FACTOR=0.1" >> ~/.profile
source ~/.profile
```
