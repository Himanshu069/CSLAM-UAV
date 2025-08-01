Collaborative SLAM in UAVs

Firstly , Clone the PX4-Autopilot repository and then checkout to the branch release/1.15

Then, run the commands 
git config -f .gitmodules submodule.Tools/simulation/gz.url https://github.com/Himanshu069/PX4-gazebo-models.git
git submodule sync
git submodule update --remote Tools/simulation/gz



After building the packages , run

cd ~/cslam_ws/src/px4_ros_com_multi_vehicle/scripts/ && ./multi_drone_launch.sh
