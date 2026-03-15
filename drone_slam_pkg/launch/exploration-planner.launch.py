#!/usr/bin/env python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('drone_control_pkg')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    return LaunchDescription([

        Node(
            package='drone_slam_pkg',
            executable='exploration_planner',
            name='exploration_planner_0',
            namespace='x500_drone_0',
            output='screen',
            parameters = [
                {
                    "drone_radius": 0.26,
                    "other_drone_pose_topic": '/x500_drone_1/localization_pose',
                    "other_drone_safety_radius": 3,
                    "other_drone_init_x": 2.0 ,
                    "other_drone_init_y": 0.0,
                    'map_frame': '/x500_drone_0/map'
                }
            ],
            remappings = [
                ("map","/x500_drone_0/map"),
                ("fmu/out/vehicle_local_position","/fmu/out/vehicle_local_position"),
                ("fmu/in/distance_sensor","/fmu/in/distance_sensor"),
                ("fmu/out/vehicle_attitude","/fmu/out/vehicle_attitude")
            ]
        ),
        Node(
            package='px4_ros_com',
            executable='offboard_cmd_vel',
            name='drone_0_control',
            namespace='x500_drone_0',
            output='screen',
            parameters =[
                {"use_sim_time": True}
            ],
            remappings=[
                ("/cmd_vel","/x500_drone_0/cmd_vel"), 
            ]
        ),
        # # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        # )
    ])
