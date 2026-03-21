#!/usr/bin/env python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('drone_slam_pkg')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    return LaunchDescription([

        Node(
            package='px4_ros_com',
            executable='offboard_cmd_vel',
            name='drone_0_control',
            namespace='x500_drone_0',
            output='screen',
            parameters =[
                {"use_sim_time": False}
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
