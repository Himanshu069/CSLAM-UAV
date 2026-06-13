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
        Node(
           package='px4_ros_com',
           executable='offboard_cmd_vel',
           name='drone_1_control',
           namespace='x500_drone_1',
           output='screen',
           parameters =[
               {"use_sim_time": True}
           ],
           remappings=[
                ("/cmd_vel","/x500_drone_1/cmd_vel"), 
                ("/fmu/in/offboard_control_mode", "/px4_1/fmu/in/offboard_control_mode"),
                ("/fmu/in/trajectory_setpoint", "/px4_1/fmu/in/trajectory_setpoint"),
                ("/fmu/in/vehicle_command", "/px4_1/fmu/in/vehicle_command"),
                ("/fmu/out/vehicle_local_position", "/px4_1/fmu/out/vehicle_local_position"),
                ("/fmu/out/vehicle_status_v1", "/px4_1/fmu/out/vehicle_status_v1"),
                ("/fmu/out/vehicle_command_ack", "/px4_1/fmu/out/vehicle_command_ack"),
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
