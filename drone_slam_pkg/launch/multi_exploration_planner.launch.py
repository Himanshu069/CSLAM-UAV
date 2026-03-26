#!/usr/bin/env python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    package_dir = get_package_share_directory('drone_slam_pkg')
    return LaunchDescription([

        Node(
            package='drone_slam_pkg',
            executable='multi_exploration_planner',
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
                ("cmd_vel", "/x500_drone_0/cmd_vel"),
                ("map","/x500_drone_0/map"),
                ("fmu/out/vehicle_local_position","/fmu/out/vehicle_local_position"),
                ("fmu/in/distance_sensor","/fmu/in/distance_sensor"),
                ("fmu/out/vehicle_attitude","/fmu/out/vehicle_attitude")
            ]
        ),
        Node(
            package='drone_slam_pkg',
            executable='multi_logger_exploration_planner',
            name='exploration_logger_0',
            namespace='x500_drone_0',
            output='screen',
            parameters=[
                {
                    "log_dir": os.path.expanduser("~/multi_exploration_logs"),
                    "drone_namespace": "x500_drone_0",
                }
            ],
            remappings=[
                ("fmu/out/vehicle_local_position", "/fmu/out/vehicle_local_position"),
            ]   
        ),
        Node(
            package='drone_slam_pkg',
            executable='multi_exploration_planner',
            name='exploration_planner_1',
            namespace='x500_drone_1',
            output='screen',
            parameters = [
                {
                    "drone_radius": 0.26,
                    "other_drone_pose_topic": '/x500_drone_0/localization_pose',
                    "other_drone_safety_radius": 3,
                    "other_drone_init_x": -2.0 ,
                    "other_drone_init_y": 0.0,
                    'map_frame': '/x500_drone_1/map'
                }
            ],
            remappings = [
                ("cmd_vel", "/x500_drone_1/cmd_vel"),
                ("map","/x500_drone_1/map"),
                ("fmu/out/vehicle_local_position","/px4_1/fmu/out/vehicle_local_position"),
                ("fmu/in/distance_sensor","/px4_1/fmu/in/distance_sensor"),
                ("fmu/out/vehicle_attitude","/px4_1/fmu/out/vehicle_attitude")
            ]
        ),
        Node(
            package='drone_slam_pkg',
            executable='multi_logger_exploration_planner',
            name='exploration_logger_1',
            namespace='x500_drone_1',
            output='screen',
            parameters=[
                {
                    "log_dir": os.path.expanduser("~/multi_exploration_logs"),
                    "drone_namespace": "x500_drone_1",
                }
            ],
            remappings=[
                ("fmu/out/vehicle_local_position", "/px4_1/fmu/out/vehicle_local_position"),
            ]   
        
        )
        # # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        # )
    ])
