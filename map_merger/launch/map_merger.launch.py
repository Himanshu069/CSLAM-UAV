"""
map_merger.launch.py
--------------------
Launches the map_merger_node with parameters loaded from config/params.yaml.

Usage:
  ros2 launch map_merger map_merger.launch.py

Override individual parameters on the command line:
  ros2 launch map_merger map_merger.launch.py map2_x:=5.0 map2_yaw:=1.5708
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Overrideable arguments ─────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('map1_topic',   default_value='/x500_drone_0/map'),
        DeclareLaunchArgument('map2_topic',   default_value='/x500_drone_1/map'),
        DeclareLaunchArgument('merged_topic', default_value='/merged_map'),
        DeclareLaunchArgument('map1_frame',   default_value='x500_drone_0/map'),
        DeclareLaunchArgument('map2_frame',   default_value='x500_drone_1/map'),
        DeclareLaunchArgument('merged_frame', default_value='world'),

        DeclareLaunchArgument('map1_flip_x', default_value='false'),
        DeclareLaunchArgument('map1_flip_y', default_value='false'),
        DeclareLaunchArgument('map1_rot90',  default_value='0'),
        DeclareLaunchArgument('map2_flip_x', default_value='false'),
        DeclareLaunchArgument('map2_flip_y', default_value='false'),
        DeclareLaunchArgument('map2_rot90',  default_value='0'),

        DeclareLaunchArgument('map1_x',   default_value='0.0'),
        DeclareLaunchArgument('map1_y',   default_value='0.0'),
        DeclareLaunchArgument('map1_yaw', default_value='0.0'),

        DeclareLaunchArgument('map2_x',   default_value='0.0'),
        DeclareLaunchArgument('map2_y',   default_value='-3.0'),
        DeclareLaunchArgument('map2_yaw', default_value='0.0'),

        DeclareLaunchArgument('publish_rate', default_value='1.0'),
        DeclareLaunchArgument('merge_mode',   default_value='max'),
        DeclareLaunchArgument('border_m',     default_value='0.5'),
    ]

    merger_node = Node(
        package='map_merger',
        executable='map_merger_node',
        name='map_merger',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('map_merger'), 'config', 'params.yaml']
            ),
            {
                'map1_topic':   LaunchConfiguration('map1_topic'),
                'map2_topic':   LaunchConfiguration('map2_topic'),
                'merged_topic': LaunchConfiguration('merged_topic'),
                'map1_frame':   LaunchConfiguration('map1_frame'),
                'map2_frame':   LaunchConfiguration('map2_frame'),
                'merged_frame': LaunchConfiguration('merged_frame'),
                'map1_x':   LaunchConfiguration('map1_x'),
                'map1_y':   LaunchConfiguration('map1_y'),
                'map1_yaw': LaunchConfiguration('map1_yaw'),
                'map2_x':       LaunchConfiguration('map2_x'),
                'map2_y':       LaunchConfiguration('map2_y'),
                'map2_yaw':     LaunchConfiguration('map2_yaw'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'merge_mode':   LaunchConfiguration('merge_mode'),
                'border_m':     LaunchConfiguration('border_m'),
            },
        ],
    )

    return LaunchDescription(args + [merger_node])
