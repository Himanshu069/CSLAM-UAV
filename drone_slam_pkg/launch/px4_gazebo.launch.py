#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    px4_dir = os.path.join(os.getenv('HOME'), 'PX4-Autopilot')
    
    common_parameters = [{
        'subscribe_depth': True,
        'subscribe_rgbd': False,
        'subscribe_stereo': False,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'subscribe_imu': True,
        'use_sim_time': True,
        'always_check_imu_tf': False,
        'load_db': False,
        'wait_imu_to_init': True,
        'approx_sync_max_interval': 0.01
        #'queue_size': 10,
        #'sync_queue_size': 10,
    }]
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'make', '-C', px4_dir, 'px4_sitl', 'gz_x500_depth_baylands'],
            output='screen',
            shell=True
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', ' ./QGroundControl-x86_64.AppImage'],
            cwd=os.path.expanduser('~'),
            output='screen',
            shell=True
        ),
        ExecuteProcess(
             cmd=['MicroXRCEAgent','udp4', '--port', '8888']
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                         "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image",
                         "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                         "/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image",
                         "/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
                         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
                ],
            output='screen'
            ),
        Node(
             package='px4_ros_bridge',
             executable='px4_imu_bridge',
             output='screen',
             parameters=[{'use_sim_time': True}]
             ),
        Node(
             package='px4_ros_bridge',
             executable='px4_odom_bridge',
             output='screen',
             parameters=[{'use_sim_time': True}]
             ),
        Node(
            package='drone_slam_pkg',
            executable='odom_drone_tf',
            output='screen'
            ),
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments=['0','0','0','0','0','0','base_link','x500_depth_0'],
             output='screen'
             ),
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments=['0.12', '0.03', '0.242', '0', '0', '0', 'x500_depth_0', 'camera_link'],
             output='screen'
            ),
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments=['0.0123', '-0.03', '0.01878', '0', '0', '0', 'camera_link', 'x500_depth_0/camera_link/IMX214'],
             output='screen'
             ),
        Node(
              package='tf2_ros',
              executable='static_transform_publisher',
              arguments=['0.01233', '-0.03', '0.01878', '0', '0', '0', 'camera_link', 'x500_depth_0/camera_link/StereoOV7251'],
              output='screen'
              ),
    #    TimerAction(
    #             period=8.0,
    #             actions=[
    #                  ExecuteProcess(
    #                      cmd=[
    #                              'gnome-terminal', '--', 'ros2', 'launch', 'rtabmap_launch', 'rtabmap.launch.py',
    #                              'rtabmap_args:=--delete_db_on_start',
    #                              'rgb_topic:=/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
    #                              'depth_topic:=/depth_camera',
    #                              'camera_info_topic:=/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info',
    #                              'subscribe_rgbd:=true',
    #                              'frame_id:=x500_depth_0/camera_link/StereoOV7251',
    #                              'approx_sync:=false',
    #                              'wait_imu_to_init:=true',
    #                              'imu_topic:=/imu/data',
    #                              'odom_topic:=/odom',
    #                              'rviz:=true',
    #                              'use_sim_time:=true'
    #                      ],
    #                      output='screen'
    #                  )
    #                  ]
    #          )
        Node(
                package='rtabmap_odom',
                executable='rgbd_odometry',
                output='screen',
                parameters=common_parameters + [{
                    'frame_id': 'base_link',
                    'odom_frame_id': 'odom',
                    'publish_tf': True
                }],
                remappings=[
                    ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                    ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                    ('depth/image', '/depth_camera'),
                    ('odom_px4', '/odom'),
                    ('imu', '/imu/data')
                ]
            ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=common_parameters + [{
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'odom_sensor_sync': True
            }],
            remappings=[
                    ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                    ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                    ('depth/image', '/depth_camera'),
                    ('odom_px4', '/odom'),
                    ('imu', '/imu/data')
            ]
            ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=common_parameters,
            remappings=[
                    ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                    ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                    ('depth/image', '/depth_camera'),
                    ('odom_px4', '/odom'),
                    ('imu', '/imu/data')
            ]
            ),

        Node(
            package='rviz2',
            executable='rviz2',
            parameters=common_parameters,
            arguments=['-d', os.path.join(
                get_package_share_directory('rtabmap_rviz_plugins'), 'launch', 'rtabmap.rviz')],
            output='screen'
        )
       ])
