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

        # ExecuteProcess(
        #     cmd=['make', '-C', px4_dir, 'px4_sitl', 'gz_x500_depth_baylands'],
        #     output='screen'
        # ),
        TimerAction(
            period=10.0,  
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c', 
                        'cd ' + px4_dir + ' && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,6" PX4_SIM_MODEL=gz_x500_depth ./build/px4_sitl_default/bin/px4 -i 1; exec bash'],
                    output='screen'
                )
            ]
        ),
        # TimerAction(
        #     period=10.0,
        #     actions = [
        #         ExecuteProcess(
        #             cmd=[
        #                 'PX4_SYS_AUTOSTART=4001',
        #                 'PX4_GZ_MODEL_POSE="0,1"',
        #                 'PX4_SIM_MODEL=gz_x500_depth',
        #                 os.path.join(px4_dir, 'build/px4_sitl_default/bin/px4'),
        #                 '-i', '1'
        #     ],
        #     cwd=px4_dir,
        #     output='screen',
        #         )
        #     ]
        # ),
        ExecuteProcess(
            cmd=['./QGroundControl-x86_64.AppImage'],
            cwd=os.path.expanduser('~'),
            output='screen',
        ),
        ExecuteProcess(
             cmd=['MicroXRCEAgent','udp4', '--port', '8888']
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
                ],
            output='screen'
            ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image",
                "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                "/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image",
                "/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            ],
            remappings=[
                ('/depth_camera', '/drone_0/depth_camera'),
                ('/depth_camera/points', '/drone_0/depth_camera/points'),
            ],
            output='screen'
            ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/world/baylands/model/x500_depth_1/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image",
                "/world/baylands/model/x500_depth_1/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                "/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image",
                "/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            ],
            remappings=[
                ('/depth_camera', '/drone_1/depth_camera'),
                ('/depth_camera/points', '/drone_1/depth_camera/points'),
            ],
            output='screen'
        ),
        # Node(
        #     package='px4_ros_bridge',
        #     executable='px4_imu_bridge',
        #     name='px4_imu_bridge_0',
        #     namespace='x500_drone_0',
        #     output='screen',
        #     parameters=[
        #         {'use_sim_time': True},
        #         {'vehicle_ns': ''}
        #     ]
        # ),
        # Node(
        # package='px4_ros_bridge',
        # executable='px4_imu_bridge',
        # name='px4_imu_bridge_1',
        # namespace='x500_drone_1',
        # output='screen',
        # parameters=[
        #     {'use_sim_time': True},
        #     {'px4_ns': 'px4_1'},
        #     {'vehicle_ns': ''}
        # ]
        # ),
        Node(
            package='px4_ros_bridge',
            executable='px4_odom_bridge',
            name='px4_odom_bridge_0',
            parameters=[
                {'use_sim_time': True},
                {'vehicle_ns': 'x500_drone_0'}
            ]
        ),
        Node(
            package='px4_ros_bridge',
            executable='px4_odom_bridge',
            name='px4_odom_bridge_1',
            parameters=[
                {'use_sim_time': True},
                {'px4_ns': 'px4_1'},
                {'vehicle_ns': 'x500_drone_1'}
            ]
        ),
        Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_imu_odom_bridge_0',
        # namespace='x500_drone_0',
        parameters=[{'use_sim_time': True}],
        arguments=[
            "/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/model/x500_depth_0/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance"
        ],
            remappings=[
                (
                    "/world/baylands/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu",
                    "/x500_drone_0/imu/data"
                )
                # (
                #     "/model/x500_depth_0/odometry_with_covariance",
                #     "x500_drone_0/odom"
                # )
            ],
            output='screen'
        ),
        Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_imu_odom_bridge_1',
        # namespace='x500_drone_1',
        parameters=[{'use_sim_time': True}],
        arguments=[
            "/world/baylands/model/x500_depth_1/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/model/x500_depth_1/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance"
        ],
        remappings=[
            (
                "/world/baylands/model/x500_depth_1/link/base_link/sensor/imu_sensor/imu",
                "/x500_drone_1/imu/data"
            )
            # (
            #     "/model/x500_depth_1/odometry_with_covariance",
            #     "x500_drone_1/odom"
            # )
        ],
        output='screen'
        ),
        Node(
            package='drone_slam_pkg',
            executable='odom_drone_tf',
            name= 'odom_tf_publisher_0',
            parameters=[
                {'use_sim_time': True},
                {'vehicle_ns': 'x500_drone_0'}
            ]       
        ),
        Node(
        package='drone_slam_pkg',
        executable='odom_drone_tf',
        name='odom_tf_publisher_1',
        parameters=[
            {'use_sim_time': True},
            {'px4_ns': 'px4_1'},
            {'vehicle_ns': 'x500_drone_1'}
            ]
        ),
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments=['0','0','0','0','0','0','x500_drone_0/base_link','x500_drone_0/x500_depth_0'],
             output='screen'
             ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'x500_drone_0/base_link', 'x500_depth_0/base_link/imu_sensor'],
            output='screen'
            ),
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments=['0.12', '0.03', '0.242', '0', '0', '0', 'x500_drone_0/x500_depth_0', 'x500_drone_0/camera_link'],
             output='screen'
            ),
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             arguments=['0.0123', '-0.03', '0.01878', '0', '0', '0', 'x500_drone_0/camera_link', 'x500_depth_0/camera_link/IMX214'],
             output='screen'
             ),
        Node(
              package='tf2_ros',
              executable='static_transform_publisher',
              arguments=['0.01233', '-0.03', '0.01878', '0', '0', '0', 'x500_drone_0/camera_link', 'x500_drone_0/camera_link/StereoOV7251'],
              output='screen'
              ),
        #-----------------Drone 1-----------------      
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0','x500_drone_1/base_link','x500_drone_1/x500_depth_1'],
            output='screen'
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'x500_drone_1/base_link', 'x500_depth_1/base_link/imu_sensor'],
            output='screen'
            ),    
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.12', '0.03', '0.242', '0', '0', '0', 'x500_drone_1/x500_depth_1', 'x500_drone_1/camera_link'],
            output='screen'
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0123', '-0.03', '0.01878', '0', '0', '0', 'x500_drone_1/camera_link', 'x500_depth_1/camera_link/IMX214'],
            output='screen'
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.01233', '-0.03', '0.01878', '0', '0', '0', 'x500_drone_1/camera_link', 'x500_drone_1/camera_link/StereoOV7251'],
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
                # namespace = 'x500_drone_0',
                output='screen',
                parameters=common_parameters + [{
                    'frame_id': 'x500_drone_0/base_link',
                    'odom_frame_id': 'x500_drone_0/odom',
                    'map_frame_id': 'x500_drone_0/map',
                    'publish_tf': False
                }],
                remappings=[
                    ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                    ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                    ('depth/image', '/drone_0/depth_camera'),
                    ('odom_px4', '/x500_drone_0/odom'),
                    ('imu', '/x500_drone_0/imu/data')
                ]
            ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            # namespace='x500_drone_0',
            output='screen',
            parameters=common_parameters + [{
                'frame_id': 'x500_drone_0/base_link',
                'map_frame_id': 'x500_drone_0/map',
                'odom_frame_id': 'x500_drone_0/odom',
                'publish_tf': True,
                'odom_sensor_sync': True
            }],
            remappings=[
                    ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                    ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                    ('depth/image', '/drone_0/depth_camera'),
                    ('odom_px4', '/x500_drone_0/odom'),
                    ('imu', '/x500_drone_0/imu/data')
            ]
            ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            # namespace='x500_drone_0',
            output='screen',
            parameters=common_parameters,
            remappings=[
                    ('rgb/image', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                    ('rgb/camera_info', '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                    ('depth/image', '/drone_0/depth_camera'),
                    ('odom_px4', '/x500_drone_0/odom'),
                    ('imu', '/x500_drone_0/imu/data')
            ]
            ),
                        #-----------------Drone1---------------
        Node(
                package='rtabmap_odom',
                executable='rgbd_odometry',
                name = 'x500_drone_1',
                # namespace = 'x500_drone_1',
                output='screen',
                parameters=common_parameters + [{
                    'frame_id': 'x500_drone_1/base_link',
                    'odom_frame_id': 'x500_drone_1/odom',
                    'map_frame_id': 'x500_drone_1/map',
                    'publish_tf': False
                }],
                remappings=[
                    ('rgb/image', '/world/baylands/model/x500_depth_1/link/camera_link/sensor/IMX214/image'),
                    ('rgb/camera_info', '/world/baylands/model/x500_depth_1/link/camera_link/sensor/IMX214/camera_info'),
                    ('depth/image', '/drone_1/depth_camera'),
                    ('odom_px4', '/x500_drone_1/odom'),
                    ('imu', '/x500_drone_1/imu/data')
                ]
            ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name = 'x500_drone_1',
            # namespace='x500_drone_1',
            output='screen',
            parameters=common_parameters + [{
                'frame_id': 'x500_drone_1/base_link',
                'map_frame_id': 'x500_drone_1/map',
                'odom_frame_id': 'x500_drone_1/odom',
                'publish_tf': True,
                'odom_sensor_sync': True
            }],
            remappings=[
                    ('rgb/image', '/world/baylands/model/x500_depth_1/link/camera_link/sensor/IMX214/image'),
                    ('rgb/camera_info', '/world/baylands/model/x500_depth_1/link/camera_link/sensor/IMX214/camera_info'),
                    ('depth/image', '/drone_1/depth_camera'),
                    ('odom_px4', '/x500_drone_1/odom'),
                    ('imu', '/x500_drone_1/imu/data')
            ]
            ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name = 'x500_drone_1',
            # namespace='x500_drone_1',
            output='screen',
            parameters=common_parameters,
            remappings=[
                    ('rgb/image', '/world/baylands/model/x500_depth_1/link/camera_link/sensor/IMX214/image'),
                    ('rgb/camera_info', '/world/baylands/model/x500_depth_1/link/camera_link/sensor/IMX214/camera_info'),
                    ('depth/image', '/drone_1/depth_camera'),
                    ('odom_px4', '/x500_drone_1/odom'),
                    ('imu', '/x500_drone_1/imu/data')
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
