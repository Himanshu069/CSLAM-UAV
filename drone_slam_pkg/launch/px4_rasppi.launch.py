#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    def get_vslam_params(drone_ns, db_name):
        return {
           'use_sim_time': False,
            'frame_id': f'{drone_ns}/base_link',
            'guess_frame_id':f'{drone_ns}/base_link_stabilized',
            'map_frame_id': f'{drone_ns}/map',
            'odom_frame_id': f'{drone_ns}/odom',
            

            'subscribe_rgbd': False,
            'subscribe_depth': True,
            'subscribe_odom': True,
            'subscribe_imu': True,
            'approx_sync': True,
            'queue_size': 20,
            'sync_queue_size': 10,

            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.1',        
            'wait_for_transform': 2.0,
            'Optimizer/GravitySigma': '0',
            'wait_imu_to_init': False,
            # 'Vis/FeatureType': '10',
            'Kp/DetectorStrategy': '10',

            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            
            'Odom/Strategy': '1',
            'Vis/MaxFeatures': '300',
            
            'Grid/RayTracing' : 'true',
            'NormalsSegmentation': 'true',
            'Grid/MaxGroundHeight': '0.5', 
            'Grid/MaxObstacleHeight': '1.10',
            'Grid/NoiseFilteringRadius': '0.15',
            'Grid/NoiseFilteringMinNeighbors': '7',
            
            'database_path': f'~/.ros/{db_name}.db'
        }

    
    
    
    return LaunchDescription([
       # ExecuteProcess(
       #     cmd=["micro-xrce-dds-agent", "udp4", "-p", "8888"],
       #     output="screen",
       # ),

        ExecuteProcess(
            cmd=[
                'taskset', '-c', '3',
                'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                'align_depth.enable:=true',
                'enable_sync:=true',
                'pointcloud.enable:=false',
                'depth_module.depth_profile:=424x240x15',
                'rgb_camera.color_profile:=424x240x15',
            ],
        output='screen'
        ),

        Node(
            package='topic_tools',
            executable='relay',
            name='relay_camera_info',
            prefix=['taskset', '-c', '2'],
            arguments=[
                '/camera/camera/color/camera_info',
                '/x500_drone_0/rgb/camera_info'
         ],
        output='screen'
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_depth_image',
            prefix=['taskset', '-c', '2'],
            arguments=[
                '/camera/camera/aligned_depth_to_color/image_raw',
                '/x500_drone_0/depth/image'
        ],
        output='screen'
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_rgb_image',
            prefix=['taskset', '-c', '2'],
            arguments=[
                '/camera/camera/color/image_raw',
                '/x500_drone_0/rgb/image'
        ],
        output='screen'
       ),

          

        Node(package='tf2_ros', executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'x500_drone_0/base_link', 'x500_drone_0/imu_sensor']),
        
        Node(package='tf2_ros', executable='static_transform_publisher',
                arguments=['0.12', '0.03', '0.242', '0', '0', '0', 'x500_drone_0/base_link', 'camera_link']),

       # Node(package='tf2_ros', executable='static_transform_publisher',
       #         arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'x500_drone_0/camera_link', 'camera_rgb_camera_optical_frame']),

        Node(
            package='px4_ros_com',             
            executable='imu_bridge',           
            name='px4_imu_bridge',
            prefix=['taskset', '-c', '2'],
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'vehicle_ns': 'x500_drone_0'},
                {'gyro_noise': 0.0150},        # EKF2_GYR_NOISE
                {'accel_noise': 0.500},        # EKF2_ACC_NOISE
            ],
            remappings=[
                ('/fmu/out/vehicle_attitude', '/fmu/out/vehicle_attitude'),
                ('/fmu/out/sensor_combined', '/fmu/out/sensor_combined'),
            ]
        ),

        #Drone 0
        TimerAction(
           period=10.0,
           actions=[
                Node(
                    package='imu_filter_madgwick',
                    executable='imu_filter_madgwick_node',
                    name='imul_filter_0',
                    prefix=['taskset', '-c', '2'],
                    output='screen',
                    parameters=[{
                        'use_mag': False,
                        'world_frame': 'enu',
                        'publish_tf': False,
                        'use_sim_time': False,
                    }],
                    remappings=[
                        ('imu/data_raw', '/x500_drone_0/imu/data_raw'),
                        ('imu/data',     '/x500_drone_0/imu/data'),
                    ]
                ),
                Node(
                    package='rtabmap_util',
                    executable='imu_to_tf',
                    name='imu_to_tf_0',
                    prefix=['taskset', '-c', '2'],
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'fixed_frame_id': 'x500_drone_0/base_link_stabilized',
                        'base_frame_id': 'x500_drone_0/base_link',
                    }],
                    remappings=[
                        ('imu/data', '/x500_drone_0/imu/data'),
                    ]
                ),
            ]
        ),
        
        TimerAction(
            period= 20.0,
            actions=[
                Node(
                    package="rtabmap_sync",
                    executable="rgbd_sync",
                    name="rgbd_sync_x500_drone_0",
                    prefix=['taskset', '-c', '0'],
                    namespace="x500_drone_0",
                    output="screen",
                    parameters=[{
                        "use_sim_time": False,
                        "approx_sync": False,
                        "queue_size": 20,
                        "sync_queue_size": 10,
                    }],
                    remappings=[
                        ("rgb/image", "/x500_drone_0/rgb/image"),
                        ("rgb/camera_info", "/x500_drone_0/rgb/camera_info"),
                        ("depth/image", "/x500_drone_0/depth/image"),
                    ],
                ),
                Node(
                    package="rtabmap_odom",
                    executable="rgbd_odometry",
                    name="rgbd_odometry_0",
                    prefix=['taskset', '-c', '0'],
                    namespace="x500_drone_0",
                    output="screen",
                    parameters=[get_vslam_params("x500_drone_0", "rtabmap_drone_0")],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        ("map", "/x500_drone_0/map"),
                        ("odom", "/x500_drone_0/odom"),  
                    ],
                ),
                Node(
                    package='px4_ros_com',
                    executable='ros_odometry_to_vehicle_odometry',
                    name='ros_odometry_to_vehicle_odometry_0',
                    prefix=['taskset', '-c', '2'],
                    parameters=[
                        {"odom_topic": "/x500_drone_0/odom"},
                        {"vehicle_odometry_topic": "/fmu/in/vehicle_visual_odometry"},
                        {"map_frame_id": "x500_drone_0/map"},
                        {"repeat_odom": False}
                    ],
                    output='screen'
                ),

            ]
        ),

        TimerAction(
            period=30.0,
            actions=[
                    Node(
                    package="rtabmap_slam",
                    executable="rtabmap",
                    name="rtabmap_0",
                    prefix=['taskset', '-c', '2'],
                    namespace="x500_drone_0",
                    output="screen",
                    parameters=[get_vslam_params("x500_drone_0", "rtabmap_drone_0")],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        ("odom", "/x500_drone_0/odom"),
                        ("map", "/x500_drone_0/map"),
                    ],
                    arguments=["-d"],
                ),

            ]
        ),
    ])
