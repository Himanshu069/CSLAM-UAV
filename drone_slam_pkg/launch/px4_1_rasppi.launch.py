#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    #px4_dir = os.path.join(os.getenv("HOME"), "PX4-Autopilot")
    #yaml_file = os.path.expanduser('~/oakdprow.yaml')
    #with open(yaml_file,'r') as f:
    #    params = yaml.safe_load(f)
   # oak_name = "oak"
   # depthai_prefix = get_package_share_directory("depthai_ros_driver")
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
            'approx_sync_max_interval': 0.02,
            'queue_size': 50,
            'sync_queue_size': 50,

            'Odom/ResetCountdown': '1',     
            'Vis/MinInliers': '6',
            'Vis/InlierDistance': '0.1',        
            'wait_for_transform': 2.0,
            'Optimizer/GravitySigma': '0.3',
            'wait_imu_to_init': False,
            'publish_tf': True,
            # 'Vis/FeatureType': '10',
            'Kp/DetectorStrategy': '10',

            'Vis/MaxFeatures': '200',
            'Kp/MaxFeatures': '200',
            'Odom/Strategy': '1',
            'Vis/CorType': '1',
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',
            
            'Grid/RayTracing' : 'true',
            'NormalsSegmentation': 'false',
            'Grid/MaxGroundHeight': '1.15', 
            'Grid/MaxObstacleHeight': '1.75',
            'Grid/NoiseFilteringRadius': '0.15',
            'Grid/NoiseFilteringMinNeighbors': '7',
            
            'database_path': f'~/.ros/{db_name}.db'
        }

    
    
    
    return LaunchDescription([
       # ExecuteProcess(
       #     cmd=["micro-xrce-dds-agent", "udp4", "-p", "8888"],
       #     output="screen",
       # ),

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([
        #        os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        #   ]),
        #    launch_arguments={
        #        'align_depth.enable': 'true',       
        #        'pointcloud.enable': 'false',       
        #        'depth_module.profile': '640x480x15', 
        #        'rgb_camera.profile': '640x480x15'
        #    }.items()
       # ),
       # Node(
       #     package='topic_tools',
       #     executable='relay',
       #     name='relay_camera_info',
       #     arguments=[
       #         '/camera/camera/aligned_depth_to_color/camera_info',
       #         '/x500_drone_0/rgb/camera_info'
       #  ],
       # output='screen'
       # ),
       # Node(
       #     package='topic_tools',
       #     executable='relay',
       #     name='relay_depth_image',
       #     arguments=[
       #         '/camera/camera/aligned_depth_to_color/image_raw',
       #         '/x500_drone_0/depth/image'
       # ],
       # output='screen'
       # ),
       # Node(
       #     package='topic_tools',
       #     executable='relay',
       #     name='relay_rgb_image',
       #     arguments=[
       #         '/camera/camera/color/image_raw',
       #         '/x500_drone_0/rgb/image'
       # ],
       # output='screen'
       #),

       # Node(
       #     package='depthai_ros_driver',
       #     executable='camera_node',
       #     name='oak',
       #     output='screen',
       #     parameters=[params],
       #     remappings=[
       #         ('/camera/rgb/camera_info', '/x500_drone_0/rgb/camera_info'),
       #         ('/camera/rgb/image_raw','/x500_drone_0/rgb/image'),
       #         ('/camera/stereo/image_raw','/x500_drone_0/depth/image'),
       #     ]
       # ),
       # IncludeLaunchDescription(
       #     PythonLaunchDescriptionSource(
       #         os.path.join(depthai_prefix, "launch", "camera.launch.py")
       #     ),
       #     launch_arguments={
       #         "name": oak_name,
       #         "params_file": os.path.expanduser("~/oak_run.yaml"),
       #     }.items(),
       # ),
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_color_image',
            prefix='taskset -c 0',
            output='screen',
            parameters=[{'use_sim_time': False}],
            remappings=[
                ('image',       '/x500_drone_1/rgb/image_raw'),
                ('camera_info', '/x500_drone_1/rgb/camera_info'),
                ('image_rect',  '/x500_drone_1/rgb/image') 
            ],
        ),
          

     #Drone 0
        Node(package='tf2_ros', executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'x500_drone_1/base_link', 'x500_drone_1/imu_sensor']),
        
        Node(package='tf2_ros', executable='static_transform_publisher',
                arguments=['0.12', '0.03', '0.242', '-1.5708', '0', '-1.5708', 'x500_drone_1/base_link', 'x500_drone_1/camera_link']),

        Node(package='tf2_ros', executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'x500_drone_1/camera_link', 'camera_rgb_camera_optical_frame']),


        Node(
            package='px4_ros_com',             
            executable='imu_bridge',           
            name='px4_imu_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'vehicle_ns': 'x500_drone_1'},
                {'gyro_noise': 0.0150},        # EKF2_GYR_NOISE
                {'accel_noise': 0.500},        # EKF2_ACC_NOISE
            ],
            remappings=[
                ('/fmu/out/vehicle_attitude', '/px4_1/fmu/out/vehicle_attitude'),
                ('/fmu/out/sensor_combined', '/px4_1/fmu/out/sensor_combined'),
            ]
        ),
        #Drone 0
        TimerAction(
                period=5.0,
           actions=[
                Node(
                    package='imu_filter_madgwick',
                    executable='imu_filter_madgwick_node',
                    name='imul_filter_1',
                    output='screen',
                    parameters=[{
                        'use_mag': False,
                        'world_frame': 'enu',
                        'publish_tf': False,
                        'use_sim_time': False,
                    }],
                    remappings=[
                        ('imu/data_raw', '/x500_drone_1/imu/data_raw'),
                        ('imu/data',     '/x500_drone_1/imu/data'),
                    ]
                ),
                Node(
                    package='rtabmap_util',
                    executable='imu_to_tf',
                    name='imu_to_tf_1',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'fixed_frame_id': 'x500_drone_1/base_link_stabilized',
                        'base_frame_id': 'x500_drone_1/base_link',
                        'wait_for_transform': 1.0,
                    }],
                    remappings=[
                        ('imu/data', '/x500_drone_1/imu/data'),
                    ]
                ),
            ]
        ),
        
        TimerAction(
                period= 10.0,
            actions=[
               # Node(
               #     package='topic_tools',
               #     executable='relay',
               #     name='relay_rgb_image',
               #     arguments=[f'/{oak_name}/rgb/image_raw',
               #                 '/x500_drone_0/rgb/image'],
               #     output='screen'
               # ),
               # Node(
               #     package='topic_tools',
               #     executable='relay',
               #     name='relay_camera_info',
               #     arguments=[f'/{oak_name}/rgb/camera_info',
               #                 '/x500_drone_0/rgb/camera_info'],
               #     output='screen'
               # ),
               # Node(
               #     package='topic_tools',
               #     executable='relay',
               #     name='relay_depth_image',
               #     arguments=[f'/{oak_name}/stereo/image_raw',
               #                 '/x500_drone_0/depth/image'],
               #     output='screen'
               # ),
                Node(
                    package="rtabmap_sync",
                    executable="rgbd_sync",
                    name="rgbd_sync_x500_drone_1",
                    prefix='taskset -c 1',
                    namespace="x500_drone_1",
                    output="screen",
                    parameters=[{
                        "use_sim_time": False,
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.05,
                        "queue_size": 50,
                        "sync_queue_size": 50,
                    }],
                    remappings=[
                        ("rgb/image", "/x500_drone_1/rgb/image"),
                        ("rgb/camera_info", "/x500_drone_1/rgb/camera_info"),
                        ("depth/image", "/x500_drone_1/depth/image"),
                    ],
                ),
                Node(
                    package="rtabmap_odom",
                    executable="rgbd_odometry",
                    name="rgbd_odometry_1",
                    prefix="taskset -c 0,1",
                    namespace="x500_drone_1",
                    output="screen",
                    parameters=[get_vslam_params("x500_drone_1", "rtabmap_drone_1"),
                                {
                                    'RGBD/CreateOccupancyGrid': 'false',
                                    'Rtabmap/DetectionRate': '0',
                                    'Mem/STMSize': '5',
                                }
                            ],
                    remappings=[
                        ("imu", "/x500_drone_1/imu/data"),
                        ("map", "/x500_drone_1/map"),
                        ("odom", "/x500_drone_1/odom"),  
                    ],
                ),
                Node(
                    package='px4_ros_com',
                    executable='ros_odometry_to_vehicle_odometry',
                    name='ros_odometry_to_vehicle_odometry_0',
                    prefix='taskset -c 0',
                    parameters=[
                        {"odom_topic": "/x500_drone_1/odom"},
                        {"vehicle_odometry_topic": "/px4_1/fmu/in/vehicle_visual_odometry"},
                        {"map_frame_id": "x500_drone_1/odom"},
                        {"repeat_odom": False}
                    ],
                    output='screen'
                ),

        # Node(
        #     package="rtabmap_viz",
        #     executable="rtabmap_viz",
        #     name="rtabmap_viz_0",
        #     namespace="x500_drone_0",
        #     output="screen",
        #     parameters=[get_vslam_params("x500_drone_0", "rtabmap_drone_0")],
        #     remappings=[
        #         ("imu", "/x500_drone_0/imu/data"),
        #         ("odom", "/x500_drone_0/odom"),
        #         ("map", "/x500_drone_0/map"),
        #     ],
        # ),
        ]
        ),
        TimerAction(
            period= 20.0,
            actions=[
                Node(
                    package="rtabmap_slam",
                    executable="rtabmap",
                    name="rtabmap_1",
                    prefix="taskset -c 2",
                    namespace="x500_drone_1",
                    output="screen",
                    parameters=[get_vslam_params("x500_drone_1", "rtabmap_drone_1"),
                                {
                                    'RGBD/CreateOccupancyGrid': 'true',
                                    'Rtabmap/DetectionRate': '1',
                                    'Mem/STMSize': '10',
                                }
                                ],
                    remappings=[
                        ("imu", "/x500_drone_1/imu/data"),
                        ("odom", "/x500_drone_1/odom"),
                        ("map", "/x500_drone_1/map"),
                    ],
                    arguments=["-d"],
                ),

            ]
        ),
    ])
