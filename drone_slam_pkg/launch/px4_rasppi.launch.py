#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    px4_dir = os.path.join(os.getenv("HOME"), "PX4-Autopilot")
    yaml_file = os.path.expanduser('~/oak_run.yaml')
    with open(yaml_file,'r') as f:
        params = yaml.safe_load(f)

    def get_vslam_params(drone_ns, db_name):
        return {
            'use_sim_time': False,
            'frame_id': f'{drone_ns}/base_link',
            'guess_frame_id':f'{drone_ns}/base_link_stabilized',
            'map_frame_id': f'{drone_ns}/map',
            'odom_frame_id': f'{drone_ns}/odom',
            
            # 'subscribe_rgbd': True,
            'subscribe_depth': True,
            'subscribe_odom_info':True,
            'wait_for_transform': 0.5,
            # 'tf_delay': 0.1,
            # 'tf_tolerance': 0.5,
            # 'subscribe_imu': True,
            'approx_sync': True, 
            'wait_imu_to_init': False,
            # 'publish_tf': True,
            # 'queue_size': 200,
            # 'sync_queue_size': 100,
            
            # 'Odom/ResetCountdown': '5',     
            # 'Vis/MinInliers': '10',         
            # 'Odom/Strategy': '0',           
            # 'wait_for_transform': 0.5,
            # 'Optimizer/GravitySigma': '0.3',

            'Optimizer/GravitySigma': '0.1',
            'Vis/FeatureType': '6',
            'Kp/DetectorStrategy': '6',
            'Grid/MapFrameProjection': 'true',
            'NormalsSegmentation': 'false',
            # 'Grid/MinGroundHeight': '-0.2',
            'Grid/MaxGroundHeight': '1.15' ,
            'Grid/MaxObstacleHeight': '1.75',
            'RGBD/StartAtOrigin': 'true',
            # 'Grid/GroundIsObstacle': 'false',
            'Grid/RayTracing': 'true',

            # 'Grid/MaxGroundHeight': '0.2',   # indoor
            # 'Grid/MinGroundHeight': '-0.2',
            # 'Grid/MinObstacleHeight': '2.0',

            # 'Grid/3D': True,
            # 'Grid/MaxGroundHeight': '0.1', 
            # 'Grid/MaxObstacleHeight': '2.1',
            # 'Grid/NoiseFilteringRadius': '0.05',
            # 'Grid/NoiseFilteringMinNeighbors': '2',
            
            'database_path': f'~/.ros/{db_name}.db'
        }

    
    
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=["micro-xrce-dds-agent", "udp4", "-p", "8888"],
            output="screen",
        ),

        Node(
            package='px4_ros_bridge',
            executable='px4_imu_bridge',
            name='px4_imu_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'vehicle_ns': 'x500_drone_0'}
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
           ]),
            launch_arguments={
                'align_depth.enable': 'true',       
                'pointcloud.enable': 'false',       
                'depth_module.profile': '640x480x15', 
                'rgb_camera.profile': '640x480x15'
            }.items()
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_camera_info',
            arguments=[
                '/camera/camera/aligned_depth_to_color/camera_info',
                '/x500_drone_0/rgb/camera_info'
         ],
        output='screen'
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_depth_image',
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
            arguments=[
                '/camera/camera/color/image_raw',
                '/x500_drone_0/rgb/image'
        ],
        output='screen'
       ),

        #Node(
        #    package='depthai_ros_driver',
        #    executable='camera_node',
        #    name='oak',
        #    output='screen',
        #    parameters=[params],
        #    remappings=[
        #        ('/camera/rgb/camera_info', '/x500_drone_0/rgb/image'),
        #        ('/camera/rgb/image_raw','/x500_drone_0/rgb/image'),
        #        ('/camera/stereo/image_raw','/x500_drone_0/depth/image'),
        #    ]
        #),

        #Drone 0
        Node(package='tf2_ros', executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'x500_drone_0/base_link', 'x500_depth_0/base_link/imu_sensor']),
        
        Node(package='tf2_ros', executable='static_transform_publisher',
                arguments=['0.12', '0.03', '0.242', '0', '1.570796327', '0', 'x500_drone_0/base_link', 'x500_drone_0/camera_link']),
             
        #Drone 0
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imul_filter_0',
            namespace='x500_drone_0',
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
            namespace='x500_drone_0',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'fixed_frame_id': 'x500_drone_0/base_link_stabilized',
                'base_frame_id': 'x500_drone_0/base_link',
            }],
            remappings=[
                ('imu', '/x500_drone_0/imu/data'),
            ]
        ),
        Node(
            package="rtabmap_sync",
            executable="rgbd_sync",
            name="rgbd_sync_x500_drone_0",
            namespace="x500_drone_0",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "approx_sync": True,
                "approx_sync_max_interval": 0.04,
                "queue_size": 200,
                "sync_queue_size": 100,
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
            namespace="x500_drone_0",
            output="screen",
            parameters=[get_vslam_params("x500_drone_0", "rtabmap_drone_0")],
            remappings=[
                ("imu", "/x500_drone_0/imu/data"),
                ("map", "/x500_drone_0/map"),
            ],
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap_0",
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
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            name='pointcloud_xyz_0',
            namespace= 'x500_drone_0',
            parameters=[{'decimation': 4,
                        'max_depth': 3.0,
                        'voxel_size': 0.10,
                        'use_sim_time': False}],
            remappings=[('depth/image', '/x500_drone_0/depth/image'),
                        ('depth/camera_info', '/x500_drone_0/rgb/camera_info'),
                        ('cloud', '/x500_drone_0/camera/cloud')]
        ),
        Node(
            package='rtabmap_costmap_plugins', executable='voxel_marker', output='screen',
            name='voxel_marker_0',
            namespace="/x500_drone_0",
            parameters=[{'use_sim_time': False}]
        ),

        Node(
            package='px4_ros_com',         
            executable='ros_odometry_to_vehicle_odometry',  
            name='ros_odometry_to_vehicle_odometry_0',
            parameters=[
                {"odom_topic": "/x500_drone_0/odom"},
                {"vehicle_odometry_topic": "/fmu/in/vehicle_visual_odometry"},
                {"map_frame_id": "/x500_drone_0/map"},   
                {"repeat_odom": True}      
            ],
            output='screen'
            )
            #     package='drone_slam_pkg',
                #     executable='offboard_control',
                #     name='drone_0_control',
                #     namespace='x500_drone_0',
                #     output='screen',
                #     prefix='xterm -hold -e',
                #     parameters =[
                #         {"use_sim_time": True}
                #     ],
                #     remappings=[
                #         ("/cmd_vel","/x500_drone_0/cmd_vel"), 
                #     ]
                # ),
                #Drone 1
 
    ])
