#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    px4_dir = os.path.join(os.getenv("HOME"), "PX4-Autopilot")

    def get_vslam_params(drone_ns, db_name):
        return {
            'use_sim_time': True,
            'frame_id': f'{drone_ns}/base_link',
            'guess_frame_id':f'{drone_ns}/base_link_stabilized',
            'map_frame_id': f'{drone_ns}/map',
            'odom_frame_id': f'{drone_ns}/odom',
            
            'subscribe_rgbd': True,
            # 'subscribe_depth': False,
            'subscribe_odom_info':True,
            'wait_for_transform': 0.5,
            # 'tf_delay': 0.1,
            # 'tf_tolerance': 0.5,
            # 'subscribe_imu': True,
            'approx_sync': False, 
            'wait_imu_to_init': True,
            # 'publish_tf': True,
            # 'queue_size': 200,
            # 'sync_queue_size': 100,
            
            # 'Odom/ResetCountdown': '5',     
            # 'Vis/MinInliers': '10',         
            # 'Odom/Strategy': '0',           
            # 'wait_for_transform': 0.5,
            # 'Optimizer/GravitySigma': '0.3',

            'Optimizer/GravitySigma': '0.1',
            'Vis/FeatureType': '10',
            'Kp/DetectorStrategy': '10',
            'Grid/MapFrameProjection': 'true',
            'NormalsSegmentation': 'false',
            # 'Grid/MinGroundHeight': '-0.2',
            'Grid/MaxGroundHeight': '1.15' ,
            'Grid/MaxObstacleHeight': '1.75',
            'RGBD/StartAtOrigin': 'true',
            # 'Grid/GroundIsObstacle': 'false',
            # 'Grid/RayTracing': True,

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
            cmd=["gnome-terminal", "--", "make", "-C", px4_dir, "px4_sitl", "gz_x500_depth"],
            output="screen",
            shell=True
        ),

        ExecuteProcess(
            cmd=["gnome-terminal", "--", "./QGroundControl-x86_64.AppImage"],
            cwd=os.path.expanduser("~"),
            output="screen",
            shell=True
        ),

        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "--port", "8888"],
            output="screen",
        ),

        TimerAction(
            period=15.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "gnome-terminal", "--", "bash", "-c",
                        "cd " + px4_dir +
                        " && PX4_SYS_AUTOSTART=4001 "
                        'PX4_GZ_MODEL_POSE="2,0,0" '
                        'PX4_GZ_MODEL_ORIENTATION="0,0,1.5708" '
                        "PX4_SIM_MODEL=gz_x500_depth "
                        "./build/px4_sitl_default/bin/px4 -i 1; exec bash"
                    ],
                    output="screen",
                )
            ],
        ),

        TimerAction(
            period=20.0,
            actions=[


                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="gz_bridge_rgbd_imu_2drones",
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                    arguments=[
                        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                        
                        # Drone 0
                        "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        "/world/default/model/x500_depth_0/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",

                        # Drone 1
                        "/world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        "/world/default/model/x500_depth_1/link/camera_link/sensor/StereoOV7251/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/world/default/model/x500_depth_1/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    ],
                    remappings=[
                        # Drone 0
                        ("/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image", "/x500_drone_0/rgb/image"),
                        ("/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info", "/x500_drone_0/rgb/camera_info"),
                        ("/world/default/model/x500_depth_0/link/camera_link/sensor/StereoOV7251/depth_image", "/x500_drone_0/depth/image"),
                        ("/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu", "/x500_drone_0/imu/data_raw"),

                        # Drone 1
                        ("/world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/image", "/x500_drone_1/rgb/image"),
                        ("/world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/camera_info", "/x500_drone_1/rgb/camera_info"),
                        ("/world/default/model/x500_depth_1/link/camera_link/sensor/StereoOV7251/depth_image", "/x500_drone_1/depth/image"),
                        ("/world/default/model/x500_depth_1/link/base_link/sensor/imu_sensor/imu", "/x500_drone_1/imu/data_raw"),
                    ],
                ),

                #Drone 0
                Node(package='tf2_ros', executable='static_transform_publisher',
                     arguments=['0', '0', '0', '0', '0', '0', 'x500_drone_0/base_link', 'x500_depth_0/base_link/imu_sensor']),
             
                Node(package='tf2_ros', executable='static_transform_publisher',
                     arguments=['0.12', '0.03', '0.242', '-1.570796327', '0', '-1.570796327', 'x500_drone_0/base_link', 'x500_drone_0/camera_link']),
                
                Node(package='tf2_ros', executable='static_transform_publisher',
                     arguments=['0.0123', '-0.03', '0.01878', '0', '0', '0', 'x500_drone_0/camera_link', 'x500_depth_0/camera_link/IMX214']),
                
                Node(package='tf2_ros', executable='static_transform_publisher',
                     arguments=['0.01233', '-0.03', '0.01878', '0', '0', '0', 'x500_drone_0/camera_link', 'x500_drone_0/camera_link/StereoOV7251']),

                
                # Drone 1
                Node(package='tf2_ros', executable='static_transform_publisher',
                     arguments=['0', '0', '0', '0', '0', '0', 'x500_drone_1/base_link', 'x500_depth_1/base_link/imu_sensor']),
                
                Node(package='tf2_ros', executable='static_transform_publisher',
                     arguments=['0.12', '0.03', '0.242', '-1.570796327', '0', '-1.570796327', 'x500_drone_1/base_link', 'x500_drone_1/camera_link']),
                
                Node(package='tf2_ros', executable='static_transform_publisher',
                     arguments=['0.0123', '-0.03', '0.01878', '0', '0', '0', 'x500_drone_1/camera_link', 'x500_depth_1/camera_link/IMX214']),
                Node(package='tf2_ros', executable='static_transform_publisher',
                     arguments=['0.01233', '-0.03', '0.01878', '0', '0', '0', 'x500_drone_1/camera_link', 'x500_drone_1/camera_link/StereoOV7251']),

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
                        'use_sim_time': True,
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
                        'use_sim_time': True,
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
                        "use_sim_time": True,
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
                Node(
                    package="rtabmap_viz",
                    executable="rtabmap_viz",
                    name="rtabmap_viz_0",
                    namespace="x500_drone_0",
                    output="screen",
                    parameters=[get_vslam_params("x500_drone_0", "rtabmap_drone_0")],
                    remappings=[
                        ("imu", "/x500_drone_0/imu/data"),
                        ("odom", "/x500_drone_0/odom"),
                        ("map", "/x500_drone_0/map"),
                    ],
                ),
                Node(
                    package='rtabmap_util', executable='point_cloud_xyz', output='screen',
                    name='pointcloud_xyz_0',
                    namespace= 'x500_drone_0',
                    parameters=[{'decimation': 2,
                                'max_depth': 3.0,
                                'voxel_size': 0.02,
                                'use_sim_time': True}],
                    remappings=[('depth/image', '/x500_drone_0/depth/image'),
                                ('depth/camera_info', '/x500_drone_0/rgb/camera_info'),
                                ('cloud', '/x500_drone_0/camera/cloud')]
                ),
                Node(
                    package='rtabmap_costmap_plugins', executable='voxel_marker', output='screen',
                    name='voxel_marker_0',
                    namespace="/x500_drone_0",
                    parameters=[{'use_sim_time': True}]
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
                ),
                Node(
                    package='drone_slam_pkg',
                    executable='exploration_planner',
                    name='exploration_planner_0',
                    namespace='x500_drone_0',
                    output='screen',
                    remappings = [
                        ("map","/x500_drone_0/map"),
                        ("fmu/out/vehicle_local_position","/fmu/out/vehicle_local_position"),
                        ("cmd_vel", "/x500_drone_0/cmd_vel"),
                    ]
                ),
                Node(
                    package='px4_ros_com',
                    executable='offboard_cmd_vel',
                    name='drone_0_control',
                    namespace='x500_drone_0',
                    output='screen',
                    parameters = [{
                        'use_sim_time': True,
                    }],
                    remappings=[
                        ("/cmd_vel","/x500_drone_0/cmd_vel"), 
                    ]
                ),
                # Node(
                #     package='drone_control_pkg',
                #     executable='velocity_control',
                #     name='teleop',
                #     output='screen',
                # ),
                #Drone 1
                Node(
                    package='imu_filter_madgwick',
                    executable='imu_filter_madgwick_node',
                    name='imu_filter_1',
                    namespace='x500_drone_1',
                    output='screen',
                    parameters=[{
                        'use_mag': False,
                        'world_frame': 'enu',
                        'publish_tf': False,
                        'use_sim_time': True,
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
                    namespace='x500_drone_1',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'fixed_frame_id': 'x500_drone_1/base_link_stabilized',
                        'base_frame_id': 'x500_drone_1/base_link',
                    }],
                    remappings=[
                        ('imu', '/x500_drone_1/imu/data'),
                    ]
                ),
                Node(
                    package="rtabmap_sync",
                    executable="rgbd_sync",
                    name="rgbd_sync_x500_drone_1",
                    namespace="x500_drone_1",
                    output="screen",
                    parameters=[{
                        "use_sim_time": True,
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.04,
                        "queue_size": 200,
                        "sync_queue_size": 100,
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
                    namespace="x500_drone_1",
                    output="screen",
                    parameters=[get_vslam_params("x500_drone_1", "rtabmap_drone_1")],
                    remappings=[
                        ("imu", "/x500_drone_1/imu/data"),
                        ("map", "/x500_drone_1/map"),

                    ],
                ),
                Node(
                    package="rtabmap_slam",
                    executable="rtabmap",
                    name="rtabmap",
                    namespace="x500_drone_1",
                    output="screen",
                    parameters=[get_vslam_params("x500_drone_1", "rtabmap_drone_1")],
                    remappings=[
                        ("imu", "/x500_drone_1/imu/data"),
                        ("odom", "/x500_drone_1/odom"),
                        ("map", "/x500_drone_1/map"),
                    ],
                    arguments=["-d"], 
                ),
                Node(
                    package="rtabmap_viz",
                    executable="rtabmap_viz",
                    name="rtabmap_viz_1",
                    namespace="x500_drone_1",
                    output="screen",
                    parameters=[get_vslam_params("x500_drone_1", "rtabmap_drone_1")],
                    remappings=[
                        ("imu", "/x500_drone_1/imu/data"),
                        ("odom", "/x500_drone_1/odom"),
                        ("map", "/x500_drone_1/map")
                    ],
                ),

                Node(
                    package='rtabmap_util', executable='point_cloud_xyz', output='screen',
                    name='pointcloud_xyz_1',
                    namespace= 'x500_drone_1',
                    parameters=[{'decimation': 2,
                                'max_depth': 3.0,
                                'voxel_size': 0.02,
                                'use_sim_time': True}],
                    remappings=[('depth/image', '/x500_drone_1/depth/image'),
                                ('depth/camera_info', '/x500_drone_1/rgb/camera_info'),
                                ('cloud', '/x500_drone_1/camera/cloud')]
                ),
                Node(
                    package='rtabmap_costmap_plugins', executable='voxel_marker', output='screen',
                    name='voxel_marker_1',
                    namespace="/x500_drone_1",
                    parameters=[{'use_sim_time': True}]
                ),

                Node(
                    package='px4_ros_com',         
                    executable='ros_odometry_to_vehicle_odometry',  
                    name='ros_odometry_to_vehicle_odometry_1',
                    parameters=[ 
                        {"odom_topic": "/x500_drone_1/odom"},
                        {"vehicle_odometry_topic": "px4_1/fmu/in/vehicle_visual_odometry"},
                        {"map_frame_id": "/x500_drone_1/map"},   
                        {"repeat_odom": True}      
                    ],
                    output='screen'
                ),
                # Node(
                #     package='drone_slam_pkg',
                #     executable='exploration_planner',
                #     name='exploration_planner_1',
                #     output='screen',
                #     remappings = [
                #         ("map","/x500_drone_1/map"),
                #         ("fmu/out/vehicle_local_position","px4_1/fmu/out/vehicle_local_position"),
                #     ]

                # ),
            ],
        ),
    ])