# !/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    px4_dir = os.path.join(os.getenv('HOME'), 'PX4-Autopilot')
    urdf_file = '/home/himanshu/1_cslam_single_drone/src/CSLAM-UAV/drone_slam_pkg/urdf/base_link.urdf'

    vslam_params = {
        'use_sim_time': True,

        # Frames
        'frame_id': 'base_link',
        'map_frame_id': 'map',

        # We use a dedicated odom frame to avoid TF conflict with your /odom_drone_tf node
        'odom_frame_id': 'odom',

        # Sensor subscriptions
        'subscribe_rgbd': True,
        'subscribe_depth': False,      # IMPORTANT: because we will feed RGBD from rgbd_sync
        'subscribe_imu': True,

        # Sync/queues
        'approx_sync': False,          # IMPORTANT: sync is done by rgbd_sync
        'queue_size': 200,
        'sync_queue_size': 100,

        # IMU init
        'wait_imu_to_init': True,

        # TF publishing
        'publish_tf': True,
    }

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'make', '-C', px4_dir, 'px4_sitl', 'gz_x500_depth_testworld'],
            output='screen',
            shell=True
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', ' ./QGroundControl-x86_64.AppImage'],
            cwd=os.path.expanduser('~/Downloads'),
            output='screen',
            shell=True
        ),
        ExecuteProcess(
             cmd=['MicroXRCEAgent','udp4', '--port', '8888']
        ),
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='gz_bridge_rgbd_imu',
                    output='screen',
                    parameters=[{'use_sim_time': True}],
                    arguments=[
                        # Simulation clock
                        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

                        # RGB camera
                        "/world/testworld/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image",

                        # Camera info
                        "/world/testworld/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",

                        # Depth camera raw image
                        "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",

                        # Depth camera as PointCloud2
                        "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",

                        # IMU
                        "/world/testworld/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    ]
                ),
                
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': open(urdf_file).read()},
                                {'use_sim_time': True}
                                ]
                ),
                # Node(
                #         package='rtabmap_odom',
                #         executable='rgbd_odometry',
                #         output='screen',
                #         parameters=[{
                #             **common_parameters, 
                #             'frame_id': 'base_link',
                #             'odom_frame_id': 'odom',
                #             'publish_tf': True
                #         }],
                #         remappings=[
                #             ('rgb/image', '/world/testworld/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                #             ('rgb/camera_info', '/world/testworld/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                #             ('depth/image', '/depth_camera'),
                #             ('imu', '/world/testworld/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
                #         ]
                #     ),


                # Node(
                #     package='drone_slam_pkg',
                #     executable='odom_drone_tf',
                #     name='odom_drone_tf',
                #     output='screen',
                #     parameters=[{'use_sim_time': True}]
                # ),
                Node(
                    package='rtabmap_sync',
                    executable='rgbd_sync',
                    name='rgbd_sync',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'approx_sync': True,                # sync RGB+Depth here
                        'approx_sync_max_interval': 0.04,
                        'queue_size': 200,
                        'sync_queue_size': 100,
                    }],
                    remappings=[
                        ('rgb/image', '/world/testworld/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                        ('rgb/camera_info', '/world/testworld/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
                        ('depth/image', '/depth_camera'),
                    ],
                ),

                # 2) Visual odometry (publishes TF rtabmap_odom -> base_link and topic /rtabmap/odom)
                Node(
                    package='rtabmap_odom',
                    executable='rgbd_odometry',
                    name='rgbd_odometry',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=[
                        ('imu', '/world/testworld/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
                        # rgbd_image is already /rtabmap/rgbd_image from the rgbd_sync node
                    ],
                ),

                # 3) SLAM (publishes map -> rtabmap_odom TF)
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    name='rtabmap',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=[
                        ('imu', '/world/testworld/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
                        ('odom', '/rtabmap/odom'),
                    ],
                    arguments=['-d'],   # delete previous ~/.ros/rtabmap.db (same behavior as many examples)
                ),

                # 4) Visualization
                Node(
                    package='rtabmap_viz',
                    executable='rtabmap_viz',
                    name='rtabmap_viz',
                    namespace='rtabmap',
                    output='screen',
                    parameters=[vslam_params],
                    remappings=[
                        ('imu', '/world/default/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu'),
                        ('odom', '/rtabmap/odom'),
                    ],
                ),

                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=['-d', os.path.join(
                        get_package_share_directory('rtabmap_rviz_plugins'),
                        'launch', 'rtabmap.rviz'
                    )],
                    parameters=[{'use_sim_time': True}]
                ),
            ]
            )])