from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={
                "name": "oak",
                "params_file": os.path.expanduser("~/oak_run.yaml"),
            }.items(),
        ),

        Node(
            package='topic_tools',
            executable='relay',
            name='relay_rgb_image',
            arguments=['/oak/rgb/image_rect', '/x500_drone_0/rgb/image'],
            output='screen'
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_camera_info',
            arguments=['/oak/rgb/camera_info', '/x500_drone_0/rgb/camera_info'],
            output='screen'
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='relay_depth_image',
            arguments=['/oak/stereo/image_raw', '/x500_drone_0/depth/image'],
            output='screen'
        ),
    ])
