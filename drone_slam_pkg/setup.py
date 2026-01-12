from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'drone_slam_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', ['launch/px4_gazebo.launch.py']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='himanshu',
        maintainer_email='078bme014.himanshu@pcampus.edu.np',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                        'odom_drone_tf = drone_slam_pkg.odom_drone_tf:main',
                        'offboard_control = drone_slam_pkg.offboard_control:main',
                        'exploration_planner = drone_slam_pkg.exploration_planner:main'
            ],
        },
    )
