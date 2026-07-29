import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'earendil_autonomy'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berkay',
    maintainer_email='berkaypaksoy07@gmail.com',
    description='Autonomous navigation and sensor fusion stack for the Earendil Bot rover in ROS 2 Humble/Jazzy.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'heading_test = earendil_autonomy.tests.heading_test:main',
            'gps_nav_test = earendil_autonomy.tests.gps_nav_test:main',
            'base_exit = earendil_autonomy.tests.base_exit:main',
            'base_enter = earendil_autonomy.tests.base_enter:main',
            'hardware_check = earendil_autonomy.hardware_check:main',
            'roverRTK = earendil_autonomy.gps.roverRTK:main',
            'gps_navigator_node = earendil_autonomy.gps.gps_navigator_node:main',
            'tunnel_test1 = earendil_autonomy.tests.tunnel_test1:main',
            'tunnel_test2 = earendil_autonomy.tests.tunnel_test2:main',
            'tunnel_test3 = earendil_autonomy.tests.tunnel_test3:main',
            'tunnel_test4 = earendil_autonomy.tests.tunnel_test4:main',
            'tunnel_test5 = earendil_autonomy.tests.tunnel_test5:main',
            'aruco_receiver = earendil_autonomy.tests.aruco_receiver:main',
            'rock_receiver = earendil_autonomy.tests.rock_receiver:main',
            'peak_finder = earendil_autonomy.tests.peak_finder:main',
            'rscp_bridge_node = earendil_autonomy.rscp.rscp_bridge_node:main',
            'mission_manager_node = earendil_autonomy.rscp.mission_manager_node:main',
            'h7_serial_node = earendil_autonomy.h7_bridge.h7_serial_node:main',
            'h7_command_node = earendil_autonomy.h7_bridge.h7_command_node:main',
            'heading_node = earendil_autonomy.h7_bridge.heading_node:main',
            'imu_node = earendil_autonomy.h7_bridge.imu_node:main',
            'magnetometer_node = earendil_autonomy.h7_bridge.magnetometer_node:main',
            'magnetometer_calibrator = earendil_autonomy.h7_bridge.magnetometer_calibrator:main',
            'wheel_telemetry_node = earendil_autonomy.h7_bridge.wheel_telemetry_node:main',
            'h7_kinematics_node = earendil_autonomy.h7_bridge.h7_kinematics_node:main',
        ],
    },
)