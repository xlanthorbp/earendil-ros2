import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'earendil_bot'

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
    description='Autonomous navigation and sensor fusion stack for the Earendil Bot rover in ROS 2 Humble.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turn_test = earendil_bot.test.turn_test:main',
            'hardware_bridge = earendil_bot.bridge.hardware_bridge:main',
            'gps_nav_test = earendil_bot.test.gps_nav_test:main',
            'tunnel_navigator = earendil_bot.test.tunnel_navigator:main',
            'base_exit = earendil_bot.test.base_exit:main',
            'base_enter = earendil_bot.test.base_enter:main',
            'aruco_detector = earendil_bot.test.aruco_detector:main',
            'hardware_check = earendil_bot.hardware_check:main',
            'rtk_node = earendil_bot.gps.rtk_node:main',
            'lidar_motor_test = earendil_bot.test.lidar_motor_test:main',
            'test1 = earendil_bot.test.test1:main',
            'test2 = earendil_bot.test.test2:main',
            'test3 = earendil_bot.test.test3:main',
            'aruco_receiver = earendil_bot.test.aruco_receiver:main',
        ],
    },
)