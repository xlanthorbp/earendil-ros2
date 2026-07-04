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
        (os.path.join('share', package_name, 'description'),
            glob('description/*.xacro') + glob('description/*.yaml')),

        (os.path.join('share', package_name, 'config'), [
            'config/twist_mux.yaml',
            'config/nav2_params.yaml',
            'config/ukf_local.yaml',
            'config/ukf_global.yaml',
            'config/navsat.yaml',
            'config/controllers.yaml',
            'config/test_params.yaml',
            'config/tunnel_params.yaml',
            'config/hardware_params.yaml',
        ]),

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
            'arduino_joy = earendil_bot.manualteleop.arduino_joy:main',
            'joy_teleop = earendil_bot.manualteleop.joy_teleop:main',
            'gps_nav_test = earendil_bot.test.gps_nav_test:main',
            'imu_turn_test = earendil_bot.test.imu_turn_test:main',
            'mag_turn_test = earendil_bot.test.mag_turn_test:main',
            'hardware_bridge = earendil_bot.bridge.hardware_bridge:main',
            'rtk_node = earendil_bot.gps.rtk_node:main',
            'gy91_imu_node = earendil_bot.imu.gy91_imu_node:main',
            'gps_waypoint_follower = earendil_bot.test.gps_waypoint_follower:main',
            'tunnel_navigator = earendil_bot.test.tunnel_navigator:main',
            'igloo_exit = earendil_bot.test.igloo_exit:main',
            'ir_bridge = earendil_bot.bridge.ir_bridge:main',
        ],
    },
)
