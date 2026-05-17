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
            'arduino_joy = earendil_bot.arduino_joy:main',
            'joy_teleop = earendil_bot.joy_teleop:main',
            'simple_motor_bridge = earendil_bot.simple_motor_bridge:main',
            'pure_gps_nav = earendil_bot.pure_gps_nav:main',
            'gy91_imu_node = earendil_bot.gy91_imu_node:main',
            'imu_heading_test = earendil_bot.imu_heading_test:main',
        ],
    },
)
