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
            'heading_test = earendil_bot.tests.heading_test:main',
            'hardware_bridge = earendil_bot.bridge.hardware_bridge:main',
            'gps_nav_test = earendil_bot.tests.gps_nav_test:main',
            'base_exit = earendil_bot.tests.base_exit:main',
            'base_enter = earendil_bot.tests.base_enter:main',
            'hardware_check = earendil_bot.hardware_check:main',
            'roverRTK = earendil_bot.gps.roverRTK:main',
            'tunnel_test1 = earendil_bot.tests.tunnel_test1:main',
            'tunnel_test2 = earendil_bot.tests.tunnel_test2:main',
            'tunnel_test3 = earendil_bot.tests.tunnel_test3:main',
            'tunnel_test4 = earendil_bot.tests.tunnel_test4:main',
            'tunnel_test5 = earendil_bot.tests.tunnel_test5:main',
            'aruco_receiver = earendil_bot.tests.aruco_receiver:main',
            'rock_receiver = earendil_bot.tests.rock_receiver:main',
        ],
    },
)