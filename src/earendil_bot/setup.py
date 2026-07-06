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
            'turn_test = earendil_bot.test.turn_test:main',
            'hardware_bridge = earendil_bot.bridge.hardware_bridge:main',
            'gps_nav_test = earendil_bot.test.gps_nav_test:main',
            'tunnel_navigator = earendil_bot.test.tunnel_navigator:main',
            'igloo_exit = earendil_bot.test.igloo_exit:main',
            'ir_bridge = earendil_bot.bridge.ir_bridge:main',
            'aruco_detector = earendil_bot.test.aruco_detector:main',
            'hardware_check = earendil_bot.hardware_check:main',
        ],
    },
)