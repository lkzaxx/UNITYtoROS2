from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'unity_openarm_bridge'

setup(
    name=package_name,
    version='1.0.0',
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
    maintainer='OpenArm Team',
    maintainer_email='openarm@enactic.ai',
    description='Unity-OpenArm ROS2 Bridge using TCP Endpoint',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tcp_bridge_node = unity_openarm_bridge.tcp_bridge_node:main',
            'openarm_controller = unity_openarm_bridge.openarm_controller:main',
        ],
    },
)
