from setuptools import find_packages, setup

package_name = 'unity_bridge_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Unity Bridge Package for ROS2-Unity communication',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'status_publisher = unity_bridge_py.status_publisher:main',
            'cmd_subscriber = unity_bridge_py.cmd_subscriber:main',
            'chatter_subscriber = unity_bridge_py.chatter_subscriber:main',
            'chatter_publisher = unity_bridge_py.chatter_publisher:main',
            'connection_monitor = unity_bridge_py.connection_monitor:main',
            'cmd_vel_subscriber = unity_bridge_py.cmd_vel_subscriber:main',
            'simple_unity_bridge = unity_bridge_py.simple_unity_bridge:main',
        ],
    },
)
