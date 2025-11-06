#!/usr/bin/env python3

"""
Simulation Bridge Launch File

This launch file starts a minimal Unity-OpenArm bridge for simulation and testing:
- ROS-TCP-Endpoint server
- Unity-OpenArm bridge node (simulation mode)
- Basic joint state simulation

Author: OpenArm Team
License: Apache-2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for simulation testing"""
    
    # Declare launch arguments
    tcp_ip_arg = DeclareLaunchArgument(
        'tcp_ip',
        default_value='127.0.0.1',
        description='TCP server IP address'
    )
    
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='10000',
        description='TCP server port'
    )
    
    heartbeat_rate_arg = DeclareLaunchArgument(
        'heartbeat_rate',
        default_value='2.0',
        description='Heartbeat publishing rate (Hz)'
    )
    
    # Get launch configurations
    tcp_ip = LaunchConfiguration('tcp_ip')
    tcp_port = LaunchConfiguration('tcp_port')
    heartbeat_rate = LaunchConfiguration('heartbeat_rate')
    
    # ROS-TCP-Endpoint server node
    tcp_endpoint_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='tcp_endpoint_server',
        parameters=[{
            'ROS_IP': tcp_ip,
            'ROS_TCP_PORT': tcp_port,
        }],
        output='screen',
        emulate_tty=True,
        prefix='echo "Starting TCP Endpoint Server on {}:{}" &&'.format('${tcp_ip}', '${tcp_port}'),
    )
    
    # Unity-OpenArm bridge node (simulation mode)
    unity_bridge_node = Node(
        package='unity_openarm_bridge',
        executable='tcp_bridge_node',
        name='unity_openarm_bridge_sim',
        parameters=[{
            'tcp_ip': tcp_ip,
            'tcp_port': tcp_port,
            'heartbeat_rate': heartbeat_rate,
            'heartbeat_text': 'openarm_sim_alive',
        }],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # Launch arguments
        tcp_ip_arg,
        tcp_port_arg,
        heartbeat_rate_arg,
        
        # Nodes
        tcp_endpoint_node,
        unity_bridge_node,
    ])
