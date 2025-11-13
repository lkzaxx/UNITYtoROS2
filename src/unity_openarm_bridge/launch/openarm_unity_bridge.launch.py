#!/usr/bin/env python3

"""
OpenArm Unity Bridge Launch File

This launch file starts the complete Unity-OpenArm bridge system including:
- ROS-TCP-Endpoint server
- Unity-OpenArm bridge node
- OpenArm controller node
- OpenArm description (URDF)

Author: OpenArm Team
License: Apache-2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate the launch description for the complete OpenArm Unity bridge system"""
    
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
        default_value='1.0',
        description='Heartbeat publishing rate (Hz)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    load_robot_description_arg = DeclareLaunchArgument(
        'load_robot_description',
        default_value='true',
        description='Load OpenArm robot description'
    )
    
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Start RViz for visualization'
    )
    
    # OpenArm official controller arguments
    start_openarm_controller_arg = DeclareLaunchArgument(
        'start_openarm_controller',
        default_value='true',
        description='Start OpenArm official ros2_control controller'
    )
    
    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type',
        default_value='real',
        choices=['real', 'fake'],
        description='OpenArm hardware type: real or fake (for testing)'
    )
    
    arm_type_arg = DeclareLaunchArgument(
        'arm_type',
        default_value='v10',
        description='OpenArm arm type (e.g., v10)'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface for OpenArm hardware'
    )
    
    # Get launch configurations
    tcp_ip = LaunchConfiguration('tcp_ip')
    tcp_port = LaunchConfiguration('tcp_port')
    heartbeat_rate = LaunchConfiguration('heartbeat_rate')
    use_sim_time = LaunchConfiguration('use_sim_time')
    load_robot_description = LaunchConfiguration('load_robot_description')
    start_rviz = LaunchConfiguration('start_rviz')
    start_openarm_controller = LaunchConfiguration('start_openarm_controller')
    hardware_type = LaunchConfiguration('hardware_type')
    arm_type = LaunchConfiguration('arm_type')
    can_interface = LaunchConfiguration('can_interface')
    
    # ROS-TCP-Endpoint server node
    tcp_endpoint_node = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='tcp_endpoint_server',
        parameters=[{
            'ROS_IP': tcp_ip,
            'ROS_TCP_PORT': tcp_port,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        emulate_tty=True,
    )
    
    # Unity-OpenArm bridge node
    unity_bridge_node = Node(
        package='unity_openarm_bridge',
        executable='tcp_bridge_node',
        name='unity_openarm_bridge',
        parameters=[{
            'tcp_ip': tcp_ip,
            'tcp_port': tcp_port,
            'heartbeat_rate': heartbeat_rate,
            'heartbeat_text': 'openarm_ros2_alive',
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        emulate_tty=True,
    )
    
    # OpenArm controller node
    openarm_controller_node = Node(
        package='unity_openarm_bridge',
        executable='openarm_controller',
        name='openarm_controller',
        parameters=[{
            'control_rate': 100.0,
            'max_joint_velocity': 1.0,
            'max_joint_acceleration': 2.0,
            'workspace_limits': [0.2, 0.8, -0.4, 0.4, 0.1, 0.6],
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        emulate_tty=True,
    )
    
    # Robot description (conditional)
    robot_description_nodes = []
    
    # Try to include OpenArm description if available
    try:
        openarm_description_share = get_package_share_directory('openarm_description')
        
        # Robot state publisher
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': PathJoinSubstitution([
                    FindPackageShare('openarm_description'),
                    'urdf',
                    'openarm.urdf.xacro'
                ]),
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(load_robot_description),
            output='screen',
        )
        
        # Joint state publisher (for manual control)
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(load_robot_description),
            output='screen',
        )
        
        robot_description_nodes = [
            robot_state_publisher_node,
            joint_state_publisher_node,
        ]
        
    except Exception as e:
        print(f"OpenArm description not found: {e}")
        print("Continuing without robot description...")
    
    # OpenArm official bringup (conditional)
    # Include OpenArm official launch file with ros2_control
    openarm_bringup_launch = None
    try:
        openarm_bringup_share = get_package_share_directory('openarm_bringup')
        # Convert hardware_type to use_fake_hardware parameter
        use_fake_hardware_value = PythonExpression([
            "'false' if '", hardware_type, "' == 'real' else 'true'"
        ])
        openarm_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(openarm_bringup_share, 'launch', 'openarm.launch.py')
            ]),
            launch_arguments={
                'arm_type': arm_type,
                'use_fake_hardware': use_fake_hardware_value,
                'robot_controller': 'forward_position_controller',
                'can_interface': can_interface,
            }.items(),
            condition=IfCondition(start_openarm_controller),
        )
    except Exception as e:
        print(f"OpenArm bringup not found: {e}")
        print("Continuing without OpenArm official controller...")
        print("Note: Unity commands will be forwarded but OpenArm hardware may not respond.")
    
    # RViz node (conditional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('unity_openarm_bridge'),
        'config',
        'openarm_rviz.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(start_rviz),
        output='screen',
    )
    
    # Create launch description
    launch_description = LaunchDescription([
        # Launch arguments
        tcp_ip_arg,
        tcp_port_arg,
        heartbeat_rate_arg,
        use_sim_time_arg,
        load_robot_description_arg,
        start_rviz_arg,
        start_openarm_controller_arg,
        hardware_type_arg,
        arm_type_arg,
        can_interface_arg,
        
        # Core nodes
        tcp_endpoint_node,
        unity_bridge_node,
        openarm_controller_node,
        
        # Conditional nodes
        rviz_node,
    ])
    
    # Add robot description nodes if available
    for node in robot_description_nodes:
        launch_description.add_action(node)
    
    # Add OpenArm official bringup if available
    if openarm_bringup_launch is not None:
        launch_description.add_action(openarm_bringup_launch)
    
    return launch_description
