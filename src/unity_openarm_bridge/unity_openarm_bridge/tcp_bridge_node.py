#!/usr/bin/env python3

"""
Unity-OpenArm TCP Bridge Node

This node provides the main TCP bridge functionality for Unity-ROS2 communication
using the ROS-TCP-Endpoint protocol. It handles:
- Basic heartbeat and ping/pong services
- Unity pose commands
- OpenArm joint state publishing
- TCP connection management

Author: OpenArm Team
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from example_interfaces.srv import Trigger
from std_srvs.srv import Empty

import json
import time
from typing import Dict, List, Optional


class UnityOpenArmBridge(Node):
    """
    Main bridge node for Unity-OpenArm communication via TCP.
    
    This node acts as the central hub for all Unity-ROS2 communication,
    handling both basic bridge functions and OpenArm-specific operations.
    """
    
    def __init__(self):
        super().__init__('unity_openarm_bridge')
        
        # Declare parameters
        self.declare_parameter('heartbeat_rate', 1.0)
        self.declare_parameter('heartbeat_text', 'ros2_alive')
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 10000)
        
        # Get parameters
        self.heartbeat_rate = self.get_parameter('heartbeat_rate').get_parameter_value().double_value
        self.heartbeat_text = self.get_parameter('heartbeat_text').get_parameter_value().string_value
        self.tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
        self.tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        
        # QoS Profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize publishers
        self.setup_publishers()
        
        # Initialize subscribers
        self.setup_subscribers()
        
        # Initialize services
        self.setup_services()
        
        # Initialize timers
        self.setup_timers()
        
        # OpenArm state
        self.current_joint_states = JointState()
        self.target_pose = PoseStamped()
        
        self.get_logger().info(f'Unity-OpenArm Bridge initialized')
        self.get_logger().info(f'TCP Server should be running on {self.tcp_ip}:{self.tcp_port}')
        self.get_logger().info(f'Heartbeat rate: {self.heartbeat_rate} Hz')
    
    def setup_publishers(self):
        """Initialize all publishers"""
        # Unity communication
        self.heartbeat_pub = self.create_publisher(
            String, 
            '/unity/heartbeat', 
            self.reliable_qos
        )
        
        # OpenArm state publishing
        self.joint_states_pub = self.create_publisher(
            JointState,
            '/openarm/joint_states',
            self.sensor_qos
        )
        
        self.end_effector_pose_pub = self.create_publisher(
            PoseStamped,
            '/openarm/end_effector_pose',
            self.sensor_qos
        )
        
        # OpenArm official controller command publisher
        # Forward Unity joint commands to OpenArm forward_position_controller
        self.forward_position_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            self.reliable_qos
        )
    
    def setup_subscribers(self):
        """Initialize all subscribers"""
        # Unity commands
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/unity/pose',
            self.unity_pose_callback,
            self.reliable_qos
        )
        
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/unity/joint_commands',
            self.unity_joint_cmd_callback,
            self.reliable_qos
        )
        
        # OpenArm feedback (if available)
        self.openarm_state_sub = self.create_subscription(
            JointState,
            '/openarm/hardware/joint_states',
            self.openarm_state_callback,
            self.sensor_qos
        )
    
    def setup_services(self):
        """Initialize all services"""
        # Basic bridge services
        self.ping_service = self.create_service(
            Trigger,
            '/unity/ping',
            self.ping_callback
        )
        
        # OpenArm control services
        self.home_service = self.create_service(
            Empty,
            '/openarm/home_position',
            self.home_position_callback
        )
    
    def setup_timers(self):
        """Initialize all timers"""
        # Heartbeat timer
        self.heartbeat_timer = self.create_timer(
            1.0 / self.heartbeat_rate,
            self.heartbeat_callback
        )
        
        # Joint state publishing timer (for simulation)
        self.joint_pub_timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_joint_states
        )
    
    def heartbeat_callback(self):
        """Publish heartbeat message to Unity"""
        msg = String()
        msg.data = self.heartbeat_text
        self.heartbeat_pub.publish(msg)
    
    def ping_callback(self, request, response):
        """Handle ping service requests from Unity"""
        response.success = True
        response.message = "pong"
        self.get_logger().debug("Ping received, responding with pong")
        return response
    
    def unity_pose_callback(self, msg: PoseStamped):
        """Handle pose commands from Unity"""
        self.target_pose = msg
        self.get_logger().info(
            f"Received Unity pose: "
            f"pos=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}) "
            f"frame={msg.header.frame_id}"
        )
        
        # TODO: Forward to OpenArm IK solver or motion planner
        # For now, just log the received pose
    
    def unity_joint_cmd_callback(self, msg: JointState):
        """Handle joint commands from Unity"""
        self.get_logger().info(
            f"Received Unity joint command: {len(msg.position)} joints"
        )
        
        if len(msg.position) > 0:
            joint_str = ", ".join([f"{pos:.3f}" for pos in msg.position[:7]])  # Show first 7 joints
            self.get_logger().info(f"Joint positions: [{joint_str}]")
        
        # Forward to OpenArm official controller (forward_position_controller)
        if len(msg.position) >= 7:  # OpenArm has 7 DOF
            # Convert JointState to Float64MultiArray for forward_position_controller
            forward_cmd = Float64MultiArray()
            forward_cmd.data = [float(pos) for pos in msg.position[:7]]  # Extract first 7 joints
            
            # Publish to OpenArm official controller
            self.forward_position_pub.publish(forward_cmd)
            self.get_logger().debug(
                f"Forwarded {len(forward_cmd.data)} joint commands to OpenArm forward_position_controller"
            )
            
            # Also simulate for testing/visualization (optional)
            self.simulate_joint_movement(msg)
    
    def openarm_state_callback(self, msg: JointState):
        """Handle real OpenArm hardware state feedback"""
        self.current_joint_states = msg
        self.get_logger().debug("Received OpenArm hardware state")
    
    def simulate_joint_movement(self, target_joints: JointState):
        """Simulate joint movement for testing without hardware"""
        # Simple simulation: gradually move towards target
        if not self.current_joint_states.position:
            # Initialize with target if no current state
            self.current_joint_states = target_joints
            self.current_joint_states.header.stamp = self.get_clock().now().to_msg()
            return
        
        # Gradual movement simulation
        alpha = 0.1  # Movement speed factor
        new_positions = []
        
        for i, target_pos in enumerate(target_joints.position):
            if i < len(self.current_joint_states.position):
                current_pos = self.current_joint_states.position[i]
                new_pos = current_pos + alpha * (target_pos - current_pos)
                new_positions.append(new_pos)
            else:
                new_positions.append(target_pos)
        
        self.current_joint_states.position = new_positions
        self.current_joint_states.header.stamp = self.get_clock().now().to_msg()
    
    def publish_joint_states(self):
        """Publish current joint states to Unity"""
        if self.current_joint_states.position:
            # Update timestamp
            self.current_joint_states.header.stamp = self.get_clock().now().to_msg()
            self.current_joint_states.header.frame_id = "openarm_base_link"
            
            # Ensure joint names are set
            if not self.current_joint_states.name:
                self.current_joint_states.name = [
                    f"joint_{i+1}" for i in range(len(self.current_joint_states.position))
                ]
            
            self.joint_states_pub.publish(self.current_joint_states)
    
    def home_position_callback(self, request, response):
        """Move OpenArm to home position"""
        self.get_logger().info("Moving OpenArm to home position")
        
        # Define home position (all joints at 0)
        home_joints = JointState()
        home_joints.header.stamp = self.get_clock().now().to_msg()
        home_joints.header.frame_id = "openarm_base_link"
        home_joints.name = [f"joint_{i+1}" for i in range(7)]
        home_joints.position = [0.0] * 7
        home_joints.velocity = [0.0] * 7
        home_joints.effort = [0.0] * 7
        
        # Simulate movement to home
        self.simulate_joint_movement(home_joints)
        
        return response


def main(args=None):
    """Main entry point for the TCP bridge node"""
    rclpy.init(args=args)
    
    try:
        node = UnityOpenArmBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in TCP bridge node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
