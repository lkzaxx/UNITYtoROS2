#!/usr/bin/env python3

"""
OpenArm Controller Node

This node handles OpenArm-specific control logic including:
- Forward/Inverse kinematics
- Motion planning integration
- Hardware interface management
- Safety monitoring

Author: OpenArm Team
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from example_interfaces.srv import Trigger

import numpy as np
import math
from typing import List, Optional, Tuple


class OpenArmController(Node):
    """
    OpenArm-specific controller for advanced manipulation tasks.
    
    This node provides higher-level control functions for the OpenArm,
    including kinematics, motion planning, and safety features.
    """
    
    def __init__(self):
        super().__init__('openarm_controller')
        
        # Declare parameters
        self.declare_parameter('control_rate', 100.0)  # Control loop frequency
        self.declare_parameter('max_joint_velocity', 1.0)  # rad/s
        self.declare_parameter('max_joint_acceleration', 2.0)  # rad/s²
        self.declare_parameter('workspace_limits', [0.2, 0.8, -0.4, 0.4, 0.1, 0.6])  # [x_min, x_max, y_min, y_max, z_min, z_max]
        
        # Get parameters
        self.control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.max_joint_vel = self.get_parameter('max_joint_velocity').get_parameter_value().double_value
        self.max_joint_acc = self.get_parameter('max_joint_acceleration').get_parameter_value().double_value
        self.workspace_limits = self.get_parameter('workspace_limits').get_parameter_value().double_array_value
        
        # QoS Profiles
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # OpenArm kinematic parameters (simplified 7-DOF model)
        self.joint_limits = [
            (-math.pi, math.pi),      # Joint 1: Base rotation
            (-math.pi/2, math.pi/2),  # Joint 2: Shoulder pitch
            (-math.pi, math.pi),      # Joint 3: Shoulder roll
            (-math.pi, 0),            # Joint 4: Elbow pitch
            (-math.pi, math.pi),      # Joint 5: Wrist pitch
            (-math.pi/2, math.pi/2),  # Joint 6: Wrist roll
            (-math.pi, math.pi),      # Joint 7: Wrist yaw
        ]
        
        # DH parameters (simplified - replace with actual OpenArm parameters)
        self.dh_params = [
            [0.0, 0.0, 0.333, 0.0],      # Joint 1
            [0.0, -math.pi/2, 0.0, 0.0], # Joint 2
            [0.0, math.pi/2, 0.316, 0.0], # Joint 3
            [0.0825, math.pi/2, 0.0, 0.0], # Joint 4
            [-0.0825, -math.pi/2, 0.384, 0.0], # Joint 5
            [0.0, math.pi/2, 0.0, 0.0],   # Joint 6
            [0.088, math.pi/2, 0.107, 0.0], # Joint 7
        ]
        
        # State variables
        self.current_joints = [0.0] * 7
        self.target_joints = [0.0] * 7
        self.joint_velocities = [0.0] * 7
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        
        # Control flags
        self.control_enabled = True
        self.emergency_stop = False
        
        # Initialize publishers
        self.setup_publishers()
        
        # Initialize subscribers
        self.setup_subscribers()
        
        # Initialize services
        self.setup_services()
        
        # Initialize timers
        self.setup_timers()
        
        self.get_logger().info('OpenArm Controller initialized')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
    
    def setup_publishers(self):
        """Initialize all publishers"""
        # Joint commands to hardware/simulation
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/openarm/joint_commands',
            self.control_qos
        )
        
        # Cartesian pose feedback
        self.pose_feedback_pub = self.create_publisher(
            PoseStamped,
            '/openarm/current_pose',
            self.control_qos
        )
        
        # Status and diagnostics
        self.status_pub = self.create_publisher(
            String,
            '/openarm/status',
            self.control_qos
        )
    
    def setup_subscribers(self):
        """Initialize all subscribers"""
        # Joint state feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/openarm/joint_states',
            self.joint_state_callback,
            self.control_qos
        )
        
        # Cartesian pose commands
        self.pose_cmd_sub = self.create_subscription(
            PoseStamped,
            '/openarm/target_pose',
            self.pose_command_callback,
            self.control_qos
        )
        
        # Cartesian velocity commands
        self.twist_cmd_sub = self.create_subscription(
            Twist,
            '/openarm/cmd_vel',
            self.twist_command_callback,
            self.control_qos
        )
        
        # Gripper commands
        self.gripper_cmd_sub = self.create_subscription(
            Float32,
            '/openarm/gripper_command',
            self.gripper_command_callback,
            self.control_qos
        )
    
    def setup_services(self):
        """Initialize all services"""
        # Control services
        self.enable_service = self.create_service(
            Trigger,
            '/openarm/enable_control',
            self.enable_control_callback
        )
        
        self.disable_service = self.create_service(
            Trigger,
            '/openarm/disable_control',
            self.disable_control_callback
        )
        
        self.emergency_stop_service = self.create_service(
            Trigger,
            '/openarm/emergency_stop',
            self.emergency_stop_callback
        )
        
        # Utility services
        self.get_fk_service = self.create_service(
            Trigger,  # Simplified - should use custom service type
            '/openarm/get_forward_kinematics',
            self.get_fk_callback
        )
    
    def setup_timers(self):
        """Initialize all timers"""
        # Main control loop
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )
        
        # Status publishing
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self.publish_status
        )
    
    def joint_state_callback(self, msg: JointState):
        """Handle joint state feedback"""
        if len(msg.position) >= 7:
            self.current_joints = list(msg.position[:7])
            if len(msg.velocity) >= 7:
                self.joint_velocities = list(msg.velocity[:7])
            
            # Update current pose using forward kinematics
            self.update_current_pose()
    
    def pose_command_callback(self, msg: PoseStamped):
        """Handle Cartesian pose commands"""
        if not self.control_enabled or self.emergency_stop:
            self.get_logger().warn("Control disabled, ignoring pose command")
            return
        
        if not self.is_pose_in_workspace(msg):
            self.get_logger().warn("Target pose outside workspace limits")
            return
        
        self.target_pose = msg
        
        # Solve inverse kinematics
        target_joints = self.inverse_kinematics(msg)
        if target_joints is not None:
            self.target_joints = target_joints
            self.get_logger().debug(f"IK solution found for target pose")
        else:
            self.get_logger().warn("No IK solution found for target pose")
    
    def twist_command_callback(self, msg: Twist):
        """Handle Cartesian velocity commands"""
        if not self.control_enabled or self.emergency_stop:
            return
        
        # Convert twist to joint velocities using Jacobian
        jacobian = self.compute_jacobian()
        if jacobian is not None:
            # Pseudo-inverse for velocity control
            twist_vector = np.array([
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z
            ])
            
            joint_vels = np.linalg.pinv(jacobian) @ twist_vector
            
            # Apply velocity limits
            joint_vels = np.clip(joint_vels, -self.max_joint_vel, self.max_joint_vel)
            
            # Update target joints based on velocity
            dt = 1.0 / self.control_rate
            for i in range(7):
                self.target_joints[i] = self.current_joints[i] + joint_vels[i] * dt
                # Apply joint limits
                self.target_joints[i] = max(self.joint_limits[i][0], 
                                          min(self.joint_limits[i][1], self.target_joints[i]))
    
    def gripper_command_callback(self, msg: Float32):
        """Handle gripper commands"""
        gripper_pos = max(0.0, min(1.0, msg.data))  # Clamp to [0, 1]
        self.get_logger().debug(f"Gripper command: {gripper_pos:.3f}")
        # TODO: Forward to gripper hardware interface
    
    def control_loop(self):
        """Main control loop"""
        if not self.control_enabled or self.emergency_stop:
            return
        
        # Simple joint-space PD control
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.header.frame_id = "openarm_base_link"
        joint_cmd.name = [f"joint_{i+1}" for i in range(7)]
        
        # PD gains
        kp = 10.0
        kd = 1.0
        
        positions = []
        velocities = []
        
        for i in range(7):
            # Position error
            pos_error = self.target_joints[i] - self.current_joints[i]
            
            # Velocity command (simple PD)
            vel_cmd = kp * pos_error - kd * self.joint_velocities[i]
            
            # Apply limits
            vel_cmd = max(-self.max_joint_vel, min(self.max_joint_vel, vel_cmd))
            
            positions.append(self.target_joints[i])
            velocities.append(vel_cmd)
        
        joint_cmd.position = positions
        joint_cmd.velocity = velocities
        
        self.joint_cmd_pub.publish(joint_cmd)
    
    def update_current_pose(self):
        """Update current end-effector pose using forward kinematics"""
        pose = self.forward_kinematics(self.current_joints)
        if pose is not None:
            self.current_pose = pose
            self.pose_feedback_pub.publish(pose)
    
    def forward_kinematics(self, joints: List[float]) -> Optional[PoseStamped]:
        """Compute forward kinematics (simplified implementation)"""
        # This is a simplified FK implementation
        # Replace with actual OpenArm kinematics
        
        if len(joints) < 7:
            return None
        
        # Simple approximation for demonstration
        # In reality, use proper DH transformation matrices
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "openarm_base_link"
        
        # Simplified calculation (replace with proper kinematics)
        reach = 0.8  # Approximate arm reach
        
        pose.pose.position.x = reach * math.cos(joints[0]) * math.cos(joints[1])
        pose.pose.position.y = reach * math.sin(joints[0]) * math.cos(joints[1])
        pose.pose.position.z = 0.3 + reach * math.sin(joints[1])
        
        # Simplified orientation (replace with proper rotation matrices)
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        
        return pose
    
    def inverse_kinematics(self, target_pose: PoseStamped) -> Optional[List[float]]:
        """Compute inverse kinematics (simplified implementation)"""
        # This is a placeholder for IK solver
        # Replace with proper IK implementation (e.g., using KDL, MoveIt, or custom solver)
        
        x = target_pose.pose.position.x
        y = target_pose.pose.position.y
        z = target_pose.pose.position.z
        
        # Simple 2-DOF approximation for base and shoulder
        joints = [0.0] * 7
        
        # Base rotation
        joints[0] = math.atan2(y, x)
        
        # Shoulder pitch (simplified)
        reach = math.sqrt(x*x + y*y + (z-0.3)**2)
        if reach > 0.8:  # Outside workspace
            return None
        
        joints[1] = math.asin((z - 0.3) / reach)
        
        # Other joints (simplified - set to reasonable values)
        joints[2] = 0.0
        joints[3] = -math.pi/4  # Elbow bend
        joints[4] = 0.0
        joints[5] = 0.0
        joints[6] = 0.0
        
        # Check joint limits
        for i, (min_val, max_val) in enumerate(self.joint_limits):
            if joints[i] < min_val or joints[i] > max_val:
                return None
        
        return joints
    
    def compute_jacobian(self) -> Optional[np.ndarray]:
        """Compute Jacobian matrix (simplified implementation)"""
        # Placeholder for Jacobian computation
        # Replace with proper analytical or numerical Jacobian
        
        # Return 6x7 Jacobian matrix (6 DOF Cartesian, 7 DOF joint)
        jacobian = np.random.rand(6, 7) * 0.1  # Placeholder
        return jacobian
    
    def is_pose_in_workspace(self, pose: PoseStamped) -> bool:
        """Check if pose is within workspace limits"""
        x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        
        return (self.workspace_limits[0] <= x <= self.workspace_limits[1] and
                self.workspace_limits[2] <= y <= self.workspace_limits[3] and
                self.workspace_limits[4] <= z <= self.workspace_limits[5])
    
    def publish_status(self):
        """Publish system status"""
        status_msg = String()
        
        if self.emergency_stop:
            status_msg.data = "EMERGENCY_STOP"
        elif not self.control_enabled:
            status_msg.data = "DISABLED"
        else:
            status_msg.data = "ACTIVE"
        
        self.status_pub.publish(status_msg)
    
    def enable_control_callback(self, request, response):
        """Enable control service"""
        if self.emergency_stop:
            response.success = False
            response.message = "Cannot enable: Emergency stop active"
        else:
            self.control_enabled = True
            response.success = True
            response.message = "Control enabled"
            self.get_logger().info("Control enabled")
        
        return response
    
    def disable_control_callback(self, request, response):
        """Disable control service"""
        self.control_enabled = False
        response.success = True
        response.message = "Control disabled"
        self.get_logger().info("Control disabled")
        return response
    
    def emergency_stop_callback(self, request, response):
        """Emergency stop service"""
        self.emergency_stop = True
        self.control_enabled = False
        
        # Stop all motion
        self.target_joints = self.current_joints.copy()
        
        response.success = True
        response.message = "Emergency stop activated"
        self.get_logger().warn("EMERGENCY STOP ACTIVATED")
        return response
    
    def get_fk_callback(self, request, response):
        """Get forward kinematics service"""
        pose = self.forward_kinematics(self.current_joints)
        if pose is not None:
            response.success = True
            response.message = f"Current pose: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}"
        else:
            response.success = False
            response.message = "Failed to compute forward kinematics"
        
        return response


def main(args=None):
    """Main entry point for the OpenArm controller node"""
    rclpy.init(args=args)
    
    try:
        node = OpenArmController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in OpenArm controller: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
