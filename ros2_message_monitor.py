#!/usr/bin/env python3

"""
ROS 2 訊息監聽器

監聽來自 Unity 的訊息並顯示詳細資訊
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from example_interfaces.srv import Trigger

class ROS2MessageMonitor(Node):
    def __init__(self):
        super().__init__('ros2_message_monitor')
        
        # 計數器
        self.heartbeat_count = 0
        self.pose_count = 0
        self.joint_count = 0
        
        # 訂閱者
        self.heartbeat_sub = self.create_subscription(
            String,
            '/unity/heartbeat',
            self.heartbeat_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/unity/pose',
            self.pose_callback,
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/unity/joint_commands',
            self.joint_callback,
            10
        )
        
        # 服務
        self.ping_service = self.create_service(
            Trigger,
            '/unity/ping',
            self.ping_callback
        )
        
        # 定時器
        self.timer = self.create_timer(5.0, self.print_statistics)
        
        self.get_logger().info('ROS 2 訊息監聽器已啟動')
        self.get_logger().info('監聽主題:')
        self.get_logger().info('  - /unity/heartbeat')
        self.get_logger().info('  - /unity/pose')
        self.get_logger().info('  - /unity/joint_commands')
        self.get_logger().info('提供服務:')
        self.get_logger().info('  - /unity/ping')
    
    def heartbeat_callback(self, msg):
        self.heartbeat_count += 1
        self.get_logger().info(f'💓 心跳 #{self.heartbeat_count}: {msg.data}')
    
    def pose_callback(self, msg):
        self.pose_count += 1
        pos = msg.pose.position
        ori = msg.pose.orientation
        self.get_logger().info(
            f'📍 姿態 #{self.pose_count}: '
            f'位置=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) '
            f'方向=({ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f}) '
            f'frame={msg.header.frame_id}'
        )
    
    def joint_callback(self, msg):
        self.joint_count += 1
        if len(msg.position) > 0:
            positions = [f'{pos:.3f}' for pos in msg.position[:7]]
            self.get_logger().info(
                f'🦾 關節 #{self.joint_count}: [{", ".join(positions)}]'
            )
    
    def ping_callback(self, request, response):
        response.success = True
        response.message = "pong from ROS2"
        self.get_logger().info('📞 收到 Ping，回應 Pong')
        return response
    
    def print_statistics(self):
        self.get_logger().info(
            f'📊 統計 - 心跳: {self.heartbeat_count}, '
            f'姿態: {self.pose_count}, 關節: {self.joint_count}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    monitor = ROS2MessageMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
