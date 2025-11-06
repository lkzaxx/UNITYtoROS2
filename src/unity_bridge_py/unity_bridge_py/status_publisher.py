#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher_ = self.create_publisher(String, '/unity/status', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2Hz
        self.count = 0
        self.get_logger().info('Status Publisher started, publishing to /unity/status')

    def timer_callback(self):
        msg = String()
        msg.data = f'ROS2 Status #{self.count}: {time.strftime("%H:%M:%S")}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    status_publisher = StatusPublisher()
    
    try:
        rclpy.spin(status_publisher)
    except KeyboardInterrupt:
        pass
    
    status_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
