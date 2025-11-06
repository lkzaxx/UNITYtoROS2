#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class ChatterPublisher(Node):
    def __init__(self):
        super().__init__('chatter_publisher')
        self.publisher_ = self.create_publisher(String, '/chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2Hz (每 0.5 秒)
        self.count = 0
        self.get_logger().info('Chatter Publisher started, publishing to /chatter')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from ROS2 Docker! Message #{self.count} at {time.strftime("%H:%M:%S")}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    chatter_publisher = ChatterPublisher()
    
    try:
        rclpy.spin(chatter_publisher)
    except KeyboardInterrupt:
        pass
    
    chatter_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

