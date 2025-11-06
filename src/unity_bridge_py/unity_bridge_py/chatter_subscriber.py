#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


class ChatterSubscriber(Node):
    def __init__(self):
        super().__init__('chatter_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Chatter Subscriber started, listening to /chatter')
        self.count = 0
        self.connection_announced = False

    def listener_callback(self, msg):
        if not self.connection_announced:
            # 第一次收到訊息時顯示連線成功
            print("\n" + "="*60)
            print("✅ Unity → ROS2 連線成功！")
            print("="*60 + "\n")
            self.connection_announced = True
        
        self.count += 1
        # 顯示更直觀的訊息
        print(f"[訊息 #{self.count}] Unity → ROS2: {msg.data}")
        self.get_logger().info(f'[{self.count}] Received from Unity: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    chatter_subscriber = ChatterSubscriber()
    
    try:
        rclpy.spin(chatter_subscriber)
    except KeyboardInterrupt:
        pass
    
    chatter_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

