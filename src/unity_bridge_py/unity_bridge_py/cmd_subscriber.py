#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CmdSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/unity/cmd',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Command Subscriber started, listening to /unity/cmd')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command: "{msg.data}"')
        
        # Process different commands
        if msg.data.lower().startswith('move'):
            self.get_logger().info('Processing movement command...')
        elif msg.data.lower().startswith('stop'):
            self.get_logger().info('Processing stop command...')
        elif msg.data.lower().startswith('status'):
            self.get_logger().info('Processing status request...')
        else:
            self.get_logger().info('Unknown command received')


def main(args=None):
    rclpy.init(args=args)
    cmd_subscriber = CmdSubscriber()
    
    try:
        rclpy.spin(cmd_subscriber)
    except KeyboardInterrupt:
        pass
    
    cmd_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
