#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Cmd Vel Subscriber started, listening to /cmd_vel')
        self.count = 0
        self.connection_announced = False

    def cmd_vel_callback(self, msg):
        if not self.connection_announced:
            # 第一次收到訊息時顯示連線成功
            print("\n" + "="*60)
            print("🚗 Unity → ROS2 Cmd Vel 連線成功！")
            print("="*60 + "\n")
            self.connection_announced = True
        
        self.count += 1
        # 顯示更直觀的訊息
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        print(f"[Cmd #{self.count}] Unity → ROS2: 線速度={linear_x:.2f} m/s, 角速度={angular_z:.2f} rad/s")
        self.get_logger().info(f'[{self.count}] Received Twist: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = CmdVelSubscriber()
    
    try:
        rclpy.spin(cmd_vel_subscriber)
    except KeyboardInterrupt:
        pass
    
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
