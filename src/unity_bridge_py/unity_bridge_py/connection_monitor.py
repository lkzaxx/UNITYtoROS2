#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys


class ConnectionMonitor(Node):
    def __init__(self):
        super().__init__('connection_monitor')
        
        # 訂閱 Unity 的訊息
        self.unity_subscription = self.create_subscription(
            String,
            '/chatter',
            self.unity_message_callback,
            10)
        
        # 發布測試訊息給 Unity
        self.unity_publisher = self.create_publisher(String, '/chatter', 10)
        
        # 狀態標記
        self.unity_connected = False
        self.unity_received_count = 0
        self.last_unity_message_time = None
        
        # 定時器：每 5 秒檢查連線狀態
        self.status_timer = self.create_timer(5.0, self.print_status)
        
        # 定時器：每 2 秒發送測試訊息給 Unity
        self.test_publisher_timer = self.create_timer(2.0, self.send_test_message)
        self.test_message_count = 0
        
        print("\n" + "="*70)
        print("🔍 Unity ↔ ROS2 連線監控器已啟動")
        print("="*70)
        print("等待 Unity 連接...\n")

    def unity_message_callback(self, msg):
        """收到來自 Unity 的訊息"""
        if not self.unity_connected:
            self.unity_connected = True
            print("\n" + "="*70)
            print("✅ Unity → ROS2 連線成功！")
            print("="*70 + "\n")
        
        self.unity_received_count += 1
        self.last_unity_message_time = time.time()
        
        # 顯示收到的訊息
        print(f"📨 [Unity → ROS2] 訊息 #{self.unity_received_count}: {msg.data}")

    def send_test_message(self):
        """發送測試訊息給 Unity"""
        if self.unity_connected:  # 只有在連線成功後才發送
            self.test_message_count += 1
            msg = String()
            msg.data = f"ROS2 → Unity 測試訊息 #{self.test_message_count}"
            self.unity_publisher.publish(msg)
            print(f"📤 [ROS2 → Unity] 已發送: {msg.data}")

    def print_status(self):
        """定期顯示連線狀態"""
        print("\n" + "-"*70)
        print("📊 連線狀態報告")
        print("-"*70)
        
        if self.unity_connected:
            print("✅ Unity → ROS2: 已連接")
            print(f"   已接收訊息數: {self.unity_received_count}")
            if self.last_unity_message_time:
                elapsed = time.time() - self.last_unity_message_time
                if elapsed < 3:
                    print(f"   最後訊息時間: {elapsed:.1f} 秒前 ✅")
                else:
                    print(f"   最後訊息時間: {elapsed:.1f} 秒前 ⚠️")
        else:
            print("❌ Unity → ROS2: 未連接")
            print("   等待 Unity 發布訊息...")
        
        print(f"📤 ROS2 → Unity: 已發送 {self.test_message_count} 條測試訊息")
        print("-"*70 + "\n")


def main(args=None):
    rclpy.init(args=args)
    connection_monitor = ConnectionMonitor()
    
    try:
        rclpy.spin(connection_monitor)
    except KeyboardInterrupt:
        print("\n" + "="*70)
        print("🛑 連線監控器已停止")
        print("="*70 + "\n")
    
    connection_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

