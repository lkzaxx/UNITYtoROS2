#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
import sys
import random


class SimpleUnityBridge(Node):
    def __init__(self):
        super().__init__('simple_unity_bridge')
        
        # 訂閱 Unity 的訊息
        self.unity_subscription = self.create_subscription(
            String,
            '/chatter',
            self.unity_message_callback,
            10)
        
        # 發布訊息給 Unity
        self.unity_publisher = self.create_publisher(String, '/chatter', 10)
        
        # 狀態標記
        self.unity_connected = False
        self.unity_received_count = 0
        self.sent_message_count = 0
        self.last_unity_message_time = None
        self.last_sent_messages = []  # 記錄最近發送的訊息，避免接收到自己的訊息
        self.auto_send_enabled = False  # 自動發送開關
        
        # 定時器：每 1 秒發送座標
        self.coordinate_timer = self.create_timer(1.0, self.auto_send_coordinates)
        
        # 定時器：每 10 秒顯示狀態
        self.status_timer = self.create_timer(10.0, self.print_status)
        
        print("\n" + "="*80)
        print("🔗 Unity ↔ ROS2 簡單橋接器已啟動")
        print("="*80)
        print("功能說明：")
        print("  📨 自動接收來自 Unity 的訊息")
        print("  📤 持續發送隨機座標給 Unity (每秒1次)")
        print("  📊 定期顯示連線狀態")
        print("  🚫 無自循環 - 過濾自己發送的訊息")
        print("="*80)
        print("等待 Unity 連接...")
        print("輸入指令:")
        print("  start              - 開始持續發送座標")
        print("  stop               - 停止發送座標")
        print("  coord              - 手動發送一次座標")
        print("  status             - 顯示當前狀態")
        print("  quit               - 退出程式")
        print("-"*80 + "\n")
        
        # 啟動輸入線程
        self.input_thread = threading.Thread(target=self.input_handler, daemon=True)
        self.input_thread.start()

    def unity_message_callback(self, msg):
        """收到來自 Unity 的訊息"""
        # 檢查是否是自己剛發送的訊息
        if msg.data in self.last_sent_messages:
            # 從記錄中移除，避免重複過濾
            self.last_sent_messages.remove(msg.data)
            return  # 忽略自己發送的訊息
        
        if not self.unity_connected:
            self.unity_connected = True
            print("\n" + "="*80)
            print("✅ Unity → ROS2 連線成功！")
            print("="*80)
        
        self.unity_received_count += 1
        self.last_unity_message_time = time.time()
        
        # 簡化顯示收到的訊息
        print(f"接收到unity訊息:")
        print(f"內容為: {msg.data}")
        print("-"*40)

    def send_random_coordinates(self):
        """發送隨機座標給 Unity"""
        x = random.randint(0, 100)
        y = random.randint(0, 100)
        z = random.randint(0, 100)
        
        message_content = f"X:{x} Y:{y} Z:{z}"
        
        self.sent_message_count += 1
        msg = String()
        msg.data = message_content
        
        # 記錄發送的訊息，避免接收到自己的訊息
        self.last_sent_messages.append(message_content)
        # 只保留最近5條訊息記錄
        if len(self.last_sent_messages) > 5:
            self.last_sent_messages.pop(0)
        
        self.unity_publisher.publish(msg)
        
        # 簡化顯示發送的訊息
        print(f"發送至unity訊息: {message_content}")
        print("-"*40)

    def auto_send_coordinates(self):
        """自動發送座標（定時器回調）"""
        if self.auto_send_enabled:
            self.send_random_coordinates()

    def print_status(self):
        """顯示連線狀態"""
        print("\n" + "="*80)
        print("📊 Unity ↔ ROS2 橋接狀態報告")
        print("="*80)
        
        # Unity → ROS2 狀態
        if self.unity_connected:
            print("✅ Unity → ROS2: 已連接")
            print(f"   📨 已接收訊息數: {self.unity_received_count}")
            if self.last_unity_message_time:
                elapsed = time.time() - self.last_unity_message_time
                if elapsed < 5:
                    print(f"   ⏰ 最後訊息時間: {elapsed:.1f} 秒前 ✅")
                elif elapsed < 30:
                    print(f"   ⏰ 最後訊息時間: {elapsed:.1f} 秒前 ⚠️")
                else:
                    print(f"   ⏰ 最後訊息時間: {elapsed:.1f} 秒前 ❌")
        else:
            print("❌ Unity → ROS2: 未連接")
            print("   等待 Unity 發布訊息...")
        
        # ROS2 → Unity 狀態
        auto_status = "🟢 運行中" if self.auto_send_enabled else "🔴 已停止"
        print(f"📤 ROS2 → Unity: 已發送 {self.sent_message_count} 條訊息")
        print(f"🔄 自動發送狀態: {auto_status}")
        
        print("="*80)
        print("輸入指令: start | stop | coord | status | quit")
        print("-"*80 + "\n")

    def input_handler(self):
        """處理用戶輸入"""
        while True:
            try:
                user_input = input().strip()
                
                if user_input.lower() == 'quit':
                    print("\n🛑 正在退出橋接器...")
                    break
                elif user_input.lower() == 'status':
                    self.print_status()
                elif user_input.lower() == 'start':
                    if not self.auto_send_enabled:
                        self.auto_send_enabled = True
                        print("🟢 開始持續發送座標 (每秒1次)")
                    else:
                        print("⚠️ 自動發送已經在運行中")
                elif user_input.lower() == 'stop':
                    if self.auto_send_enabled:
                        self.auto_send_enabled = False
                        print("🔴 停止持續發送座標")
                    else:
                        print("⚠️ 自動發送已經停止")
                elif user_input.lower() == 'coord':
                    self.send_random_coordinates()
                elif user_input:
                    print("❌ 未知指令，可用指令:")
                    print("   start          - 開始持續發送座標")
                    print("   stop           - 停止發送座標")
                    print("   coord          - 手動發送一次座標")
                    print("   status         - 顯示當前狀態")
                    print("   quit           - 退出程式")
                    
            except EOFError:
                break
            except Exception as e:
                print(f"❌ 輸入處理錯誤: {e}")


def main(args=None):
    rclpy.init(args=args)
    bridge = SimpleUnityBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("\n" + "="*80)
        print("🛑 Unity ↔ ROS2 簡單橋接器已停止")
        print("="*80)
    except Exception as e:
        print(f"\n❌ 橋接器錯誤: {e}")
    finally:
        try:
            bridge.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
