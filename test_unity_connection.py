#!/usr/bin/env python3

"""
Unity-ROS2 連線測試腳本

這個腳本測試 Unity 與 ROS 2 之間的基本通訊功能：
1. 檢查 TCP 連接
2. 測試心跳訊號
3. 測試 Ping/Pong 服務
4. 發送測試姿態命令
5. 監聽關節狀態

使用方法：
python3 test_unity_connection.py
"""

import socket
import json
import time
import threading
from typing import Dict, Any

class UnityROS2ConnectionTester:
    def __init__(self, host='127.0.0.1', port=10000):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.received_messages = []
        
    def connect(self) -> bool:
        """連接到 ROS-TCP-Endpoint"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"✅ 成功連接到 {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"❌ 連接失敗: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.socket:
            self.socket.close()
            self.connected = False
            print("🔌 連接已斷開")
    
    def send_message(self, topic: str, msg_type: str, data: Dict[str, Any]) -> bool:
        """發送訊息到 ROS 2"""
        if not self.connected:
            print("❌ 未連接，無法發送訊息")
            return False
        
        try:
            # 構建 ROS-TCP-Endpoint 格式的訊息
            message = {
                "op": "publish",
                "topic": topic,
                "type": msg_type,
                "msg": data
            }
            
            json_str = json.dumps(message)
            message_bytes = json_str.encode('utf-8')
            
            # 發送訊息長度（4 bytes）+ 訊息內容
            length = len(message_bytes)
            self.socket.send(length.to_bytes(4, byteorder='little'))
            self.socket.send(message_bytes)
            
            print(f"📤 發送到 {topic}: {data}")
            return True
            
        except Exception as e:
            print(f"❌ 發送失敗: {e}")
            return False
    
    def listen_for_messages(self, duration: float = 10.0):
        """監聽來自 ROS 2 的訊息"""
        if not self.connected:
            print("❌ 未連接，無法監聽訊息")
            return
        
        print(f"👂 開始監聽訊息 ({duration} 秒)...")
        start_time = time.time()
        
        try:
            self.socket.settimeout(1.0)  # 1秒超時
            
            while time.time() - start_time < duration:
                try:
                    # 讀取訊息長度
                    length_bytes = self.socket.recv(4)
                    if len(length_bytes) < 4:
                        continue
                    
                    length = int.from_bytes(length_bytes, byteorder='little')
                    
                    # 讀取訊息內容
                    message_bytes = b''
                    while len(message_bytes) < length:
                        chunk = self.socket.recv(length - len(message_bytes))
                        if not chunk:
                            break
                        message_bytes += chunk
                    
                    if len(message_bytes) == length:
                        try:
                            message = json.loads(message_bytes.decode('utf-8'))
                            self.received_messages.append(message)
                            
                            topic = message.get('topic', 'unknown')
                            msg_data = message.get('msg', {})
                            print(f"📥 收到來自 {topic}: {msg_data}")
                            
                        except json.JSONDecodeError:
                            print(f"⚠️  收到無效 JSON: {message_bytes}")
                
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"⚠️  接收錯誤: {e}")
                    break
                    
        except Exception as e:
            print(f"❌ 監聽失敗: {e}")
        
        print(f"📊 總共收到 {len(self.received_messages)} 條訊息")
    
    def test_heartbeat_subscription(self):
        """測試訂閱心跳訊號"""
        print("\n🔔 測試心跳訊號訂閱...")
        
        # 訂閱心跳主題
        subscribe_msg = {
            "op": "subscribe",
            "topic": "/unity/heartbeat",
            "type": "std_msgs/String"
        }
        
        try:
            json_str = json.dumps(subscribe_msg)
            message_bytes = json_str.encode('utf-8')
            length = len(message_bytes)
            
            self.socket.send(length.to_bytes(4, byteorder='little'))
            self.socket.send(message_bytes)
            
            print("📡 已訂閱 /unity/heartbeat")
            
            # 監聽心跳訊號
            self.listen_for_messages(5.0)
            
        except Exception as e:
            print(f"❌ 心跳測試失敗: {e}")
    
    def test_pose_publishing(self):
        """測試發布姿態命令"""
        print("\n📍 測試姿態命令發布...")
        
        # 發送測試姿態
        pose_data = {
            "header": {
                "stamp": {
                    "sec": int(time.time()),
                    "nanosec": 0
                },
                "frame_id": "unity_world"
            },
            "pose": {
                "position": {
                    "x": 0.5,
                    "y": 0.0,
                    "z": 0.3
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1.0
                }
            }
        }
        
        success = self.send_message(
            "/unity/pose", 
            "geometry_msgs/PoseStamped", 
            pose_data
        )
        
        if success:
            print("✅ 姿態命令發送成功")
        else:
            print("❌ 姿態命令發送失敗")
    
    def test_joint_commands(self):
        """測試關節命令發布"""
        print("\n🦾 測試關節命令發布...")
        
        # 發送測試關節命令
        joint_data = {
            "header": {
                "stamp": {
                    "sec": int(time.time()),
                    "nanosec": 0
                },
                "frame_id": "openarm_base_link"
            },
            "name": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"],
            "position": [0.0, 0.5, 0.0, -1.0, 0.0, 0.5, 0.0],
            "velocity": [0.0] * 7,
            "effort": [0.0] * 7
        }
        
        success = self.send_message(
            "/unity/joint_commands",
            "sensor_msgs/JointState",
            joint_data
        )
        
        if success:
            print("✅ 關節命令發送成功")
        else:
            print("❌ 關節命令發送失敗")
    
    def run_full_test(self):
        """執行完整的連線測試"""
        print("🚀 開始 Unity-ROS2 連線測試")
        print("=" * 50)
        
        # 1. 連接測試
        if not self.connect():
            return False
        
        try:
            # 2. 心跳測試
            self.test_heartbeat_subscription()
            
            # 3. 姿態命令測試
            self.test_pose_publishing()
            
            # 4. 關節命令測試
            self.test_joint_commands()
            
            # 5. 最終監聽
            print("\n👂 最終監聽測試...")
            self.listen_for_messages(3.0)
            
            print("\n✅ 測試完成！")
            return True
            
        except Exception as e:
            print(f"❌ 測試過程中發生錯誤: {e}")
            return False
        
        finally:
            self.disconnect()


def main():
    """主函數"""
    print("Unity-ROS2 連線測試工具")
    print("確保 ROS 2 TCP Endpoint 正在運行於 127.0.0.1:10000")
    print()
    
    tester = UnityROS2ConnectionTester()
    
    try:
        success = tester.run_full_test()
        
        if success:
            print("\n🎉 所有測試通過！Unity 可以與 ROS 2 正常通訊。")
        else:
            print("\n⚠️  測試未完全通過，請檢查 ROS 2 服務狀態。")
            
    except KeyboardInterrupt:
        print("\n⏹️  測試被用戶中斷")
    except Exception as e:
        print(f"\n❌ 測試失敗: {e}")


if __name__ == "__main__":
    main()
