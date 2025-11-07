#!/usr/bin/env python3
"""
ROS2 TCP Bridge Node for Unity Integration
處理 Unity 和 ROS2 之間的訊息橋接
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
import json
import time
from datetime import datetime

class UnityTCPBridge(Node):
    def __init__(self):
        super().__init__('unity_tcp_bridge')
        
        self.get_logger().info('🚀 Unity TCP Bridge 節點啟動...')
        
        # 參數設定
        self.declare_parameter('debug_mode', True)
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # 統計資訊
        self.messages_from_unity = 0
        self.messages_to_unity = 0
        self.last_heartbeat_time = None
        
        # ========== 訂閱者 (從 Unity 接收) ==========
        
        # 心跳訊號
        self.heartbeat_sub = self.create_subscription(
            String,
            '/unity/heartbeat',
            self.heartbeat_callback,
            10
        )
        
        # 關節命令
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/unity/joint_commands',
            self.joint_commands_callback,
            10
        )
        
        # 速度命令
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Unity 狀態
        self.unity_status_sub = self.create_subscription(
            String,
            '/unity/status',
            self.unity_status_callback,
            10
        )
        
        # ========== 發布者 (發送到 Unity) ==========
        
        # OpenArm 關節狀態
        self.joint_states_pub = self.create_publisher(
            JointState,
            '/openarm/joint_states',
            10
        )
        
        # OpenArm 狀態
        self.openarm_status_pub = self.create_publisher(
            String,
            '/openarm/status',
            10
        )
        
        # 回應 Unity 狀態
        self.unity_status_response_pub = self.create_publisher(
            String,
            '/unity/status',
            10
        )
        
        # ========== 訂閱者 (從 ROS2 系統接收) ==========
        
        # 真實的關節狀態（如果有硬體）
        self.real_joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',  # 真實硬體的關節狀態
            self.real_joint_states_callback,
            10
        )
        
        # ========== 定時器 ==========
        
        # 模擬關節狀態發布（每100ms）
        self.joint_state_timer = self.create_timer(0.1, self.publish_simulated_joint_states)
        
        # 狀態報告（每5秒）
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        # 心跳檢查（每3秒）
        self.heartbeat_check_timer = self.create_timer(3.0, self.check_heartbeat)
        
        # 模擬的關節位置
        self.simulated_joint_positions = [0.0] * 6
        self.target_joint_positions = [0.0] * 6
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        self.get_logger().info('✅ Unity TCP Bridge 初始化完成')
        self.get_logger().info('📡 等待 Unity 連接...')
    
    # ========== 回調函數 ==========
    
    def heartbeat_callback(self, msg):
        """處理來自 Unity 的心跳"""
        self.messages_from_unity += 1
        self.last_heartbeat_time = time.time()
        
        if self.debug_mode:
            self.get_logger().info(f'💓 收到 Unity 心跳: {msg.data}')
        
        # 回應心跳
        response = String()
        response.data = f'ros2_heartbeat_ack_{datetime.now().strftime("%H:%M:%S.%f")[:-3]}'
        self.unity_status_response_pub.publish(response)
    
    def joint_commands_callback(self, msg):
        """處理來自 Unity 的關節命令"""
        self.messages_from_unity += 1
        
        if msg.position and len(msg.position) > 0:
            self.get_logger().info(f'📥 收到關節命令: {len(msg.position)} 個關節')
            
            # 更新目標位置
            for i in range(min(len(msg.position), len(self.target_joint_positions))):
                self.target_joint_positions[i] = msg.position[i]
                
            # 顯示前3個關節的位置
            if self.debug_mode:
                for i in range(min(3, len(msg.position))):
                    if i < len(msg.name):
                        self.get_logger().info(f'   {msg.name[i]}: {msg.position[i]:.3f} rad')
            
            # 如果有真實硬體，這裡應該轉發命令到硬體控制器
            # self.forward_to_hardware_controller(msg)
    
    def cmd_vel_callback(self, msg):
        """處理來自 Unity 的速度命令"""
        self.messages_from_unity += 1
        
        if self.debug_mode:
            self.get_logger().info(
                f'📥 收到速度命令: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}'
            )
        
        # 如果有移動平台，這裡應該轉發命令
        # self.forward_to_mobile_base(msg)
    
    def unity_status_callback(self, msg):
        """處理來自 Unity 的狀態訊息"""
        self.messages_from_unity += 1
        
        if self.debug_mode:
            self.get_logger().info(f'📥 Unity 狀態: {msg.data}')
    
    def real_joint_states_callback(self, msg):
        """處理來自真實硬體的關節狀態"""
        # 轉發到 Unity
        unity_joint_state = JointState()
        unity_joint_state.header.stamp = self.get_clock().now().to_msg()
        unity_joint_state.header.frame_id = 'openarm_base'
        unity_joint_state.name = msg.name
        unity_joint_state.position = msg.position
        unity_joint_state.velocity = msg.velocity if msg.velocity else []
        unity_joint_state.effort = msg.effort if msg.effort else []
        
        self.joint_states_pub.publish(unity_joint_state)
        self.messages_to_unity += 1
        
        if self.debug_mode:
            self.get_logger().info(f'📤 轉發真實關節狀態到 Unity: {len(msg.name)} 個關節')
    
    # ========== 定時器函數 ==========
    
    def publish_simulated_joint_states(self):
        """發布模擬的關節狀態（用於測試）"""
        # 平滑移動到目標位置
        alpha = 0.1  # 平滑因子
        for i in range(len(self.simulated_joint_positions)):
            self.simulated_joint_positions[i] += alpha * (
                self.target_joint_positions[i] - self.simulated_joint_positions[i]
            )
        
        # 創建並發布關節狀態訊息
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'openarm_base'
        joint_state.name = self.joint_names
        joint_state.position = self.simulated_joint_positions
        joint_state.velocity = [0.0] * 6  # 速度設為0
        joint_state.effort = [0.0] * 6    # 力矩設為0
        
        self.joint_states_pub.publish(joint_state)
        self.messages_to_unity += 1
        
        # 每秒記錄一次（避免過多日誌）
        if int(time.time()) % 10 == 0 and time.time() % 1 < 0.1:
            self.get_logger().debug(f'📤 發送模擬關節狀態: {self.simulated_joint_positions[:3]}...')
    
    def publish_status(self):
        """發布橋接器狀態"""
        status_msg = String()
        
        # 檢查 Unity 連接狀態
        unity_connected = False
        if self.last_heartbeat_time:
            time_since_heartbeat = time.time() - self.last_heartbeat_time
            unity_connected = time_since_heartbeat < 5.0
        
        # 創建狀態訊息
        status_data = {
            'node': 'unity_tcp_bridge',
            'timestamp': datetime.now().isoformat(),
            'unity_connected': unity_connected,
            'messages_from_unity': self.messages_from_unity,
            'messages_to_unity': self.messages_to_unity,
            'simulated_mode': True  # 目前是模擬模式
        }
        
        status_msg.data = json.dumps(status_data)
        self.openarm_status_pub.publish(status_msg)
        
        # 簡化的狀態訊息給 Unity
        simple_status = String()
        simple_status.data = f'Bridge OK | Unity: {"✅" if unity_connected else "❌"} | Msgs: {self.messages_from_unity}/{self.messages_to_unity}'
        self.unity_status_response_pub.publish(simple_status)
        
        self.get_logger().info(
            f'📊 狀態: Unity {"連接" if unity_connected else "斷線"} | '
            f'收到: {self.messages_from_unity} | 發送: {self.messages_to_unity}'
        )
    
    def check_heartbeat(self):
        """檢查心跳狀態"""
        if self.last_heartbeat_time:
            time_since_heartbeat = time.time() - self.last_heartbeat_time
            
            if time_since_heartbeat > 5.0:
                self.get_logger().warning('⚠️ Unity 心跳超時！可能已斷線')
            elif time_since_heartbeat > 3.0:
                self.get_logger().warning(f'⚠️ {time_since_heartbeat:.1f}秒未收到 Unity 心跳')
        else:
            self.get_logger().debug('等待第一個 Unity 心跳...')
    
    # ========== 工具函數 ==========
    
    def destroy_node(self):
        """清理資源"""
        self.get_logger().info('🔄 Unity TCP Bridge 節點關閉')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    bridge_node = UnityTCPBridge()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()