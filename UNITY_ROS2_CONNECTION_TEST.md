# Unity-ROS2 連線測試指南

本指南提供完整的 Unity 與 ROS 2 連線測試流程。

## 🎯 測試目標

驗證 Unity 與 ROS 2 之間的雙向通訊：
- ✅ TCP 連接建立
- ✅ 心跳訊號接收
- ✅ 姿態命令發送
- ✅ 關節命令發送
- ✅ 關節狀態接收
- ✅ 服務呼叫 (Ping/Pong)

## 🚀 Step 1: 啟動 ROS 2 服務

### 1.1 啟動 Docker 容器
```bash
cd C:\code\vr_robot\robot_ros2
docker-compose -f docker-compose-humble.yml up -d
```

### 1.2 啟動 TCP Endpoint 伺服器
```bash
docker exec -it ros2_humble bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws && source install/setup.bash

# 啟動 TCP 伺服器 (埠號 10000)
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

### 1.3 啟動 Unity 橋接節點 (新終端)
```bash
docker exec -it ros2_humble bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws && source install/setup.bash

# 啟動橋接節點
python3 /root/ros2_ws/install/unity_openarm_bridge/lib/unity_bridge_py/tcp_bridge_node
```

### 1.4 驗證服務狀態
```bash
# 檢查主題列表
ros2 topic list

# 應該看到以下主題：
# /unity/heartbeat
# /unity/pose
# /unity/joint_commands
# /openarm/joint_states
# /openarm/end_effector_pose
```

## 🧪 Step 2: Python 連線測試

### 2.1 執行自動化測試
```bash
docker exec -it ros2_humble bash
cd /root/ros2_ws
python3 test_unity_connection.py
```

### 2.2 預期輸出
```
🚀 開始 Unity-ROS2 連線測試
✅ 成功連接到 127.0.0.1:10000
🔔 測試心跳訊號訂閱...
📍 測試姿態命令發布...
🦾 測試關節命令發布...
🎉 所有測試通過！Unity 可以與 ROS 2 正常通訊。
```

### 2.3 啟動訊息監聽器 (可選)
```bash
# 在另一個終端啟動監聽器
docker exec -it ros2_humble bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws && source install/setup.bash
python3 ros2_message_monitor.py
```

## 🎮 Step 3: Unity 端設定

### 3.1 安裝必要套件
在 Unity Package Manager 中安裝：
- **ROS-TCP-Connector**: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

### 3.2 ROS 設定
1. 開啟 **Window > ROS Settings**
2. 設定參數：
   - **ROS IP Address**: `127.0.0.1`
   - **ROS Port**: `10000`
   - **Protocol**: `TCP`

### 3.3 建立測試場景
1. 建立新場景
2. 建立空的 GameObject 命名為 "ROS2Tester"
3. 將 `UnityROS2Tester.cs` 腳本附加到該物件
4. 在 Inspector 中調整測試參數

### 3.4 執行測試
1. 運行場景
2. 觀察 Console 輸出
3. 檢查 GUI 顯示的連線狀態

## 📊 Step 4: 驗證結果

### 4.1 Unity Console 預期輸出
```
🚀 開始 Unity-ROS2 連線測試
📡 嘗試連接到 ROS 2: 127.0.0.1:10000
✅ ROS 連接初始化完成
💓 收到心跳 #1: openarm_ros2_alive
📍 發布姿態: 位置=(0.5, 0, 0.3), 旋轉=(0, 0, 0)
🦾 發布關節命令: [0, 0.5, 0, -1, 0, 0.5, 0]
```

### 4.2 ROS 2 端驗證
```bash
# 檢查 Unity 發送的姿態
ros2 topic echo /unity/pose --once

# 檢查 Unity 發送的關節命令
ros2 topic echo /unity/joint_commands --once

# 測試服務呼叫
ros2 service call /unity/ping example_interfaces/srv/Trigger
```

### 4.3 網路連接驗證
```bash
# 檢查埠號是否開啟 (在 Windows 上)
netstat -ano | findstr 10000

# 在 Docker 容器內檢查
docker exec ros2_humble bash -c "lsof -i :10000 2>/dev/null || netstat -tlnp 2>/dev/null | grep 10000"
```

## 🔧 故障排除

### 問題 1: 連接失敗
**症狀**: Unity 無法連接到 127.0.0.1:10000
**解決方案**:
1. 確認 TCP Endpoint 正在運行
2. 檢查防火牆設定
3. 確認埠號未被佔用

### 問題 2: 收不到心跳
**症狀**: Unity 連接成功但收不到心跳訊號
**解決方案**:
1. 確認橋接節點正在運行
2. 檢查主題名稱是否正確
3. 驗證 QoS 設定

### 問題 3: 訊息格式錯誤
**症狀**: 收到 "無效 JSON" 錯誤
**解決方案**:
1. 檢查 ROS-TCP-Connector 版本
2. 確認訊息格式符合 ROS 2 標準
3. 檢查時間戳格式

### 問題 4: Docker 網路問題
**症狀**: 容器內外網路不通
**解決方案**:
1. 檢查 Docker 網路設定
2. 確認埠號映射正確
3. 重啟 Docker 服務

## 📈 效能測試

### 延遲測試
```bash
# 測試往返延遲
ros2 topic pub /unity/pose geometry_msgs/PoseStamped '{header: {frame_id: "test"}, pose: {position: {x: 1.0}}}' --once
```

### 頻率測試
```bash
# 測試高頻率發布
ros2 topic pub /unity/joint_commands sensor_msgs/JointState '{name: ["j1"], position: [0.0]}' --rate 10
```

### 頻寬測試
```bash
# 監控網路使用量
ros2 topic hz /unity/heartbeat
ros2 topic bw /openarm/joint_states
```

## 🎯 成功標準

✅ **基本連接**: Unity 可連接到 ROS 2 TCP 伺服器
✅ **雙向通訊**: Unity 可發送和接收訊息
✅ **即時性**: 延遲 < 50ms
✅ **穩定性**: 連續運行 5 分鐘無斷線
✅ **服務呼叫**: Ping/Pong 服務正常回應

## 📝 測試報告範本

```
Unity-ROS2 連線測試報告
========================

測試日期: ___________
測試環境: 
- OS: Windows 11
- Unity 版本: ___________
- ROS 2 版本: Humble
- Docker 版本: ___________

測試結果:
□ TCP 連接建立
□ 心跳訊號接收
□ 姿態命令發送
□ 關節命令發送
□ 關節狀態接收
□ 服務呼叫成功

效能指標:
- 連接延遲: _____ ms
- 訊息頻率: _____ Hz
- 錯誤率: _____ %

備註:
_________________________________
```

## 🔗 相關資源

- [ROS-TCP-Connector 文檔](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [OpenArm 專案](https://github.com/enactic/openarm)
- [ROS 2 Humble 文檔](https://docs.ros.org/en/humble/)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

---

**注意**: 此測試指南假設你已完成基本的 ROS 2 和 Unity 環境設定。如遇到問題，請參考故障排除章節或查閱相關文檔。
