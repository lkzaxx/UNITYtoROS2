# OpenArm × Unity ROS 2 整合專案

本專案提供一個完整的 Unity 與 ROS 2 整合解決方案，專為 OpenArm 7DOF 人形機械手臂設計。支援 ROS 2 Jazzy 和 Humble 雙版本，使用 ROS-TCP-Endpoint 進行可靠的 TCP 通訊。

## 架構

- **Windows 11**: 運行 Unity 與 ROS-TCP-Connector 的主機系統
- **WSL2 Ubuntu 22.04/24.04**: Docker 主機
- **Docker 容器**: ROS 2 Humble/Jazzy desktop-full 環境
- **OpenArm**: 7DOF 開源人形機械手臂專案
- **通訊**: TCP (ROS-TCP-Endpoint) 用於穩定的跨平台通訊

## 快速開始

### 前置需求
- 啟用 WSL2 的 Windows 11
- 具備 WSL2 整合的 Docker Desktop
- Ubuntu 22.04/24.04 WSL 發行版
- Unity 2022.3 LTS 或更新版本
- ROS-TCP-Connector Unity 套件

### 環境選擇

本專案支援兩個 ROS 2 版本：

#### ROS 2 Humble (推薦)
```bash
# 啟動 Humble 環境
docker-compose -f docker-compose-humble.yml up -d
docker exec -it ros2_humble bash
```

#### ROS 2 Jazzy
```bash
# 啟動 Jazzy 環境
docker-compose up -d
docker exec -it ros2_jazzy bash
```

### 建置專案

1. **進入容器並建置:**
   ```bash
   # Humble 版本
   source /opt/ros/humble/setup.bash
   # 或 Jazzy 版本
   # source /opt/ros/jazzy/setup.bash
   
   cd /root/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **啟動 Unity 橋接系統:**
   ```bash
   # 終端 1: 啟動 TCP Endpoint 伺服器
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
   
   # 終端 2: 啟動 Unity 橋接節點
   python3 /root/ros2_ws/install/unity_openarm_bridge/lib/unity_bridge_py/tcp_bridge_node
   
   # 終端 3: 啟動 OpenArm 控制器
   ros2 run unity_openarm_bridge openarm_controller
   ```

### 測試連線

1. **執行自動化連線測試:**
   ```bash
   cd /root/ros2_ws
   python3 test_unity_connection.py
   ```

2. **檢查心跳訊號:**
   ```bash
   ros2 topic echo /unity/heartbeat
   ```

3. **測試姿態命令:**
   ```bash
   ros2 topic pub /unity/pose geometry_msgs/PoseStamped '{header: {frame_id: "unity_world"}, pose: {position: {x: 0.5, y: 0.0, z: 0.3}}}' --once
   ```

4. **測試服務呼叫:**
   ```bash
   ros2 service call /unity/ping example_interfaces/srv/Trigger
   ```

5. **監聽 OpenArm 狀態:**
   ```bash
   ros2 topic echo /openarm/joint_states
   ros2 topic echo /openarm/end_effector_pose
   ```

## ROS 2 主題與服務

### Unity → ROS 2 (Unity 發布)
- `/unity/pose`: `geometry_msgs/PoseStamped` - Unity 發送的目標姿態
- `/unity/joint_commands`: `sensor_msgs/JointState` - Unity 發送的關節命令
- `/unity/gripper_command`: `std_msgs/String` - Unity 發送的夾爪命令

### ROS 2 → Unity (ROS 2 發布)
- `/unity/heartbeat`: `std_msgs/String` - 系統心跳訊號 (1 Hz)
- `/openarm/joint_states`: `sensor_msgs/JointState` - OpenArm 當前關節狀態
- `/openarm/end_effector_pose`: `geometry_msgs/PoseStamped` - 末端執行器姿態

### 服務
- `/unity/ping`: `example_interfaces/srv/Trigger` - 連線測試服務
- `/openarm/home_position`: `std_srvs/srv/Trigger` - 回到初始位置
- `/openarm/enable_control`: `std_srvs/srv/Trigger` - 啟用/停用控制
- `/openarm/emergency_stop`: `std_srvs/srv/Trigger` - 緊急停止

## 使用 Cursor 開發

### ROS 2 Humble 環境
1. 在 WSL 中開啟 Cursor: `cursor ~/ros2_ws`
2. 選擇 `.devcontainer/devcontainer-humble.json` 配置
3. 使用 "Reopen in Container" 在 ROS 2 Humble 環境內工作

### ROS 2 Jazzy 環境
1. 在 WSL 中開啟 Cursor: `cursor ~/ros2_ws`
2. 選擇 `.devcontainer/devcontainer.json` 配置
3. 使用 "Reopen in Container" 在 ROS 2 Jazzy 環境內工作

兩個配置都已預先安裝必要的擴展和工具。

## 配置

- **通訊協定**: TCP (ROS-TCP-Endpoint)
- **預設埠號**: 10000
- **DDS**: Cyclone DDS (用於 ROS 2 內部通訊)
- **域 ID**: 0
- **網路**: 橋接模式，支援埠號映射

## Unity 整合

### 架構說明

**重要**：ROS 2 運行在 Docker 容器內，Unity 運行在 Windows 主機上。

```
┌─────────────────┐         TCP (10000)         ┌──────────────────┐
│   Unity (Windows)│  ←──────────────────────→  │  ROS 2 (Container)│
│                  │                             │                  │
│ ROS-TCP-Connector│                             │ ROS-TCP-Endpoint │
└─────────────────┘                             └──────────────────┘
```

**連接流程**：
1. Docker 容器內的 ROS 2 啟動 TCP Endpoint 伺服器（監聽 `0.0.0.0:10000`）
2. Docker 端口映射將容器的 `10000` 映射到主機的 `10000`
3. Unity 從 Windows 主機連接到 `127.0.0.1:10000`

### 前置準備

#### 1. 確保 ROS 2 容器正在運行
```bash
# 啟動容器（如果尚未啟動）
docker-compose -f docker-compose-humble.yml up -d

# 檢查容器狀態
docker ps | grep ros2_humble
```

#### 2. 啟動 ROS 2 服務（在容器內）

**方法 A：使用 Windows .bat 腳本（推薦）**

在 Windows 上，可以使用專案提供的 .bat 腳本快速啟動服務：

**選項 1：一鍵啟動所有服務（最簡單）**
```batch
# 雙擊執行 start_all_services.bat
# 會自動開啟兩個視窗分別運行 TCP Endpoint 和橋接節點
start_all_services.bat
```

**選項 2：分別啟動服務**
```batch
# 終端 1：啟動 TCP Endpoint 伺服器
start_tcp_endpoint.bat

# 終端 2：啟動 Unity 橋接節點（需要新開一個終端）
start_unity_bridge.bat
```

**方法 B：手動執行命令**

**終端 1：啟動 TCP Endpoint 伺服器**
```bash
docker exec -it ros2_humble bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws && source install/setup.bash

# 啟動 TCP 伺服器
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

**終端 2：啟動 Unity 橋接節點**
```bash
docker exec -it ros2_humble bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws && source install/setup.bash

# 啟動橋接節點
python3 /root/ros2_ws/install/unity_openarm_bridge/lib/unity_bridge_py/tcp_bridge_node
```

### Unity 端設定

#### 1. 安裝 ROS-TCP-Connector
在 Unity Package Manager 中添加：
```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

#### 2. 設定 ROS 連接參數
1. 開啟 **Window > ROS Settings**
2. 設定參數：
   - **ROS IP Address**: `127.0.0.1`（本地主機）
   - **ROS Port**: `10000`（TCP 端口）
   - **Protocol**: `TCP`

#### 3. 建立測試場景
1. 建立新場景或使用現有場景
2. 建立空的 GameObject 命名為 "ROS2Tester"
3. 將 `UnityROS2Tester.cs` 腳本附加到該物件
4. 在 Inspector 中確認連線設定：
   - `rosIPAddress`: `127.0.0.1`
   - `rosPort`: `10000`

#### 4. 執行測試
1. 確保 ROS 2 服務正在運行（見上方「前置準備」）
2. 運行 Unity 場景
3. 觀察 Console 輸出，應該看到：
   ```
   🚀 開始 Unity-ROS2 連線測試
   📡 嘗試連接到 ROS 2: 127.0.0.1:10000
   ✅ ROS 連接初始化完成
   💓 收到心跳 #1: openarm_ros2_alive
   ```

### 驗證連接

#### 在 ROS 2 端驗證
```bash
# 檢查 Unity 發送的姿態
ros2 topic echo /unity/pose --once

# 檢查 Unity 發送的關節命令
ros2 topic echo /unity/joint_commands --once

# 監聽心跳訊號（應該每秒收到一次）
ros2 topic echo /unity/heartbeat
```

#### 在 Windows 端驗證
```powershell
# 檢查端口是否開啟
netstat -ano | findstr 10000
```

### 使用範例腳本
參考專案中的 `UnityROS2Tester.cs` 進行基本通訊測試。詳細測試流程請參考 `UNITY_ROS2_CONNECTION_TEST.md`。

## 專案結構

```
ros2_ws/
├── src/
│   ├── openarm_ros2/                    # OpenArm ROS 2 套件
│   ├── openarm_description/             # OpenArm URDF 描述
│   ├── ros_tcp_endpoint/                # Unity TCP 通訊端點
│   └── unity_openarm_bridge/            # Unity-OpenArm 橋接套件
│       ├── unity_openarm_bridge/
│       │   ├── tcp_bridge_node.py      # TCP 橋接節點
│       │   └── openarm_controller.py   # OpenArm 控制器
│       ├── launch/
│       │   ├── openarm_unity_bridge.launch.py    # 完整系統啟動
│       │   └── simulation_bridge.launch.py       # 模擬測試啟動
│       ├── config/
│       │   └── openarm_config.yaml     # OpenArm 配置
│       └── setup.py
├── .devcontainer/
│   ├── devcontainer.json               # Jazzy 開發容器配置
│   └── devcontainer-humble.json        # Humble 開發容器配置
├── docker-compose.yml                  # Jazzy Docker 設定
├── docker-compose-humble.yml           # Humble Docker 設定
├── cyclonedds.xml                      # DDS 配置
├── test_unity_connection.py            # 連線測試腳本
├── UnityROS2Tester.cs                  # Unity 測試腳本
├── UNITY_ROS2_CONNECTION_TEST.md       # 完整測試指南
└── plan.md                             # 專案計劃文檔
```

## 故障排除

### 常見問題

1. **TCP 連接失敗**
   - 確保 TCP Endpoint 伺服器正在運行
   - 檢查埠號 10000 是否被佔用
   - 驗證防火牆設定

2. **收不到心跳訊號**
   - 確認橋接節點正在運行
   - 檢查主題名稱是否正確
   - 驗證 Unity 訂閱設定

3. **Docker 容器問題**
   - 重新建置容器：`docker-compose down && docker-compose up -d`
   - 檢查容器狀態：`docker ps`
   - 查看容器日誌：`docker logs ros2_humble`

4. **建置錯誤**
   - 確保所有依賴已安裝：`rosdep install --from-paths src --ignore-src -r -y`
   - 清理建置：`rm -rf build install log && colcon build`

### 詳細測試指南
參考 `UNITY_ROS2_CONNECTION_TEST.md` 獲得完整的測試流程和故障排除步驟。

## 相關資源

- [OpenArm 專案](https://github.com/enactic/OpenArm)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS 2 Humble 文檔](https://docs.ros.org/en/humble/)
- [ROS 2 Jazzy 文檔](https://docs.ros.org/en/jazzy/)

## 授權

本專案採用 Apache 2.0 授權條款。詳見各子專案的授權文件。
