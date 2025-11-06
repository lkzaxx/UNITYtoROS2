# OpenArm Unity 整合專案重構計畫

## 目標與範疇

**主要目標**：將現有的 Unity-ROS2 橋接系統重構為基於 OpenArm 和 ROS-TCP-Endpoint 的架構

**測試環境**：Unity（ROS-TCP-Connector）與 ROS 2 在同一台電腦
**橋接方式**：ROS 2 端使用 ROS-TCP-Endpoint 開 TCP 伺服器（預設 :10000）
**目標硬體**：OpenArm 7DOF 人形機械手臂

## 最小可用功能（MVP）

### 基礎通訊
- [x] 接收 Unity 發來的 `/unity/pose`（geometry_msgs/PoseStamped）
- [x] 每秒發布 `/unity/heartbeat`（std_msgs/String）
- [x] 提供一個 `/unity/ping` 服務（回傳 pong），方便 Unity 端健康檢查

### OpenArm 整合
- [ ] 接收 Unity 關節命令 `/unity/joint_commands`（sensor_msgs/JointState）
- [ ] 發布 OpenArm 關節狀態 `/openarm/joint_states`（sensor_msgs/JointState）
- [ ] 提供 OpenArm 控制服務 `/openarm/move_to_pose`（geometry_msgs/PoseStamped）

**驗收標準**：Unity 能連上 127.0.0.1:10000，ROS 端可看到 Pose 收到，Unity 能收到心跳與 pong，並能控制 OpenArm 模擬

## 環境配置

**OS**：WSL2 Ubuntu 22.04（已配置）
**ROS 2**：Humble（已配置並測試）
**連接埠**：10000/TCP
**容器**：Docker Compose（已配置）

## 專案重構結構

```
ros2_ws/
├─ src/
│  ├─ ros_tcp_endpoint/                # Unity 官方 TCP 橋接器
│  ├─ openarm_ros2/                    # OpenArm ROS2 套件
│  ├─ openarm_description/             # OpenArm URDF 描述檔案
│  ├─ unity_openarm_bridge/            # 新的整合橋接套件（取代 unity_bridge_py）
│  │  ├─ package.xml
│  │  ├─ setup.py
│  │  ├─ resource/
│  │  │  └─ unity_openarm_bridge
│  │  ├─ unity_openarm_bridge/
│  │  │  ├─ __init__.py
│  │  │  ├─ tcp_bridge_node.py         # TCP 橋接主節點
│  │  │  ├─ openarm_controller.py      # OpenArm 控制邏輯
│  │  │  └─ qos_profiles.yaml          # QoS 設定
│  │  ├─ launch/
│  │  │  ├─ openarm_unity_bridge.launch.py  # 完整系統啟動
│  │  │  └─ simulation_bridge.launch.py     # 純模擬測試
│  │  └─ config/
│  │     └─ openarm_config.yaml        # OpenArm 參數配置
└─ .devcontainer/（已配置 Humble）
```

## 任務清單（按優先順序）

### Phase 1: 清理現有架構
- [ ] **1.1** 刪除不必要的檔案
  - [ ] 刪除所有 `.md` 說明檔（保留 README.md）
  - [ ] 刪除 `backup_to_windows.sh`, `export_code.sh` 等腳本
  - [ ] 刪除 `start-humble.*` 啟動檔
  - [ ] 清理 `unity_bridge_py` 舊套件中的測試節點

- [ ] **1.2** 保留核心配置
  - [x] 保留 `docker-compose-humble.yml`（已優化）
  - [x] 保留 `docker-entrypoint-humble.sh`（已優化）
  - [x] 保留 `.devcontainer/` 配置
  - [ ] 保留 `cyclonedx.xml` DDS 配置

### Phase 2: 安裝 OpenArm 依賴
- [ ] **2.1** 克隆 OpenArm 相關套件
  ```bash
  cd ~/ros2_ws/src
  git clone https://github.com/enactic/openarm_ros2.git
  git clone https://github.com/enactic/openarm_description.git
  ```

- [ ] **2.2** 安裝 ROS-TCP-Endpoint
  ```bash
  git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git ros_tcp_endpoint
  ```

- [ ] **2.3** 安裝依賴套件
  ```bash
  rosdep install --from-paths src --ignore-src -r -y
  ```

### Phase 3: 建立新的橋接套件
- [ ] **3.1** 建立 `unity_openarm_bridge` 套件
  ```bash
  ros2 pkg create --build-type ament_python unity_openarm_bridge \
    --dependencies rclpy geometry_msgs sensor_msgs std_msgs example_interfaces
  ```

- [ ] **3.2** 實作核心節點
  - [ ] `tcp_bridge_node.py`：處理 Unity TCP 通訊
  - [ ] `openarm_controller.py`：OpenArm 控制邏輯
  - [ ] QoS 配置檔案

- [ ] **3.3** 建立 Launch 檔案
  - [ ] `openarm_unity_bridge.launch.py`：完整系統
  - [ ] `simulation_bridge.launch.py`：純模擬測試

### Phase 4: 整合測試
- [ ] **4.1** 編譯測試
  ```bash
  colcon build --symlink-install
  source install/setup.bash
  ```

- [ ] **4.2** 啟動測試
  ```bash
  ros2 launch unity_openarm_bridge simulation_bridge.launch.py
  ```

- [ ] **4.3** 驗證功能
  - [ ] TCP 連接 (127.0.0.1:10000)
  - [ ] 心跳訊號
  - [ ] Ping/Pong 服務
  - [ ] OpenArm 關節控制

## 互動介面設計

### Topics

**訂閱（ROS 2 ← Unity）**
- `/unity/pose` : geometry_msgs/PoseStamped（目標姿態）
- `/unity/joint_commands` : sensor_msgs/JointState（關節命令）
- `/unity/gripper_command` : std_msgs/Float32（夾爪控制）

**發布（ROS 2 → Unity）**
- `/unity/heartbeat` : std_msgs/String（心跳訊號）
- `/openarm/joint_states` : sensor_msgs/JointState（當前關節狀態）
- `/openarm/end_effector_pose` : geometry_msgs/PoseStamped（末端執行器姿態）

### Services
- `/unity/ping` : example_interfaces/srv/Trigger（健康檢查）
- `/openarm/move_to_pose` : geometry_msgs/PoseStamped（姿態控制）
- `/openarm/home_position` : std_srvs/srv/Empty（回到初始位置）

### Parameters
- `tcp_ip`: "127.0.0.1"（TCP 伺服器 IP）
- `tcp_port`: 10000（TCP 埠號）
- `heartbeat_rate`: 1.0（心跳頻率 Hz）
- `openarm_config_file`: "config/openarm_config.yaml"

## 刪除檔案清單

### 說明文件（保留 README.md）
- [ ] `CHECK_REPO.md`
- [ ] `FIX_403_ERROR.md`
- [ ] `GITHUB_AUTH.md`
- [ ] `MIGRATE_TO_HUMBLE.md`
- [ ] `PUSH_TO_GITHUB.md`
- [ ] `QUICK_START.md`
- [ ] `SWITCH_TO_HUMBLE.md`

### 腳本檔案
- [ ] `backup_to_windows.sh`
- [ ] `diagnose_unity_connection.sh`
- [ ] `export_code.sh`
- [ ] `fix_port_conflict.sh`
- [ ] `import_to_humble.sh`
- [ ] `rebuild_docker.sh`
- [ ] `setup-env.sh`
- [ ] `test_connection.sh`
- [ ] `test_from_windows.ps1`
- [ ] `test_unity_connection.sh`

### 啟動檔案
- [ ] `start-humble.bat`
- [ ] `start-humble.ps1`

### 舊套件檔案（unity_bridge_py 內）
- [ ] `chatter_publisher.py`
- [ ] `chatter_subscriber.py`
- [ ] `cmd_subscriber.py`
- [ ] `cmd_vel_subscriber.py`
- [ ] `connection_monitor.py`
- [ ] `simple_unity_bridge.py`
- [ ] `status_publisher.py`

### 多餘配置
- [ ] `docker-compose.yml`（保留 humble 版本）
- [ ] `docker-compose-humble-windows.yml`
- [ ] `docker-entrypoint.sh`（保留 humble 版本）
- [ ] `windows_cyclonedx_with_ports.xml`

## 驗收清單

### 基礎功能
- [ ] ROS-TCP-Endpoint 成功啟動（監聽 :10000）
- [ ] unity_openarm_bridge 建置成功
- [ ] Unity 可連接 127.0.0.1:10000
- [ ] 心跳訊號正常發送
- [ ] Ping/Pong 服務正常

### OpenArm 整合
- [ ] OpenArm 描述檔案載入成功
- [ ] 關節狀態發布正常
- [ ] Unity 關節命令接收正常
- [ ] 姿態控制服務正常
- [ ] 模擬環境運行正常

### 系統整合
- [ ] 一鍵啟動 Launch 檔案正常
- [ ] Docker 容器環境正常
- [ ] Cursor 開發環境正常

## 後續擴充計畫

### Phase 5: 進階功能
- [ ] MoveIt2 整合（軌跡規劃）
- [ ] 力回饋控制
- [ ] 碰撞檢測
- [ ] 安全限制

### Phase 6: 實體硬體
- [ ] CAN 通訊整合
- [ ] 硬體安全檢查
- [ ] 校準程序
- [ ] 遙操作介面

### Phase 7: AI 整合
- [ ] 模仿學習資料收集
- [ ] 強化學習環境
- [ ] Isaac Lab 模擬整合

## 開發時程

**Week 1**: Phase 1-2（清理 + 安裝依賴）
**Week 2**: Phase 3（建立新橋接套件）
**Week 3**: Phase 4（整合測試）
**Week 4**: 文件整理與優化

## 風險與應對

**風險1**: OpenArm 套件相容性問題
**應對**: 先用模擬環境測試，逐步整合實體硬體

**風險2**: TCP 連接穩定性
**應對**: 加入重連機制和錯誤處理

**風險3**: 效能瓶頸
**應對**: 使用適當的 QoS 設定和訊息頻率控制
