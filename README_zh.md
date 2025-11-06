# ROS 2 Jazzy × Unity 整合專案

本專案提供一個在 WSL2 的 Docker 中運行的 ROS 2 Jazzy 環境，設計用於與 Unity 使用 [ros2-for-unity](https://github.com/RobotecAI/ros2-for-unity) 橋接器進行通訊。

## 架構

- **Windows 11**: 運行 Unity 與 ros2-for-unity 的主機系統
- **WSL2 Ubuntu 24.04**: Docker 主機
- **Docker 容器**: ROS 2 Jazzy desktop-full 環境
- **通訊**: DDS (Cyclone DDS) 用於低延遲雙向通訊

## 快速開始

### 前置需求
- 啟用 WSL2 的 Windows 11
- 具備 WSL2 整合的 Docker Desktop
- Ubuntu 24.04 WSL 發行版

### 設定

1. **克隆/導航到工作區:**
   ```bash
   cd ~/ros2_ws
   ```

2. **使用 Docker Compose 啟動:**
   ```bash
   docker-compose up -d
   docker exec -it ros2_jazzy bash
   ```

3. **建置工作區 (在容器內):**
   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

### 測試通訊

1. **啟動狀態發布器:**
   ```bash
   ros2 run unity_bridge_py status_publisher
   ```

2. **在另一個終端機中測試訂閱器:**
   ```bash
   ros2 topic echo /unity/status
   ```

3. **測試命令發布器:**
   ```bash
   ros2 topic pub /unity/cmd std_msgs/String 'data: "move forward"' --once
   ```

4. **啟動命令訂閱器:**
   ```bash
   ros2 run unity_bridge_py cmd_subscriber
   ```

## 主題

- `/unity/status`: 由 ROS 2 發布，Unity 訂閱 (狀態更新)
- `/unity/cmd`: 由 Unity 發布，ROS 2 訂閱 (命令)

## 使用 Cursor 開發

1. 在 WSL 中開啟 Cursor: `cursor ~/ros2_ws`
2. 使用 "Reopen in Container" 在 ROS 2 環境內工作
3. `.devcontainer/devcontainer.json` 已配置為無縫開發

## 配置

- **DDS**: 啟用多播的 Cyclone DDS
- **域 ID**: 0 (確保 Unity 使用相同的 ID)
- **網路**: 主機網路以獲得最佳性能

## Unity 整合

在 Unity 中設定 ros2-for-unity 時:
1. 確保 Unity 環境中設定 `ROS_DOMAIN_ID=0`
2. 使用相同的 DDS 實作 (建議使用 Cyclone DDS)
3. 訂閱 `/unity/status` 並發布到 `/unity/cmd`

## 套件結構

```
ros2_ws/
├── src/
│   └── unity_bridge_py/          # Python 橋接套件
│       ├── unity_bridge_py/
│       │   ├── status_publisher.py    # 向 Unity 發布狀態
│       │   └── cmd_subscriber.py      # 接收來自 Unity 的命令
│       └── setup.py
├── .devcontainer/
│   └── devcontainer.json         # Cursor 開發容器配置
├── docker-compose.yml            # Docker 設定
├── cyclonedds.xml               # DDS 配置
└── README.md
```

## 故障排除

1. **看不到主題**: 確保節點正在運行且 DDS 配置匹配
2. **GUI 無法運作**: 驗證 WSLg 已啟用且 X11 轉發已配置
3. **Unity 無法連接**: 檢查防火牆設定和 ROS_DOMAIN_ID 一致性

## 下一步

- 在 `unity_bridge_msgs` 套件中添加自定義訊息類型
- 與 MoveIt 2 整合進行機器人控制
- 添加感測器模擬 (LiDAR、攝影機)
- 實作 Nav2 整合進行導航
