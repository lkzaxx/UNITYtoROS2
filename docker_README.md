# Unity-OpenArm ROS2 環境使用指南

## 📋 概述

這個 Docker Compose 配置會自動：
1. ✅ 安裝所有必要的依賴
2. ✅ 克隆 ROS-TCP-Endpoint 套件
3. ✅ 編譯 unity_openarm_bridge 和 ros_tcp_endpoint
4. ✅ 自動啟動 TCP Endpoint 和 Unity Bridge 服務
5. ✅ 提供工具容器用於調試和監控

## 🚀 快速開始

### 1. 啟動服務

**方法一：使用腳本（推薦）**
```cmd
start_compose.bat
```

**方法二：手動啟動**
```cmd
docker-compose -f docker-compose-humble.yml up -d
```

### 2. 查看日誌（確認啟動成功）

```cmd
# 查看實時日誌
docker-compose -f docker-compose-humble.yml logs -f

# 或使用腳本
view_status.bat
```

### 3. 等待服務初始化

首次啟動需要 30-60 秒來編譯套件。查看日誌中的：
```
✅ 所有服務已啟動！
📡 服務狀態:
   • TCP Endpoint: 監聽 0.0.0.0:10000
   • Unity Bridge: 運行中
```

### 4. 連接 Unity

在 Unity 的 ROS Settings 中配置：
- **ROS IP**: `127.0.0.1` 或 `localhost`
- **ROS Port**: `10000`
- **Protocol**: `ROS2`

## 📝 常用命令

### 服務管理

```cmd
# 啟動服務
start_compose.bat

# 停止服務
stop_compose.bat

# 查看狀態
view_status.bat

# 重啟服務
docker-compose -f docker-compose-humble.yml restart

# 查看容器狀態
docker-compose -f docker-compose-humble.yml ps
```

### 進入容器

```cmd
# 進入主容器（運行服務的容器）
docker exec -it unity_ros2_tcp bash

# 進入工具容器（用於調試）
docker exec -it ros2_tools bash
```

### ROS2 調試命令

在工具容器中執行（`docker exec -it ros2_tools bash`）：

```bash
# 列出所有 ROS2 主題
ros2 topic list

# 監控 Unity 心跳
ros2 topic echo /unity/heartbeat

# 列出所有服務
ros2 service list

# 列出所有節點
ros2 node list

# 查看節點詳情
ros2 node info /unity_openarm_bridge

# 查看主題詳情
ros2 topic info /openarm/joint_states
```

## 🔧 故障排除

### 問題 1: 服務無法啟動

```cmd
# 查看詳細日誌
docker-compose -f docker-compose-humble.yml logs

# 檢查容器狀態
docker ps -a
```

### 問題 2: 端口被占用

```cmd
# 檢查端口 10000 是否被占用
netstat -ano | findstr :10000

# 找到進程 PID 並結束
taskkill /PID <PID> /F
```

### 問題 3: Unity 無法連接

1. 確認服務已啟動：
   ```cmd
   docker-compose -f docker-compose-humble.yml ps
   ```

2. 確認端口正在監聽：
   ```cmd
   netstat -ano | findstr :10000
   ```

3. 檢查日誌是否有錯誤：
   ```cmd
   docker-compose -f docker-compose-humble.yml logs unity_ros2_tcp
   ```

### 問題 4: 編譯失敗

```cmd
# 進入容器手動編譯
docker exec -it unity_ros2_tcp bash

# 在容器內執行
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ros_tcp_endpoint unity_openarm_bridge
```

### 問題 5: 重置環境

```cmd
# 完全清理並重新啟動
docker-compose -f docker-compose-humble.yml down
docker-compose -f docker-compose-humble.yml up -d --force-recreate
```

## 📊 服務架構

```
┌─────────────────────────────────────────┐
│  unity_ros2_tcp (主容器)                │
│  ┌───────────────────────────────────┐  │
│  │ ROS2 TCP Endpoint (Port: 10000)  │  │
│  └───────────────────────────────────┘  │
│  ┌───────────────────────────────────┐  │
│  │ Unity-OpenArm Bridge              │  │
│  └───────────────────────────────────┘  │
└─────────────────────────────────────────┘
           ↕
┌─────────────────────────────────────────┐
│  Unity Editor                            │
│  (ROS TCP Connector)                     │
└─────────────────────────────────────────┘
```

## 🔍 健康檢查

容器配置了健康檢查，會每 15 秒檢查一次：
- TCP Endpoint 進程是否運行
- Unity Bridge 進程是否運行

查看健康狀態：
```cmd
docker inspect unity_ros2_tcp | findstr "Health"
```

## 📦 包含的套件

- **ros_tcp_endpoint**: Unity ROS TCP 通信端點
- **unity_openarm_bridge**: Unity-OpenArm 橋接器
  - `tcp_bridge_node`: TCP 橋接節點
  - `openarm_controller`: OpenArm 控制器

## 🛠️ 高級配置

### 修改 ROS 域 ID

編輯 `docker-compose-humble.yml`：
```yaml
environment:
  - ROS_DOMAIN_ID=0  # 改成你需要的 ID
```

### 修改 TCP 端口

編輯 `docker-compose-humble.yml`，找到啟動命令並修改：
```bash
-p ROS_TCP_PORT:=10000  # 改成你需要的端口
```

### 添加更多套件

1. 將套件源碼放到 `./src/` 目錄
2. 編輯 `docker-compose-humble.yml`，在編譯命令中添加：
   ```bash
   colcon build --symlink-install \
     --packages-select ros_tcp_endpoint unity_openarm_bridge your_package
   ```

## 📚 相關資源

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)

## ❓ 需要幫助？

如果遇到問題：
1. 先查看日誌：`docker-compose -f docker-compose-humble.yml logs`
2. 檢查服務狀態：使用 `view_status.bat`
3. 嘗試重啟服務：`docker-compose -f docker-compose-humble.yml restart`

---

**版本**: 1.0.0  
**最後更新**: 2025-11-07
