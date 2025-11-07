# Docker 使用指南

## 問題修正說明

### 原始配置的問題

1. **網路模式衝突**: 使用 `network_mode: host` 的同時定義了 `ports` 映射
2. **未使用的 Volumes**: 定義了 `ros2_install` 和 `ros2_build` 但從未使用

### 修正內容

已創建 `docker-compose-humble-fixed.yml`，修正了以上問題。

---

## 如何創建和啟動 Docker 容器

### 方法 1: 使用 Docker Compose（推薦）✅

#### Windows (PowerShell 或 CMD)

```powershell
# 進入專案目錄
cd W:\idaka\unity_ros2\ROS2

# 使用修正版配置啟動
docker-compose -f docker-compose-humble-fixed.yml up -d

# 或使用原始配置（有小問題但仍可運行）
docker-compose -f docker-compose-humble.yml up -d
```

#### WSL2 / Linux

```bash
# 進入專案目錄
cd /mnt/w/idaka/unity_ros2/ROS2

# 使用修正版配置啟動
docker-compose -f docker-compose-humble-fixed.yml up -d
```

---

### 方法 2: 使用提供的啟動腳本

#### 在 WSL2 / Linux 中執行

```bash
# 給予執行權限
chmod +x start_ros2_unity.sh

# 執行腳本
./start_ros2_unity.sh
```

**注意**: 這個腳本會使用原始的 `docker-compose-humble.yml`，建議修改腳本改用修正版。

---

### 方法 3: 使用 Docker Desktop GUI

如果您安裝了 Docker Desktop：

1. **下載映像**:
   - 打開 Docker Desktop
   - 點擊左側 "Images"
   - 搜尋 `osrf/ros`
   - 找到 `osrf/ros:humble-desktop`
   - 點擊 "Pull" 下載

2. **啟動容器**:
   - 方法 A: 在終端執行上述 docker-compose 命令
   - 方法 B: 在 Docker Desktop 中：
     - 點擊左側 "Containers"
     - 點擊右上角的 "Compose" 按鈕
     - 選擇 `docker-compose-humble-fixed.yml`
     - 點擊 "Start"

---

## 驗證容器是否正常運行

### 1. 檢查容器狀態

```powershell
# 列出運行中的容器
docker ps

# 應該看到兩個容器:
# - unity_ros2_tcp (主要服務)
# - ros2_tools (工具容器)
```

### 2. 查看容器日誌

```powershell
# 查看主服務日誌
docker logs unity_ros2_tcp

# 持續監控日誌
docker logs -f unity_ros2_tcp
```

### 3. 測試 ROS2 功能

```powershell
# 進入工具容器
docker exec -it ros2_tools bash

# 在容器內執行以下命令:
ros2 topic list
ros2 topic echo /unity/heartbeat
```

---

## 常用操作命令

### 啟動容器

```powershell
# 啟動（背景模式）
docker-compose -f docker-compose-humble-fixed.yml up -d

# 啟動（前景模式，可看到輸出）
docker-compose -f docker-compose-humble-fixed.yml up
```

### 停止容器

```powershell
# 停止所有服務
docker-compose -f docker-compose-humble-fixed.yml down

# 停止並刪除 volumes
docker-compose -f docker-compose-humble-fixed.yml down -v
```

### 重啟容器

```powershell
# 重啟特定容器
docker restart unity_ros2_tcp

# 重啟所有容器
docker-compose -f docker-compose-humble-fixed.yml restart
```

### 進入容器

```powershell
# 進入主服務容器
docker exec -it unity_ros2_tcp bash

# 進入工具容器
docker exec -it ros2_tools bash
```

### 查看容器資源使用

```powershell
# 查看 CPU、記憶體使用
docker stats unity_ros2_tcp ros2_tools
```

---

## 故障排除

### 問題 1: 容器無法啟動

**檢查步驟**:
```powershell
# 1. 檢查 Docker 是否運行
docker version

# 2. 查看詳細錯誤
docker-compose -f docker-compose-humble-fixed.yml logs

# 3. 檢查端口是否被占用
netstat -ano | findstr :10000
```

**解決方案**:
- 確保 Docker Desktop 正在運行
- 確保端口 10000 未被其他程序占用
- 檢查 WSL2 是否正常運行

### 問題 2: Unity 無法連接到容器

**檢查步驟**:
```powershell
# 1. 確認容器正在運行
docker ps | findstr unity_ros2_tcp

# 2. 檢查 TCP Endpoint 是否啟動
docker exec unity_ros2_tcp bash -c "ps aux | grep tcp_endpoint"

# 3. 測試端口連通性
# 在 PowerShell 中:
Test-NetConnection -ComputerName localhost -Port 10000
```

**Unity 連接設定**:
- IP: `127.0.0.1` 或 `localhost`
- Port: `10000`

### 問題 3: ROS2 topics 看不到

**檢查步驟**:
```powershell
# 進入容器
docker exec -it unity_ros2_tcp bash

# 在容器內:
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list
```

**解決方案**:
- 確保正確 source ROS2 環境
- 確保 RMW_IMPLEMENTATION 設定正確
- 檢查 ROS_DOMAIN_ID 是否一致

### 問題 4: 權限錯誤

**Windows 上的解決方案**:
```powershell
# 確保 Docker Desktop 有正確的檔案共享權限
# Settings > Resources > File Sharing
# 添加: W:\idaka\unity_ros2
```

---

## 建議使用哪個配置檔？

### 推薦: `docker-compose-humble-fixed.yml`

**原因**:
- ✅ 修正了網路模式衝突
- ✅ 移除了未使用的 volume 定義
- ✅ 更清晰的配置結構

### 如果使用原始版本

`docker-compose-humble.yml` 仍然可以運行，但有以下注意事項：
- ⚠️ ports 映射會被忽略（因為使用了 host 網路模式）
- ⚠️ 有未使用的 volume 定義（不影響功能）

---

## 下一步操作

### 1. 整合到啟動腳本

修改 `start_ros2_unity.sh`:

```bash
# 將第 49 行改為:
$COMPOSE_CMD -f docker-compose-humble-fixed.yml up -d
```

### 2. 創建 Windows 批次檔

創建 `start_docker.bat`:

```batch
@echo off
echo 啟動 ROS2 Docker 容器...
docker-compose -f docker-compose-humble-fixed.yml up -d
echo.
echo 容器已啟動！
echo 查看日誌: docker logs -f unity_ros2_tcp
pause
```

### 3. 添加到 Git

建議將修正版本加入版本控制：

```bash
git add docker-compose-humble-fixed.yml
git commit -m "Add fixed Docker Compose configuration"
```

---

## 總結

**建議流程**:

1. ✅ 使用 `docker-compose-humble-fixed.yml` 啟動容器
2. ✅ 驗證容器運行: `docker ps`
3. ✅ 測試 ROS2 功能: `docker exec -it ros2_tools bash`
4. ✅ 在 Unity 中設定連接: `127.0.0.1:10000`
5. ✅ 測試 Unity 與 ROS2 的通訊

**需要幫助時**:
- 查看容器日誌: `docker logs -f unity_ros2_tcp`
- 檢查 ROS2 topics: `docker exec -it ros2_tools ros2 topic list`
- 進入容器除錯: `docker exec -it unity_ros2_tcp bash`
