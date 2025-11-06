# Unity ↔ ROS 2 快速啟動指南

本文件提供 Unity 與 ROS 2 連線的快速啟動和測試指令。

## 📋 目錄

- [環境準備](#環境準備)
- [啟動 ROS 2 端](#啟動-ros-2-端)
- [測試連線](#測試連線)
- [常用指令](#常用指令)
- [故障排除](#故障排除)

---

## 🚀 環境準備

### 1. 載入 ROS 2 環境

```bash
source /root/ros2_ws/setup-env.sh
```

### 2. 檢查環境變數

```bash
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI: $CYCLONEDDS_URI"
```

應該顯示：
```
ROS_DOMAIN_ID: 0
RMW_IMPLEMENTATION: rmw_cyclonedds_cpp
CYCLONEDDS_URI: file:///etc/cyclonedds.xml
```

### 3. 檢查 CycloneDDS 配置

```bash
# 查看配置文件
cat /etc/cyclonedds.xml

# 檢查容器 IP
hostname -I | awk '{print $1}'
```

**當前配置狀態：**
- ✅ 配置文件位置：`/etc/cyclonedds.xml`
- ✅ 環境變數：`CYCLONEDDS_URI=file:///etc/cyclonedds.xml`
- ✅ 多播模式：已啟用 (`AllowMulticast=true`)
- ✅ Peer 設定：指向 Windows 主機 `192.168.65.1`
- ✅ 容器 IP：`192.168.65.6`（可能因 WSL2 重啟而變動）

---

## 🎯 啟動 ROS 2 端

### 方法 1：使用簡單橋接器（推薦 - 無自循環）

```bash
source /root/ros2_ws/setup-env.sh
ros2 run unity_bridge_py simple_unity_bridge
```

**輸出範例：**
```
================================================================================
🔗 Unity ↔ ROS2 簡單橋接器已啟動
================================================================================
功能說明：
  📨 自動接收來自 Unity 的訊息
  📤 發送隨機座標給 Unity
  📊 定期顯示連線狀態
  🚫 無自循環 - 過濾自己發送的訊息
================================================================================
```

**互動指令：**
- `start` - 開始持續發送座標 (每秒1次)
- `stop` - 停止持續發送座標
- `coord` - 手動發送一次座標 (X:0-100 Y:0-100 Z:0-100)
- `status` - 顯示當前狀態
- `quit` - 退出程式

**訊息顯示格式：**
```
🟢 開始持續發送座標 (每秒1次)
發送至unity訊息: X:87 Y:48 Z:28
----------------------------------------
發送至unity訊息: X:12 Y:95 Z:33
----------------------------------------
接收到unity訊息:
內容為: Hello from Unity!
----------------------------------------
```

### 方法 2：使用連線監控器（有自循環測試）

```bash
source /root/ros2_ws/setup-env.sh
ros2 run unity_bridge_py connection_monitor
```

**輸出範例：**
```
======================================================================
✅ Unity → ROS2 連線成功！
======================================================================

📨 [Unity → ROS2] 訊息 #1: Hello Ros2ForUnity!
📤 [ROS2 → Unity] 已發送: ROS2 → Unity 測試訊息 #1
```

**功能：**
- ✅ 自動檢測 Unity 連線狀態
- ✅ 顯示連線成功訊息
- ✅ 自動發送測試訊息給 Unity
- ✅ 每 5 秒顯示連線狀態報告

### 方法 2：使用改進的訂閱器

```bash
source /root/ros2_ws/setup-env.sh
ros2 run unity_bridge_py chatter_subscriber
```

```bash
source /root/ros2_ws/setup-env.sh
ros2 run unity_bridge_py connection_monitor
```

**輸出範例：**
```
============================================================
✅ Unity → ROS2 連線成功！
============================================================

[訊息 #1] Unity → ROS2: Hello Ros2ForUnity!
[訊息 #2] Unity → ROS2: Hello Ros2ForUnity!
```

### 方法 3：使用快速測試腳本

```bash
/root/ros2_ws/test_unity_connection.sh
```

---

## 📡 測試連線

### 測試 1：監聽 Unity 的訊息

```bash
source /root/ros2_ws/setup-env.sh
ros2 run unity_bridge_py chatter_subscriber
```

### 測試 2：發送訊息給 Unity

```bash
source /root/ros2_ws/setup-env.sh
ros2 topic pub /chatter std_msgs/String 'data: "Hello from ROS2!"' -r 1
```

或使用專用發布器：

```bash
source /root/ros2_ws/setup-env.sh
ros2 run unity_bridge_py chatter_publisher
```

### 測試 3：雙向通訊測試

**終端 1 - 啟動訂閱器（接收 Unity 訊息）：**
```bash
source /root/ros2_ws/setup-env.sh
ros2 run unity_bridge_py chatter_subscriber
```

**終端 2 - 啟動發布器（發送給 Unity）：**
```bash
source /root/ros2_ws/setup-env.sh
ros2 run unity_bridge_py chatter_publisher
```

**終端 3 - 檢查話題狀態：**
```bash
source /root/ros2_ws/setup-env.sh
ros2 topic list
ros2 topic info /chatter
```

---

## 🔧 常用指令

### 檢查話題列表

```bash
source /root/ros2_ws/setup-env.sh
ros2 topic list
```

### 檢查話題資訊

```bash
source /root/ros2_ws/setup-env.sh
ros2 topic info /chatter
```

**預期輸出（連線成功時）：**
```
Type: std_msgs/msg/String
Publisher count: 2
Subscription count: 2
```

### 監聽話題內容

```bash
source /root/ros2_ws/setup-env.sh
ros2 topic echo /chatter
```

### 檢查話題頻率

```bash
source /root/ros2_ws/setup-env.sh
ros2 topic hz /chatter
```

### 一次性發布測試訊息

```bash
source /root/ros2_ws/setup-env.sh
ros2 topic pub /chatter std_msgs/String 'data: "Test message"' --once
```

### 持續發布測試訊息

```bash
source /root/ros2_ws/setup-env.sh
# 每秒 1 次
ros2 topic pub /chatter std_msgs/String 'data: "Test"' -r 1

# 每秒 2 次
ros2 topic pub /chatter std_msgs/String 'data: "Test"' -r 2
```

### 檢查節點列表

```bash
source /root/ros2_ws/setup-env.sh
ros2 node list
```

### 檢查容器 IP

```bash
hostname -I | awk '{print $1}'
```

---

## 📦 可用的 ROS 2 節點

### 1. `simple_unity_bridge` - 簡單橋接器（推薦 - 無自循環）

```bash
ros2 run unity_bridge_py simple_unity_bridge
```

**功能：**
- 🚫 **無自循環** - 過濾自己發送的訊息，避免誤判連線
- 📨 自動接收並簡潔顯示來自 Unity 的訊息
- 📤 **持續發送隨機座標** (每秒1次) 或手動發送
- 📊 定期顯示連線狀態報告
- 🎮 互動式操作界面

**訊息格式：**
- 發送：`X:87 Y:48 Z:28` (隨機0-100範圍座標，每秒1次)
- 接收：簡潔顯示Unity發送的內容
- 控制：start/stop 控制持續發送

### 2. `connection_monitor` - 連線監控器（有自循環）

```bash
ros2 run unity_bridge_py connection_monitor
```

**功能：**
- 自動檢測 Unity 連線
- 顯示連線狀態報告
- ⚠️ **自動發送測試訊息**（會產生自循環）

### 3. `chatter_subscriber` - 訂閱器

```bash
ros2 run unity_bridge_py chatter_subscriber
```

**功能：**
- 訂閱 `/chatter` 話題
- 顯示收到的訊息
- 顯示連線成功提示

### 4. `chatter_publisher` - 發布器

```bash
ros2 run unity_bridge_py chatter_publisher
```

**功能：**
- 發布到 `/chatter` 話題
- 每 0.5 秒發布一次（2 Hz）

### 5. `status_publisher` - 狀態發布器

```bash
ros2 run unity_bridge_py status_publisher
```

**功能：**
- 發布到 `/unity/status` 話題
- 每 0.5 秒發布一次（2 Hz）

### 6. `cmd_subscriber` - 命令訂閱器

```bash
ros2 run unity_bridge_py cmd_subscriber
```

**功能：**
- 訂閱 `/unity/cmd` 話題
- 接收來自 Unity 的命令

---

## 🎮 Unity 端設定

### 環境變數設定

在 Unity 中設定以下環境變數：

```
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///C:/cyclonedds/cyclonedds.xml
```

### Windows 端 CycloneDDS 配置

文件位置：`C:/cyclonedds/cyclonedds.xml`

內容：
```xml
<CycloneDDS>
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <!-- 容器 IP（需要與容器端配置對應） -->
        <Peer address="192.168.65.6"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

**注意：** 容器 IP 可能因 WSL2 重啟而改變，請使用 `hostname -I` 檢查當前 IP。

---

## ⚙️ WSL2 容器端 CycloneDDS 配置

### 當前配置狀態

**配置文件位置：**
- 主要配置：`/etc/cyclonedds.xml`
- 備用配置：`/root/.cyclonedds.xml`（符號連結）

**環境變數（docker-compose.yml 中設定）：**
```yaml
environment:
  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  - CYCLONEDDS_URI=file:///etc/cyclonedds.xml
  - ROS_DOMAIN_ID=0
```

**配置文件內容：**
```xml
<CycloneDDS>
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <!-- Windows 主機 IP（WSL2 網關，備用方案） -->
        <Peer address="192.168.65.1"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

### 配置檢查命令

```bash
# 檢查環境變數
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI: $CYCLONEDDS_URI"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

# 查看配置文件
cat /etc/cyclonedds.xml

# 檢查容器 IP
hostname -I | awk '{print $1}'
```

### 配置對應關係

```
┌─────────────────┐         DDS 通訊         ┌─────────────────┐
│   Unity (Win)   │ ◄─────────────────────► │  ROS 2 (容器)   │
│  192.168.65.1   │                         │  192.168.65.6   │
└─────────────────┘                         └─────────────────┘
         │                                           │
   配置指向:                                   配置指向:
192.168.65.6 (容器)                         192.168.65.1 (Windows)
```

### 更新配置（如果需要）

如果需要修改配置：

```bash
# 編輯配置文件
nano /root/ros2_ws/cyclonedds.xml

# 複製到系統位置
cp /root/ros2_ws/cyclonedds.xml /etc/cyclonedds.xml

# 重新載入環境
source /root/ros2_ws/setup-env.sh
```

---

## ✅ 連線成功標誌

### ROS 2 端

- ✅ 看到 `Unity → ROS2 連線成功！` 訊息
- ✅ `chatter_subscriber` 收到 Unity 的訊息
- ✅ `ros2 topic list` 顯示 `/chatter` 話題
- ✅ `ros2 topic info /chatter` 顯示 `Publisher count: 2`

### Unity 端

- ✅ Unity Console 顯示 `heard: ROS2 → Unity 測試訊息 #X`
- ✅ Unity Console 顯示 `heard: Hello Ros2ForUnity!`（自循環）

---

## 🔍 故障排除

### 問題 0：Unity 無法連接到 ROS2（版本兼容性）

**症狀：**
- Unity 收不到 ROS2 的訊息
- ROS2 收不到 Unity 的訊息
- 所有配置都正確但就是無法通訊

**可能原因：**
- 當前使用 ROS2 Jazzy (最新版本)，Unity 的 ROS2 套件可能不兼容

**解決方案：切換到 ROS2 Humble (LTS版本)**

```bash
# 停止當前容器
docker stop ros2_jazzy

# 使用 Humble 版本
docker-compose -f docker-compose-humble.yml up -d
docker exec -it ros2_humble bash

# 重新構建包
source /root/ros2_ws/setup-env.sh
colcon build --packages-select unity_bridge_py

# 測試連線
ros2 run unity_bridge_py simple_unity_bridge
```

**詳細切換指南：** 參考 [SWITCH_TO_HUMBLE.md](SWITCH_TO_HUMBLE.md)

### 問題 1：看不到 `/chatter` 話題

**解決方案：**
```bash
# 1. 確認環境變數
source /root/ros2_ws/setup-env.sh

# 2. 等待 DDS 發現（10-15 秒）
sleep 15

# 3. 檢查話題
ros2 topic list
```

### 問題 2：收不到 Unity 的訊息

**檢查項目：**
1. Unity 場景是否正在運行
2. Unity 環境變數是否正確設定
3. Windows 端 `cyclonedds.xml` 是否包含容器 IP
4. Unity 場景是否已重啟（讓環境變數生效）

### 問題 3：Unity 收不到 ROS 2 的訊息

**檢查項目：**
1. ROS 2 端是否正在發布訊息
2. 檢查 `ros2 topic info /chatter` 的 `Publisher count`
3. Unity Console 是否有錯誤訊息

### 問題 4：容器 IP 變動

**解決方案：**
```bash
# 檢查當前 IP
hostname -I | awk '{print $1}'

# 更新 Windows 端的 cyclonedds.xml 中的 Peer address
```

---

## 📝 完整測試流程

### 步驟 1：啟動 ROS 2 端

```bash
source /root/ros2_ws/setup-env.sh
ros2 run unity_bridge_py connection_monitor
```

### 步驟 2：啟動 Unity 場景

在 Unity 編輯器中運行場景。

### 步驟 3：觀察連線狀態

在 ROS 2 端應該會看到：
```
======================================================================
✅ Unity → ROS2 連線成功！
======================================================================
```

在 Unity Console 應該會看到：
```
heard: ROS2 → Unity 測試訊息 #1
heard: ROS2 → Unity 測試訊息 #2
```

### 步驟 4：驗證雙向通訊

- ROS 2 端收到 Unity 的訊息 ✅
- Unity 端收到 ROS 2 的訊息 ✅

---

## 🎯 快速參考

### 最常用的指令

```bash
# 載入環境
source /root/ros2_ws/setup-env.sh

# 啟動連線監控器（推薦）
ros2 run unity_bridge_py connection_monitor

# 檢查話題
ros2 topic list

# 檢查話題資訊
ros2 topic info /chatter

# 監聽話題
ros2 topic echo /chatter
```

---

## 📚 相關文件

- [README.md](README.md) - 專案說明
- [README_zh.md](README_zh.md) - 中文說明
- [test_connection.sh](test_connection.sh) - 連線測試腳本
- [test_unity_connection.sh](test_unity_connection.sh) - Unity 連線測試腳本

---

## 💡 提示

1. **首次連線可能需要 10-15 秒**讓 DDS 發現機制完成
2. **容器 IP 可能變動**，WSL2 重啟後需重新檢查
3. **Unity 場景需要重啟**才能讓環境變數生效
4. **使用 `connection_monitor`** 是最直觀的連線狀態檢查方式

---

**最後更新：** 2024-11-04

