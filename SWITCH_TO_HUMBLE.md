# 切換到 ROS2 Humble 版本指南

Unity的ROS2套件可能與最新的Jazzy版本不兼容。Humble是LTS版本，兼容性更好。

## 🔄 切換方法

### 方法 1：使用新的 docker-compose 文件（推薦）

```bash
# 停止當前 Jazzy 容器
docker stop ros2_jazzy

# 使用 Humble 版本啟動
docker-compose -f docker-compose-humble.yml up -d

# 進入 Humble 容器
docker exec -it ros2_humble bash
```

### 方法 2：直接運行 Humble 容器

```bash
# 停止當前容器
docker stop ros2_jazzy

# 直接運行 Humble 容器
docker run -it --rm \
  --name ros2_humble \
  -p 7400:7400/udp \
  -p 7410-7420:7410-7420/udp \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///etc/cyclonedds.xml \
  -e ROS_DOMAIN_ID=0 \
  -v $(pwd):/root/ros2_ws \
  -v $(pwd)/cyclonedds.xml:/etc/cyclonedds.xml \
  osrf/ros:humble-desktop-full bash
```

### 方法 3：修改現有 docker-compose.yml

```bash
# 編輯 docker-compose.yml
nano docker-compose.yml

# 將第3行改為：
# image: osrf/ros:humble-desktop-full

# 將第4行改為：
# container_name: ros2_humble

# 重新啟動
docker-compose down
docker-compose up -d
```

## 🔧 進入容器後的設置

```bash
# 載入環境
source /root/ros2_ws/setup-env.sh

# 檢查版本
echo "ROS2版本: $ROS_DISTRO"

# 重新構建你的包
cd /root/ros2_ws
colcon build --packages-select unity_bridge_py

# 測試橋接器
ros2 run unity_bridge_py simple_unity_bridge
```

## 📊 版本差異

| 特性 | Jazzy (當前) | Humble (建議) |
|------|-------------|---------------|
| 發布時間 | 2024年5月 | 2022年5月 |
| 支援期限 | 2025年5月 | 2027年5月 (LTS) |
| Unity兼容性 | 可能不兼容 | ✅ 良好兼容 |
| 穩定性 | 最新功能 | ✅ 穩定成熟 |

## ⚠️ 注意事項

1. **備份當前工作**：切換前確保代碼已保存
2. **重新構建**：Humble版本需要重新構建所有包
3. **測試連線**：切換後測試Unity連線是否正常
4. **依賴檢查**：某些新功能可能在Humble中不可用

## 🎯 推薦流程

1. 停止當前Jazzy容器
2. 使用 `docker-compose-humble.yml` 啟動Humble容器
3. 重新構建unity_bridge_py包
4. 測試Unity連線
5. 如果連線成功，可以刪除Jazzy相關配置

## 🔍 驗證連線

切換到Humble後，測試連線：

```bash
# 啟動橋接器
ros2 run unity_bridge_py simple_unity_bridge

# 輸入 start 開始發送座標
start

# 檢查Unity是否能接收到座標
```

如果Unity能正常接收座標，說明版本兼容性問題已解決！
