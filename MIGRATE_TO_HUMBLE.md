# 程式碼轉移到 Humble 容器指南

## 📋 方法 1：使用 Volume 掛載（最簡單，如果文件在主機上）

如果你的 `docker-compose.yml` 使用了 `./:/root/ros2_ws` 掛載，那麼文件已經在主機上了！

### 步驟：

```bash
# 1. 在 Windows/WSL2 中找到 ros2_ws 目錄
# 通常位置：C:\Users\你的用戶名\ros2_ws 或 WSL2 中的 /mnt/c/...

# 2. 在同一目錄下創建 docker-compose-humble.yml（使用相同volume掛載）
# 3. 啟動 Humble 容器，文件會自動出現在 /root/ros2_ws
```

## 📦 方法 2：從容器導出備份（推薦）

### 步驟 A：在當前 Jazzy 容器中導出

```bash
# 在當前容器中執行
cd /root/ros2_ws
chmod +x export_code.sh
./export_code.sh
```

這會創建備份在 `/tmp/ros2_ws_export/ros2_ws_full_backup.tar.gz`

### 步驟 B：複製到 Windows

```powershell
# 在 Windows PowerShell 中執行
docker cp ros2_jazzy:/tmp/ros2_ws_export/ros2_ws_full_backup.tar.gz C:\Users\lkzax\ros2_ws_backup.tar.gz
```

### 步驟 C：在新 Humble 容器中導入

```bash
# 啟動 Humble 容器後
# 先複製備份檔案到容器
docker cp C:\Users\lkzax\ros2_ws_backup.tar.gz ros2_humble:/tmp/

# 進入容器
docker exec -it ros2_humble bash

# 在容器中執行
cd /root/ros2_ws
tar -xzf /tmp/ros2_ws_backup.tar.gz

# 重新構建
source /root/ros2_ws/setup-env.sh
colcon build --packages-select unity_bridge_py
```

## 🔄 方法 3：手動複製關鍵文件

### 需要轉移的文件列表：

1. **源碼文件：**
   - `src/unity_bridge_py/unity_bridge_py/simple_unity_bridge.py` ⭐ **最重要**
   - `src/unity_bridge_py/setup.py`
   - `src/unity_bridge_py/unity_bridge_py/connection_monitor.py`
   - `src/unity_bridge_py/unity_bridge_py/chatter_*.py`

2. **配置文件：**
   - `cyclonedds.xml`
   - `setup-env.sh`
   - `docker-entrypoint.sh`
   - `docker-compose*.yml`

3. **文檔：**
   - `QUICK_START.md`
   - `SWITCH_TO_HUMBLE.md`

### 手動複製命令：

```powershell
# 從Jazzy容器複製源碼目錄
docker cp ros2_jazzy:/root/ros2_ws/src C:\Users\lkzax\ros2_ws\

# 複製配置文件
docker cp ros2_jazzy:/root/ros2_ws/cyclonedds.xml C:\Users\lkzax\ros2_ws\
docker cp ros2_jazzy:/root/ros2_ws/setup-env.sh C:\Users\lkzax\ros2_ws\

# 複製到Humble容器
docker cp C:\Users\lkzax\ros2_ws\src ros2_humble:/root/ros2_ws/
docker cp C:\Users\lkzax\ros2_ws\cyclonedds.xml ros2_humble:/root/ros2_ws/
```

## 🎯 快速轉移腳本（一鍵完成）

### 在當前容器中執行：

```bash
# 創建完整的tar備份
cd /root/ros2_ws
tar -czf /tmp/ros2_ws_backup.tar.gz \
    --exclude='build' \
    --exclude='install' \
    --exclude='log' \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    src/ *.yml *.xml *.sh *.md

# 顯示備份位置
echo "備份創建在: /tmp/ros2_ws_backup.tar.gz"
echo "檔案大小:"
ls -lh /tmp/ros2_ws_backup.tar.gz
```

### 在 Windows 中執行：

```powershell
# 1. 停止舊容器（如果還在運行）
docker stop ros2_jazzy

# 2. 複製備份到Windows
docker cp ros2_jazzy:/tmp/ros2_ws_backup.tar.gz C:\Users\lkzax\ros2_ws_backup.tar.gz

# 3. 啟動Humble容器（使用docker run）
docker run -it --rm --name ros2_humble -p 7400:7400/udp -p 7410-7420:7410-7420/udp -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI=file:///etc/cyclonedds.xml -e ROS_DOMAIN_ID=0 osrf/ros:humble-desktop-full bash

# 4. 在另一個終端複製備份到容器
docker cp C:\Users\lkzax\ros2_ws_backup.tar.gz ros2_humble:/tmp/

# 5. 在容器中解壓
# （在容器內執行）
cd /root/ros2_ws
tar -xzf /tmp/ros2_ws_backup.tar.gz
```

## ✅ 驗證轉移成功

```bash
# 在Humble容器中檢查
ls -la /root/ros2_ws/src/unity_bridge_py/unity_bridge_py/simple_unity_bridge.py

# 應該看到文件存在
# 重新構建
source /root/ros2_ws/setup-env.sh
colcon build --packages-select unity_bridge_py

# 測試
ros2 run unity_bridge_py simple_unity_bridge
```

## 💡 最佳實踐建議

1. **使用 Volume 掛載**：如果可能，將 ros2_ws 放在主機目錄，使用 volume 掛載，這樣新容器可以直接訪問
2. **定期備份**：使用 git 版本控制你的代碼
3. **保留備份**：轉移後保留備份檔案，以防萬一

