#!/bin/bash

# ROS2 Unity TCP Bridge 啟動腳本
# 使用方法: ./start_ros2_unity.sh

echo "🚀 啟動 ROS2 Unity TCP Bridge..."

# 顏色定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 檢查 Docker 是否安裝
if ! command -v docker &> /dev/null; then
    echo -e "${RED}❌ Docker 未安裝！${NC}"
    echo "請先安裝 Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

# 檢查 Docker Compose 是否安裝
if ! command -v docker-compose &> /dev/null; then
    echo -e "${YELLOW}⚠️ docker-compose 未安裝，嘗試使用 docker compose...${NC}"
    COMPOSE_CMD="docker compose"
else
    COMPOSE_CMD="docker-compose"
fi

# 創建必要的目錄
echo "📁 創建必要的目錄..."
mkdir -p ros2_ws/src
mkdir -p scripts

# 複製橋接節點腳本
if [ -f "unity_tcp_bridge.py" ]; then
    echo "📋 複製橋接節點腳本..."
    cp unity_tcp_bridge.py scripts/
    chmod +x scripts/unity_tcp_bridge.py
else
    echo -e "${YELLOW}⚠️ 找不到 unity_tcp_bridge.py，請確保檔案存在${NC}"
fi

# 停止舊的容器（如果存在）
echo "🔄 停止舊的容器..."
$COMPOSE_CMD down 2>/dev/null

# 啟動容器
echo "🐳 啟動 Docker 容器..."
$COMPOSE_CMD up -d

# 等待服務啟動
echo "⏳ 等待服務啟動..."
sleep 5

# 檢查服務狀態
echo ""
echo "📊 檢查服務狀態..."
echo "================================"

# 檢查 TCP Endpoint
if docker exec unity_ros2_tcp bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" &>/dev/null; then
    echo -e "${GREEN}✅ ROS2 TCP Endpoint 運行正常${NC}"
    
    # 列出可用的 topics
    echo ""
    echo "📋 可用的 ROS2 Topics:"
    docker exec unity_ros2_tcp bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" | grep -E "(unity|openarm|cmd_vel)"
else
    echo -e "${RED}❌ ROS2 TCP Endpoint 啟動失敗${NC}"
fi

echo ""
echo "================================"
echo -e "${GREEN}✅ ROS2 Unity Bridge 啟動完成！${NC}"
echo ""
echo "🔍 測試命令："
echo "  1. 查看 Unity 心跳："
echo "     docker exec -it unity_ros2_tcp bash -c \"source /opt/ros/humble/setup.bash && ros2 topic echo /unity/heartbeat\""
echo ""
echo "  2. 查看關節狀態："
echo "     docker exec -it unity_ros2_tcp bash -c \"source /opt/ros/humble/setup.bash && ros2 topic echo /openarm/joint_states\""
echo ""
echo "  3. 查看容器日誌："
echo "     docker logs -f unity_ros2_tcp"
echo ""
echo "  4. 進入工具容器："
echo "     docker exec -it ros2_tools bash"
echo ""
echo "  5. 停止所有服務："
echo "     $COMPOSE_CMD down"
echo ""
echo "📡 Unity 連接設定："
echo "  IP: 127.0.0.1"
echo "  Port: 10000"
echo ""