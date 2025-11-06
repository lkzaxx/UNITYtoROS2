#!/bin/bash

echo "=========================================="
echo "重建 Docker 容器"
echo "=========================================="
echo ""

echo "1. 停止並移除現有容器..."
docker-compose down
# 強制移除可能殘留的容器
docker rm -f ros2_jazzy 2>/dev/null || true
# 清理所有相關容器
docker rm -f $(docker ps -aq --filter "name=ros2_jazzy") 2>/dev/null || true
# 清理未使用的網路
docker network prune -f

echo ""
echo "2. 重建並啟動容器..."
docker-compose up -d

echo ""
echo "3. 等待容器啟動（5 秒）..."
sleep 5

echo ""
echo "4. 檢查容器狀態..."
docker ps | grep ros2_jazzy

echo ""
echo "5. 取得新的容器 IP..."
NEW_IP=$(docker inspect ros2_jazzy 2>/dev/null | grep -A 10 "NetworkSettings" | grep "IPAddress" | head -1 | awk '{print $2}' | tr -d '",')
if [ ! -z "$NEW_IP" ]; then
    echo "   新容器 IP: $NEW_IP"
    echo ""
    echo "=========================================="
    echo "下一步："
    echo "1. 從 Windows ping: ping $NEW_IP"
    echo "2. 更新 Windows 端 cyclonedds.xml 中的 Peer address 為: $NEW_IP"
    echo "3. 重啟 Unity 場景"
    echo "=========================================="
else
    echo "   無法取得容器 IP，請手動檢查"
fi

echo ""
echo "6. 進入容器檢查..."
echo "   執行: docker exec -it ros2_jazzy bash"
echo "   然後執行: hostname -I"

