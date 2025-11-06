#!/bin/bash

echo "=========================================="
echo "修復埠號衝突問題"
echo "=========================================="
echo ""

echo "1. 停止所有相關容器..."
docker-compose down 2>/dev/null || true

echo ""
echo "2. 強制移除所有 ros2_jazzy 容器..."
docker rm -f $(docker ps -aq --filter "name=ros2_jazzy") 2>/dev/null || true

echo ""
echo "3. 清理未使用的網路..."
docker network prune -f

echo ""
echo "4. 檢查埠號 7400-7420 是否被佔用..."
# 在 Windows/WSL2 中檢查埠號
echo "   請在 Windows PowerShell 中執行:"
echo "   Get-NetTCPConnection | Where-Object {$_.LocalPort -ge 7400 -and $_.LocalPort -le 7420}"
echo "   Get-NetUDPEndpoint | Where-Object {$_.LocalPort -ge 7400 -and $_.LocalPort -le 7420}"

echo ""
echo "5. 重新啟動容器（使用縮小的埠號範圍）..."
docker-compose up -d

echo ""
echo "6. 等待容器啟動（5 秒）..."
sleep 5

echo ""
echo "7. 檢查容器狀態..."
docker ps | grep ros2_jazzy

echo ""
echo "8. 取得容器 IP..."
NEW_IP=$(docker inspect ros2_jazzy 2>/dev/null | grep -A 10 "NetworkSettings" | grep "IPAddress" | head -1 | awk '{print $2}' | tr -d '",')
if [ ! -z "$NEW_IP" ] && [ "$NEW_IP" != "null" ]; then
    echo "   新容器 IP: $NEW_IP"
    echo ""
    echo "=========================================="
    echo "下一步："
    echo "1. 從 Windows ping: ping $NEW_IP"
    echo "2. 更新 Windows 端 cyclonedds.xml 中的 Peer address 為: $NEW_IP"
    echo "3. 重啟 Unity 場景"
    echo "=========================================="
else
    echo "   無法取得容器 IP，請檢查容器日誌:"
    echo "   docker logs ros2_jazzy"
fi

