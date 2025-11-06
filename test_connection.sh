#!/bin/bash
# 連線測試腳本

source /root/ros2_ws/setup-env.sh

echo "=== ROS 2 連線測試 ==="
echo ""
echo "1. 環境變數檢查:"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "   CYCLONEDDS_URI: $CYCLONEDDS_URI"
echo ""
echo "2. 容器 IP:"
hostname -I | awk '{print "   " $1}'
echo ""
echo "3. 等待 DDS 發現（15 秒）..."
sleep 15
echo ""
echo "4. 檢查話題:"
ros2 topic list
echo ""
echo "5. 檢查 /chatter 話題:"
ros2 topic info /chatter 2>&1 || echo "   ❌ 話題尚未出現"
echo ""
echo "6. 檢查節點:"
ros2 node list
echo ""
echo "=== 測試完成 ==="

