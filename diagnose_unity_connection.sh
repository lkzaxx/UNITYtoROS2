#!/bin/bash

echo "=========================================="
echo "Unity ↔ ROS2 連線診斷工具"
echo "=========================================="
echo ""

source /root/ros2_ws/setup-env.sh

echo "1. ROS 2 環境檢查:"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "   CYCLONEDDS_URI: $CYCLONEDDS_URI"
echo ""

echo "2. 容器網路資訊:"
echo "   容器 IP: $(hostname -I | awk '{print $1}')"
echo ""

echo "3. CycloneDDS 配置:"
cat /etc/cyclonedds.xml
echo ""

echo "4. 當前話題列表:"
ros2 topic list
echo ""

echo "5. 檢查 /chatter 話題:"
ros2 topic info /chatter 2>&1 || echo "   /chatter 話題不存在"
echo ""

echo "6. 檢查 /cmd_vel 話題:"
ros2 topic info /cmd_vel 2>&1 || echo "   /cmd_vel 話題不存在"
echo ""

echo "7. 當前節點列表:"
ros2 node list
echo ""

echo "=========================================="
echo "診斷完成"
echo "=========================================="
echo ""
echo "如果 Unity 已連接，應該看到:"
echo "- /chatter 話題的 Publisher count > 0"
echo "- /cmd_vel 話題的 Publisher count > 0"
echo "- Unity 節點出現在節點列表中"
echo ""
echo "如果沒有看到以上項目，請檢查:"
echo "1. Unity 環境變數是否正確設定"
echo "2. Windows 端 cyclonedds.xml 是否包含容器 IP"
echo "3. Unity Console 是否有錯誤訊息"
echo "4. Unity 場景是否正在運行"
