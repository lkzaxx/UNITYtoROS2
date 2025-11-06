#!/bin/bash

source /root/ros2_ws/setup-env.sh

echo ""
echo "============================================================"
echo "🔍 Unity ↔ ROS2 連線測試"
echo "============================================================"
echo ""
echo "啟動連線監控器..."
echo "（按 Ctrl+C 停止）"
echo ""
echo "============================================================"
echo ""

ros2 run unity_bridge_py connection_monitor

