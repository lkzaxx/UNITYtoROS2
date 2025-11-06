#!/bin/bash
# 臨時環境設定腳本（在容器內使用）
# 用於手動載入 ROS 2 環境

source /opt/ros/jazzy/setup.bash

# 載入工作區環境（如果已建置）
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

# 設定 CycloneDDS 配置路徑
export CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# 確保配置文件存在
if [ ! -f /etc/cyclonedds.xml ]; then
    cp /root/ros2_ws/cyclonedds.xml /etc/cyclonedds.xml
fi

echo "ROS 2 環境已載入！"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI: $CYCLONEDDS_URI"

