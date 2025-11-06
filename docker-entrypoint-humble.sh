#!/bin/bash

# ROS 2 Humble Docker Entrypoint
# 自動載入 ROS 2 環境並執行後續命令

# 載入 ROS 2 環境
source /opt/ros/humble/setup.bash

# 檢查並安裝 Cyclone DDS（如果尚未安裝）
if ! dpkg -l | grep -q "ros-humble-rmw-cyclonedds-cpp"; then
    echo "安裝 Cyclone DDS..."
    apt update > /dev/null 2>&1
    apt install -y ros-humble-rmw-cyclonedds-cpp > /dev/null 2>&1
    echo "Cyclone DDS 安裝完成"
fi

# 如果工作區已建置，載入工作區環境
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

# 執行傳入的命令（如果有的話）
if [ $# -eq 0 ]; then
    # 沒有命令時，執行 bash
    exec bash
else
    # 有命令時，執行該命令
    exec "$@"
fi
