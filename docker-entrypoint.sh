#!/bin/bash

# ROS 2 Jazzy Docker Entrypoint
# 自動載入 ROS 2 環境並執行後續命令

# 載入 ROS 2 環境
source /opt/ros/jazzy/setup.bash

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

