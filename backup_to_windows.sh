#!/bin/bash

# ROS2 工作區備份腳本
# 將容器內的工作區備份到 Windows 桌面

BACKUP_DIR="/mnt/c/Users/$USER/Desktop/ros2_backup_$(date +%Y%m%d_%H%M%S)"
SOURCE_DIR="/root/ros2_ws"

echo "開始備份 ROS2 工作區..."
echo "來源目錄: $SOURCE_DIR"
echo "目標目錄: $BACKUP_DIR"

# 建立備份目錄
mkdir -p "$BACKUP_DIR"

# 複製檔案
cp -r "$SOURCE_DIR"/* "$BACKUP_DIR/"

echo "備份完成！"
echo "備份位置: $BACKUP_DIR"

# 顯示備份內容
echo "備份內容:"
ls -la "$BACKUP_DIR"

