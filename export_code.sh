#!/bin/bash
# 導出所有程式碼和配置到 tar 檔案

echo "📦 開始導出程式碼..."
echo ""

# 創建導出目錄
EXPORT_DIR="/tmp/ros2_ws_export"
mkdir -p "$EXPORT_DIR"

# 導出源碼目錄
echo "📂 導出 src 目錄..."
tar -czf "$EXPORT_DIR/src.tar.gz" -C /root/ros2_ws src/

# 導出配置文件
echo "⚙️ 導出配置文件..."
cp /root/ros2_ws/docker-compose*.yml "$EXPORT_DIR/" 2>/dev/null
cp /root/ros2_ws/cyclonedds.xml "$EXPORT_DIR/" 2>/dev/null
cp /root/ros2_ws/setup-env.sh "$EXPORT_DIR/" 2>/dev/null
cp /root/ros2_ws/docker-entrypoint.sh "$EXPORT_DIR/" 2>/dev/null

# 導出文檔
echo "📄 導出文檔..."
cp /root/ros2_ws/*.md "$EXPORT_DIR/" 2>/dev/null

# 創建完整備份
echo "💾 創建完整備份..."
cd /root/ros2_ws
tar -czf "$EXPORT_DIR/ros2_ws_full_backup.tar.gz" \
    --exclude='build' \
    --exclude='install' \
    --exclude='log' \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    src/ \
    *.yml \
    *.xml \
    *.sh \
    *.md

echo ""
echo "✅ 導出完成！"
echo ""
echo "📁 檔案位置："
echo "   $EXPORT_DIR/ros2_ws_full_backup.tar.gz"
echo ""
echo "📋 檔案列表："
ls -lh "$EXPORT_DIR"
echo ""
echo "💡 下一步："
echo "   1. 從容器複製到Windows："
echo "      docker cp ros2_jazzy:$EXPORT_DIR/ros2_ws_full_backup.tar.gz ."
echo ""
echo "   2. 在新Humble容器中解壓："
echo "      tar -xzf ros2_ws_full_backup.tar.gz -C /root/ros2_ws"

