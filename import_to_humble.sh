#!/bin/bash
# 在新Humble容器中導入程式碼

BACKUP_FILE="$1"

if [ -z "$BACKUP_FILE" ]; then
    echo "❌ 錯誤：請指定備份檔案路徑"
    echo "使用方法: $0 /path/to/ros2_ws_full_backup.tar.gz"
    exit 1
fi

if [ ! -f "$BACKUP_FILE" ]; then
    echo "❌ 錯誤：找不到檔案 $BACKUP_FILE"
    exit 1
fi

echo "📦 開始導入程式碼..."
echo ""

# 解壓到當前目錄
echo "📂 解壓檔案..."
tar -xzf "$BACKUP_FILE" -C /root/ros2_ws

echo ""
echo "✅ 導入完成！"
echo ""
echo "🔧 下一步："
echo "   1. 重新構建包："
echo "      source /root/ros2_ws/setup-env.sh"
echo "      colcon build --packages-select unity_bridge_py"
echo ""
echo "   2. 測試橋接器："
echo "      ros2 run unity_bridge_py simple_unity_bridge"

