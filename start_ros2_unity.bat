@echo off
chcp 65001 >nul
cls

echo ════════════════════════════════════════════════
echo     ROS2 Unity TCP Bridge 快速啟動
echo ════════════════════════════════════════════════
echo.

REM 檢查 Docker
docker --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Docker 未安裝！請先安裝 Docker Desktop
    echo    下載: https://www.docker.com/products/docker-desktop
    pause
    exit /b 1
)

REM 檢查 Docker 是否運行
docker info >nul 2>&1
if errorlevel 1 (
    echo ⚠️  Docker Desktop 未啟動，請先啟動 Docker Desktop
    start "" "C:\Program Files\Docker\Docker\Docker Desktop.exe"
    echo.
    echo 等待 Docker 啟動後，再次執行此腳本
    pause
    exit /b 1
)

echo ✅ Docker 已就緒
echo.

REM 創建必要目錄
if not exist "src" mkdir src
if not exist "scripts" mkdir scripts

REM 停止舊容器
echo 🔄 清理舊容器...
docker-compose -f docker-compose-fixed.yml down 2>nul
docker rm -f unity_ros2_tcp ros2_tools 2>nul
echo.

REM 啟動新容器
echo 🚀 啟動 ROS2 Unity Bridge...
docker-compose -f docker-compose-fixed.yml up -d

if errorlevel 1 (
    echo ❌ 容器啟動失敗！
    echo.
    echo 查看錯誤詳情：
    docker-compose -f docker-compose-fixed.yml logs
    pause
    exit /b 1
)

echo.
echo ⏳ 等待服務啟動（約 10 秒）...
timeout /t 10 /nobreak >nul

echo.
echo ════════════════════════════════════════════════
echo     ✅ 啟動完成！
echo ════════════════════════════════════════════════
echo.
echo 📡 Unity 連接設定：
echo    IP:   127.0.0.1
echo    Port: 10000
echo.
echo 🔍 檢查服務狀態：
docker exec ros2_tools bash -c "source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 topic list 2>/dev/null | head -5"
echo.
echo 📝 常用命令：
echo.
echo 查看日誌：
echo   docker logs -f unity_ros2_tcp
echo.
echo 進入工具容器：
echo   docker exec -it ros2_tools bash
echo.
echo 監聽心跳：
echo   docker exec ros2_tools bash -c "source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 topic echo /unity/heartbeat"
echo.
echo 停止服務：
echo   docker-compose -f docker-compose-fixed.yml down
echo.
pause