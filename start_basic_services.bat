@echo off
REM ========================================
REM Unity-OpenArm 基本服務啟動腳本
REM (僅 TCP Endpoint 和 Bridge)
REM ========================================

echo ========================================
echo Starting Unity-OpenArm Basic Services
echo ========================================
echo.

REM 檢查 Docker 容器是否運行
docker ps | findstr unity_ros2_tcp >nul
if errorlevel 1 (
    echo [ERROR] Docker container 'unity_ros2_tcp' is not running!
    echo Please start the container first with: docker start unity_ros2_tcp
    pause
    exit /b 1
)

echo [1/2] Starting ROS2 TCP Endpoint...
start "ROS2 TCP Endpoint" cmd /k "docker exec -it unity_ros2_tcp bash -c ""source /opt/ros/humble/setup.bash && cd /ros2_ws && source install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000"""
timeout /t 3 /nobreak >nul

echo [2/2] Starting Unity-OpenArm Bridge...
start "Unity OpenArm Bridge" cmd /k "docker exec -it unity_ros2_tcp bash -c ""source /opt/ros/humble/setup.bash && cd /ros2_ws && source install/setup.bash && python3 -m unity_openarm_bridge.tcp_bridge_node"""
timeout /t 2 /nobreak >nul

echo.
echo ========================================
echo Basic services started!
echo ========================================
echo.
echo Windows opened:
echo   1. ROS2 TCP Endpoint (Port: 10000)
echo   2. Unity-OpenArm Bridge
echo.
echo Now you can start Unity and connect to ROS2.
echo.
echo Press any key to exit this window...
pause >nul