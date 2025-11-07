@echo off
chcp 65001 >nul
REM ============================================
REM Start ROS-TCP-Endpoint Server
REM For Unity to connect to ROS 2
REM ============================================

echo ============================================
echo Starting ROS-TCP-Endpoint Server
echo ============================================
echo.

REM Check if container is running
docker ps | findstr ros2_humble >nul
if errorlevel 1 (
    echo [ERROR] ROS 2 container is not running!
    echo Please run: docker-compose -f docker-compose-humble.yml up -d
    pause
    exit /b 1
)

echo [INFO] Entering container and starting TCP Endpoint server...
echo [INFO] Listening on port: 10000
echo [INFO] Press Ctrl+C to stop the server
echo.

REM Enter container and start TCP Endpoint server
docker exec -it ros2_humble bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && source install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000"

pause
