@echo off
chcp 65001 >nul
REM ============================================
REM One-click start all ROS 2 services
REM Includes TCP Endpoint and Unity bridge node
REM ============================================

echo ============================================
echo Starting ROS 2 Services
echo ============================================
echo.

REM Check if container is running
docker ps | findstr ros2_humble >nul
if errorlevel 1 (
    echo [ERROR] ROS 2 container is not running!
    echo [INFO] Starting container...
    docker-compose -f docker-compose-humble.yml up -d
    if errorlevel 1 (
        echo [ERROR] Failed to start container!
        pause
        exit /b 1
    )
    echo [SUCCESS] Container started
    echo [INFO] Waiting for container to be ready...
    timeout /t 3 /nobreak >nul
)

echo [INFO] Starting services...
echo.

REM Start TCP Endpoint in a new window
echo [INFO] Starting TCP Endpoint server (new window)...
start "ROS-TCP-Endpoint" cmd /k "docker exec -it ros2_humble bash -c "source /opt/ros/humble/setup.bash && cd /root/ros2_ws && source install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000""

REM Wait a bit for the first service to start
timeout /t 2 /nobreak >nul

REM Start Unity bridge node in a new window
echo [INFO] Starting Unity bridge node (new window)...
start "Unity-Bridge" cmd /k "docker exec -it ros2_humble bash -c "source /opt/ros/humble/setup.bash && cd /root/ros2_ws && source install/setup.bash && python3 /root/ros2_ws/install/unity_openarm_bridge/lib/unity_bridge_py/tcp_bridge_node""

echo.
echo ============================================
echo [SUCCESS] All services started!
echo ============================================
echo.
echo Two windows have been opened:
echo   1. ROS-TCP-Endpoint - TCP Server
echo   2. Unity-Bridge - Unity Bridge Node
echo.
echo You can now connect Unity to 127.0.0.1:10000
echo.
echo Press any key to close this window (services will continue running)...
pause >nul