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
docker ps | findstr unity_ros2_tcp >nul
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

REM Create temporary script for TCP Endpoint
echo docker exec -it unity_ros2_tcp bash -c "source /opt/ros/humble/setup.bash && cd /root/ros2_ws && source install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000" > %TEMP%\start_tcp_endpoint.bat

REM Create temporary script for Unity bridge
echo docker exec -it unity_ros2_tcp bash -c "source /opt/ros/humble/setup.bash && cd /root/ros2_ws && source install/setup.bash && python3 /root/ros2_ws/install/unity_openarm_bridge/lib/unity_bridge_py/tcp_bridge_node" > %TEMP%\start_unity_bridge.bat

REM Start TCP Endpoint in a new window
echo [INFO] Starting TCP Endpoint server (new window)...
start "ROS-TCP-Endpoint" cmd /k "%TEMP%\start_tcp_endpoint.bat"

REM Wait a bit for the first service to start
timeout /t 2 /nobreak >nul

REM Start Unity bridge node in a new window
echo [INFO] Starting Unity bridge node (new window)...
start "Unity-Bridge" cmd /k "%TEMP%\start_unity_bridge.bat"

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

REM Clean up temporary files
del "%TEMP%\start_tcp_endpoint.bat" 2>nul
del "%TEMP%\start_unity_bridge.bat" 2>nul