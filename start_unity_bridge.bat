@echo off
chcp 65001 >nul
REM ============================================
REM Start Unity Bridge Node
REM Handles message conversion between Unity and ROS 2
REM ============================================

echo ============================================
echo Starting Unity Bridge Node
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

echo [INFO] Entering container and starting Unity bridge node...
echo [INFO] Processing topics: /unity/heartbeat, /unity/pose, /unity/joint_commands
echo [INFO] Press Ctrl+C to stop the node
echo.

REM Enter container and start Unity bridge node
docker exec -it ros2_humble bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && source install/setup.bash && python3 /root/ros2_ws/install/unity_openarm_bridge/lib/unity_bridge_py/tcp_bridge_node"

pause
