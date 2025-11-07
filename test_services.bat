@echo off
REM ========================================
REM 測試 Unity-OpenArm 服務狀態
REM ========================================

echo ========================================
echo Testing Unity-OpenArm Services
echo ========================================
echo.

REM 檢查 Docker 容器
echo [1/4] Checking Docker container...
docker ps | findstr unity_ros2_tcp >nul
if errorlevel 1 (
    echo [FAIL] Container 'unity_ros2_tcp' is not running
    echo.
    pause
    exit /b 1
) else (
    echo [OK] Container 'unity_ros2_tcp' is running
)
echo.

REM 檢查 ROS2 環境
echo [2/4] Checking ROS2 environment...
docker exec -it unity_ros2_tcp bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && source install/setup.bash && ros2 pkg list | grep -E '(tcp|unity)'" >nul 2>&1
if errorlevel 1 (
    echo [FAIL] ROS2 packages not found
    echo.
    pause
    exit /b 1
) else (
    echo [OK] ROS2 packages found
)
echo.

REM 測試 TCP 端口
echo [3/4] Checking TCP port 10000...
netstat -an | findstr ":10000" >nul
if errorlevel 1 (
    echo [INFO] Port 10000 is not in use (service not started)
) else (
    echo [OK] Port 10000 is active
)
echo.

REM 列出可用的套件
echo [4/4] Available ROS2 packages:
docker exec -it unity_ros2_tcp bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && source install/setup.bash && ros2 pkg list | grep -E '(tcp|unity)'"
echo.

echo ========================================
echo Test completed!
echo ========================================
echo.
pause