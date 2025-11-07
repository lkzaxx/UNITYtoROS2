@echo off
REM ========================================
REM 停止所有 Unity-OpenArm 服務
REM ========================================

echo ========================================
echo Stopping Unity-OpenArm Services
echo ========================================
echo.

echo Killing all Docker exec processes...

REM 殺掉所有相關的 cmd 窗口
taskkill /FI "WINDOWTITLE eq ROS2 TCP Endpoint*" /F >nul 2>&1
taskkill /FI "WINDOWTITLE eq Unity OpenArm Bridge*" /F >nul 2>&1
taskkill /FI "WINDOWTITLE eq MoveIt Servo*" /F >nul 2>&1
taskkill /FI "WINDOWTITLE eq RViz2*" /F >nul 2>&1

echo All service windows closed.
echo.
echo Note: The Docker container is still running.
echo To stop the container, run: docker stop unity_ros2_tcp
echo.
pause