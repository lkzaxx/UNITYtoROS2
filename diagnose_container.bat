@echo off
chcp 65001 >nul
cls

echo ═══════════════════════════════════════════════════
echo     Unity ROS2 TCP 容器診斷工具
echo ═══════════════════════════════════════════════════
echo.

echo [1/5] 檢查 Docker 服務狀態...
docker info >nul 2>&1
if errorlevel 1 (
    echo ❌ Docker 未運行！請啟動 Docker Desktop
    pause
    exit /b 1
)
echo ✅ Docker 服務正常
echo.

echo [2/5] 檢查容器狀態...
docker ps -a --filter "name=unity_ros2_tcp" --format "table {{.Names}}\t{{.Status}}\t{{.RunningFor}}"
echo.

echo [3/5] 查看容器最近的日誌 (最後 50 行)...
echo ───────────────────────────────────────────────────
docker logs --tail 50 unity_ros2_tcp 2>&1
echo ───────────────────────────────────────────────────
echo.

echo [4/5] 檢查端口 10000 是否被占用...
netstat -ano | findstr :10000
if errorlevel 1 (
    echo ✅ 端口 10000 未被占用
) else (
    echo ⚠️  端口 10000 已被使用
)
echo.

echo [5/5] 檢查容器是否在重啟循環中...
echo.
echo 提示: 如果容器狀態顯示 "Restarting" 或持續重啟，
echo 請查看上面的日誌找出錯誤原因。
echo.

echo ═══════════════════════════════════════════════════
echo     診斷操作建議
echo ═══════════════════════════════════════════════════
echo.
echo 如果容器持續重啟，請執行以下步驟：
echo.
echo 1. 停止並移除舊容器:
echo    docker-compose -f docker-compose-humble.yml down
echo    docker rm -f unity_ros2_tcp
echo.
echo 2. 使用修正版配置重新啟動:
echo    docker-compose -f docker-compose-humble-fixed.yml up -d
echo.
echo 3. 持續監控日誌:
echo    docker logs -f unity_ros2_tcp
echo.
echo 4. 檢查容器內的進程:
echo    docker exec unity_ros2_tcp ps aux
echo.
echo 5. 進入容器手動測試:
echo    docker exec -it unity_ros2_tcp bash
echo.

pause