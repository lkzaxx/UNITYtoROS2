@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

REM ========================================================
REM     ROS2 Unity OpenArm Docker 一鍵部署腳本 (修正版)
REM     自動建立和啟動所有必要的 Docker 容器
REM ========================================================

cls
echo.
echo ════════════════════════════════════════════════════════
echo     ROS2 Unity OpenArm Docker 環境部署工具
echo ════════════════════════════════════════════════════════
echo.

REM 設定顏色代碼
set "GREEN=[92m"
set "YELLOW=[93m"
set "RED=[91m"
set "BLUE=[94m"
set "RESET=[0m"
set "BOLD=[1m"

REM ===== 步驟 1: 檢查 Docker 是否安裝 =====
echo %BLUE%[檢查]%RESET% 檢查 Docker 是否已安裝...
docker --version >nul 2>&1
if errorlevel 1 (
    echo %RED%[錯誤]%RESET% Docker 未安裝或未啟動！
    echo.
    echo 請先安裝 Docker Desktop:
    echo https://www.docker.com/products/docker-desktop
    pause
    exit /b 1
)

docker --version
echo %GREEN%[成功]%RESET% Docker 已安裝
echo.

REM ===== 步驟 2: 檢查 Docker 是否正在運行 =====
echo %BLUE%[檢查]%RESET% 檢查 Docker 服務狀態...
docker info >nul 2>&1
if errorlevel 1 (
    echo %YELLOW%[提示]%RESET% Docker Desktop 未運行，嘗試啟動...
    start "" "C:\Program Files\Docker\Docker\Docker Desktop.exe"
    
    echo 等待 Docker Desktop 啟動...
    set count=0
    :wait_docker
    timeout /t 2 /nobreak >nul
    set /a count+=1
    docker info >nul 2>&1
    if errorlevel 1 (
        if !count! lss 30 (
            echo 等待中... (!count!/30^)
            goto wait_docker
        ) else (
            echo %RED%[錯誤]%RESET% Docker Desktop 啟動失敗！
            pause
            exit /b 1
        )
    )
)

echo %GREEN%[成功]%RESET% Docker 服務運行正常
echo.

REM ===== 步驟 3: 檢查並停止舊容器 =====
echo %BLUE%[清理]%RESET% 檢查並停止現有容器...

REM 停止可能存在的容器
docker-compose -f docker-compose-humble.yml down 2>nul

REM 強制移除個別容器
for %%c in (unity_ros2_tcp ros2_humble ros2_tools ros2_rviz) do (
    docker rm -f %%c 2>nul
)

echo %GREEN%[成功]%RESET% 舊容器已清理
echo.

REM ===== 步驟 4: 檢查必要的檔案 =====
echo %BLUE%[檢查]%RESET% 檢查必要檔案...

REM ✅ 修正：只檢查實際存在的配置檔案
if not exist "docker-compose-humble.yml" (
    echo %RED%[錯誤]%RESET% 找不到 docker-compose-humble.yml！
    pause
    exit /b 1
)

set COMPOSE_FILE=docker-compose-humble.yml

REM 創建必要的目錄
if not exist "src" mkdir src
if not exist "scripts" mkdir scripts

echo %GREEN%[成功]%RESET% 所有必要檔案都存在
echo 使用 Docker Compose 檔案: %COMPOSE_FILE%
echo.

REM ===== 步驟 5: 拉取 Docker 映像 =====
echo %BLUE%[準備]%RESET% 拉取 Docker 映像（可能需要幾分鐘）...
docker pull osrf/ros:humble-desktop
if errorlevel 1 (
    echo %RED%[錯誤]%RESET% 無法拉取 Docker 映像！
    pause
    exit /b 1
)
echo %GREEN%[成功]%RESET% Docker 映像準備完成
echo.

REM ===== 步驟 6: 啟動容器 =====
echo %BOLD%%BLUE%[啟動]%RESET% 正在建立並啟動容器...
echo.

docker-compose -f %COMPOSE_FILE% up -d

if errorlevel 1 (
    echo %RED%[錯誤]%RESET% 容器啟動失敗！
    echo.
    echo 查看詳細錯誤訊息：
    docker-compose -f %COMPOSE_FILE% logs --tail 50
    pause
    exit /b 1
)

echo.
echo %GREEN%[成功]%RESET% 容器啟動中，等待服務就緒...
echo.

REM ===== 步驟 7: 等待服務就緒 =====
echo 等待 ROS2 服務啟動...
timeout /t 5 /nobreak >nul

REM 檢查容器狀態
docker ps | findstr "ros2_humble\|unity_ros2_tcp" >nul
if errorlevel 1 (
    echo %YELLOW%⚠%RESET% 容器可能未正確啟動
) else (
    echo %GREEN%✓%RESET% 容器運行正常
)

echo.

REM ===== 步驟 8: 顯示狀態 =====
echo ════════════════════════════════════════════════════════
echo     %GREEN%部署完成！%RESET%
echo ════════════════════════════════════════════════════════
echo.

REM 顯示容器狀態
echo %BOLD%容器狀態：%RESET%
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"

echo.
echo %BOLD%Unity 連接資訊：%RESET%
echo   IP 地址:  127.0.0.1
echo   端口:     10000
echo   協議:     TCP

echo.
echo %BOLD%下一步：%RESET%
echo   1. 使用 start_all_services.bat 啟動所有服務
echo   2. 或手動啟動：
echo      - start_tcp_endpoint.bat
echo      - start_unity_bridge.bat
echo.

echo 按任意鍵關閉此視窗...
pause >nul

endlocal