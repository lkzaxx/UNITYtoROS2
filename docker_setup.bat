@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

REM ========================================================
REM     ROS2 Unity OpenArm Docker 一鍵部署腳本
REM     自動建立和啟動所有必要的 Docker 容器
REM ========================================================

cls
echo.
echo ════════════════════════════════════════════════════════
echo     ROS2 Unity OpenArm Docker 環境部署工具
echo ════════════════════════════════════════════════════════
echo.

REM 設定顏色代碼（用於 PowerShell 輸出）
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
    echo.
    echo 安裝完成後，請確保 Docker Desktop 正在運行
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
    
    REM 嘗試啟動 Docker Desktop
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
            echo 請手動啟動 Docker Desktop 後重新執行此腳本
            pause
            exit /b 1
        )
    )
)

echo %GREEN%[成功]%RESET% Docker 服務運行正常
echo.

REM ===== 步驟 3: 檢查並停止舊容器 =====
echo %BLUE%[清理]%RESET% 檢查並停止現有容器...

REM 列出相關容器
set containers_found=0
for %%c in (unity_ros2_tcp openarm_controller ros2_tools ros2_rviz) do (
    docker ps -aq -f name=%%c >nul 2>&1
    if not errorlevel 1 (
        docker ps -aq -f name=%%c | findstr . >nul 2>&1
        if not errorlevel 1 (
            set containers_found=1
        )
    )
)

if !containers_found!==1 (
    echo 發現現有容器，正在停止並移除...
    docker-compose -f docker-compose-complete.yml down 2>nul
    docker-compose -f docker-compose-humble.yml down 2>nul
    docker-compose -f docker-compose.yml down 2>nul
    
    REM 強制移除個別容器（如果還存在）
    for %%c in (unity_ros2_tcp openarm_controller ros2_tools ros2_rviz) do (
        docker rm -f %%c 2>nul
    )
    echo %GREEN%[成功]%RESET% 舊容器已清理
) else (
    echo 沒有發現舊容器
)
echo.

REM ===== 步驟 4: 檢查必要的檔案 =====
echo %BLUE%[檢查]%RESET% 檢查必要檔案...

set files_missing=0

REM 檢查 docker-compose 檔案
if not exist "docker-compose-complete.yml" (
    if not exist "docker-compose-humble.yml" (
        if not exist "docker-compose.yml" (
            echo %RED%[錯誤]%RESET% 找不到 Docker Compose 檔案！
            set files_missing=1
        ) else (
            set COMPOSE_FILE=docker-compose.yml
        )
    ) else (
        set COMPOSE_FILE=docker-compose-humble.yml
    )
) else (
    set COMPOSE_FILE=docker-compose-complete.yml
)

REM 創建必要的目錄
if not exist "src" (
    echo 創建 src 目錄...
    mkdir src
)

if not exist "scripts" (
    echo 創建 scripts 目錄...
    mkdir scripts
)

REM 檢查 cyclonedds.xml
if not exist "cyclonedds.xml" (
    echo %YELLOW%[提示]%RESET% 創建預設 cyclonedds.xml 配置...
    (
        echo ^<?xml version="1.0" encoding="UTF-8" ?^>
        echo ^<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd"^>
        echo   ^<Domain id="any"^>
        echo     ^<General^>
        echo       ^<NetworkInterfaceAddress^>auto^</NetworkInterfaceAddress^>
        echo       ^<EnableMulticastLoopback^>true^</EnableMulticastLoopback^>
        echo     ^</General^>
        echo     ^<Discovery^>
        echo       ^<ParticipantIndex^>auto^</ParticipantIndex^>
        echo       ^<EnableTopicDiscoveryEndpoints^>true^</EnableTopicDiscoveryEndpoints^>
        echo     ^</Discovery^>
        echo     ^<Tracing^>
        echo       ^<Verbosity^>warning^</Verbosity^>
        echo     ^</Tracing^>
        echo   ^</Domain^>
        echo ^</CycloneDDS^>
    ) > cyclonedds.xml
)

if !files_missing!==1 (
    echo.
    echo %RED%[錯誤]%RESET% 缺少必要檔案！請確保所有檔案都在正確位置
    pause
    exit /b 1
)

echo %GREEN%[成功]%RESET% 所有必要檔案都存在
echo 使用 Docker Compose 檔案: %COMPOSE_FILE%
echo.

REM ===== 步驟 5: 拉取 Docker 映像 =====
echo %BLUE%[準備]%RESET% 拉取 Docker 映像（可能需要幾分鐘）...
docker pull osrf/ros:humble-desktop
if errorlevel 1 (
    echo %RED%[錯誤]%RESET% 無法拉取 Docker 映像！
    echo 請檢查網路連接
    pause
    exit /b 1
)
echo %GREEN%[成功]%RESET% Docker 映像準備完成
echo.

REM ===== 步驟 6: 啟動容器 =====
echo %BOLD%%BLUE%[啟動]%RESET% 正在建立並啟動所有容器...
echo.

REM 根據用戶選擇決定是否包含視覺化
choice /C YN /M "是否需要啟動 RViz2 視覺化工具（需要 X11 支援）"
if errorlevel 2 (
    echo 啟動基本服務（不含視覺化）...
    docker-compose -f %COMPOSE_FILE% up -d
) else (
    echo 啟動所有服務（包含視覺化）...
    
    REM WSL 環境變數設定（用於 X11）
    if defined WSL_DISTRO_NAME (
        echo 檢測到 WSL 環境，設定 DISPLAY 變數...
        set DISPLAY=:0
    )
    
    docker-compose -f %COMPOSE_FILE% --profile visualization up -d
)

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
echo 等待 ROS2 TCP Endpoint 啟動...
timeout /t 5 /nobreak >nul

REM 檢查 TCP Endpoint 是否在監聽
set service_ready=0
set attempts=0

:check_service
set /a attempts+=1
netstat -an | findstr ":10000.*LISTENING" >nul 2>&1
if not errorlevel 1 (
    set service_ready=1
) else (
    if !attempts! lss 10 (
        echo 等待服務啟動... (!attempts!/10^)
        timeout /t 2 /nobreak >nul
        goto check_service
    )
)

echo.

REM ===== 步驟 8: 顯示狀態 =====
echo ════════════════════════════════════════════════════════
echo     %GREEN%部署完成！%RESET%
echo ════════════════════════════════════════════════════════
echo.

REM 顯示容器狀態
echo %BOLD%容器狀態：%RESET%
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" | findstr "unity_ros2_tcp openarm_controller ros2_tools ros2_rviz"

echo.
echo %BOLD%服務狀態：%RESET%

if !service_ready!==1 (
    echo %GREEN%✓%RESET% TCP Endpoint 服務: 運行中 (端口 10000)
) else (
    echo %YELLOW%⚠%RESET% TCP Endpoint 服務: 啟動中...
)

echo.
echo %BOLD%Unity 連接資訊：%RESET%
echo   IP 地址:  127.0.0.1
echo   端口:     10000
echo   協議:     TCP

echo.
echo %BOLD%可用命令：%RESET%
echo.
echo   查看服務日誌:
echo     docker logs -f unity_ros2_tcp
echo.
echo   進入工具容器:
echo     docker exec -it ros2_tools bash
echo.
echo   列出 ROS2 主題:
echo     docker exec ros2_tools bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
echo.
echo   監聽心跳訊號:
echo     docker exec ros2_tools bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /unity/heartbeat"
echo.
echo   停止所有服務:
echo     docker-compose -f %COMPOSE_FILE% down
echo.
echo   重新啟動服務:
echo     docker-compose -f %COMPOSE_FILE% restart
echo.

REM ===== 步驟 9: 提供快速測試選項 =====
echo ════════════════════════════════════════════════════════
choice /C YN /T 10 /D N /M "是否要開啟服務監控視窗"
if errorlevel 2 (
    echo.
    echo %GREEN%設定完成！%RESET% 您現在可以從 Unity 連接到 ROS2
    echo.
) else (
    echo.
    echo 開啟監控視窗...
    start "ROS2 TCP Endpoint 日誌" cmd /k "docker logs -f unity_ros2_tcp"
    start "ROS2 主題監控" cmd /k "docker exec ros2_tools bash -c \"source /opt/ros/humble/setup.bash && ros2 topic list && echo. && echo 監聽心跳訊號... && ros2 topic echo /unity/heartbeat\""
    echo.
    echo %GREEN%監控視窗已開啟！%RESET%
    echo.
)

echo 按任意鍵關閉此視窗...
pause >nul

endlocal