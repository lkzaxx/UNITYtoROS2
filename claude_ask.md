════════════════════════════════════════════════════════
    ROS2 Unity OpenArm Docker 環境部署工具
════════════════════════════════════════════════════════

[94m[檢查][0m 檢查 Docker 是否已安裝...
Docker version 28.5.1, build e180ab8
[92m[成功][0m Docker 已安裝

[94m[檢查][0m 檢查 Docker 服務狀態...
[92m[成功][0m Docker 服務運行正常

[94m[清理][0m 檢查並停止現有容器...
[92m[成功][0m 舊容器已清理

[94m[檢查][0m 檢查必要檔案...
[92m[成功][0m 所有必要檔案都存在
使用 Docker Compose 檔案: docker-compose-humble.yml

[94m[準備][0m 拉取 Docker 映像（可能需要幾分鐘）...
humble-desktop: Pulling from osrf/ros
Digest: sha256:b2b520a8b42d868ed498a8c062ae8d555d3fa671e698641d4a53558badb8fc0c
Status: Image is up to date for osrf/ros:humble-desktop
docker.io/osrf/ros:humble-desktop
[92m[成功][0m Docker 映像準備完成

[1m[94m[啟動][0m 正在建立並啟動容器...

yaml: line 17: found unknown escape character
[91m[錯誤][0m 容器啟動失敗！

查看詳細錯誤訊息：
yaml: line 17: found unknown escape character

docker-compose-humble.yml還是有問題



w:\idaka\unity_ros2\ROS2\claude_ask.md 問題在此檔案 