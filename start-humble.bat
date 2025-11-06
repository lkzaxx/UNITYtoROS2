@echo off
REM Windows CMD 腳本：啟動 ROS2 Humble 容器

REM 停止舊的 Jazzy 容器
docker stop ros2_jazzy

REM 啟動 Humble 容器
docker run -it --rm ^
  --name ros2_humble ^
  -p 7400:7400/udp ^
  -p 7410-7420:7410-7420/udp ^
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ^
  -e CYCLONEDDS_URI=file:///etc/cyclonedds.xml ^
  -e ROS_DOMAIN_ID=0 ^
  -v %CD%:/root/ros2_ws ^
  -v %CD%/cyclonedds.xml:/etc/cyclonedds.xml ^
  osrf/ros:humble-desktop-full bash

