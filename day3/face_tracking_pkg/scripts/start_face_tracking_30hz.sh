#!/bin/bash
# Face Tracking System 30Hz - 통합 실행 스크립트

echo "🚀 Face Tracking System 30Hz Starting..."
echo "=========================================="

# 작업 공간 설정
cd /home/rokey/ros2_ws
source install/setup.bash

# 이전 프로세스 정리
echo "🧹 Cleaning up previous processes..."
pkill -f "face_detection_node|face_tracking_node|robot_control_node"
sleep 2

# Launch 파일 실행
echo "▶️  Starting launch file..."
ros2 launch face_tracking_pkg face_tracking_30hz.launch.py

