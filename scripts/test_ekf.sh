#!/bin/bash
# EKF 테스트 실행 스크립트

echo "🔬 EKF Integration Test"
echo "=========================================="

# 환경 설정
cd ~/ros2_ws
source install/setup.bash

echo ""
echo "📋 실행할 명령어:"
echo ""
echo "터미널 1 (이미 실행 중):"
echo "  ros2 run face_tracking_pkg face_detection_node"
echo ""
echo "터미널 2 (이미 실행 중):"
echo "  ros2 run face_tracking_pkg face_tracking_node"
echo ""
echo "터미널 3 (EKF 비교 - 선택):"
echo "  ros2 run face_tracking_pkg ekf_comparison_node"
echo ""
echo "터미널 4 (Robot Control - EKF ON):"
echo "  ros2 run face_tracking_pkg robot_control_node --ros-args -p use_ekf:=true"
echo ""
echo "터미널 5 (RViz - 선택):"
echo "  ros2 run rviz2 rviz2"
echo "  # Fixed Frame: base_link"
echo "  # Add Marker: /ekf_comparison/raw (빨간색)"
echo "  # Add Marker: /ekf_comparison/filtered (초록색)"
echo ""
echo "=========================================="
echo ""
read -p "EKF Comparison Node를 실행하시겠습니까? (y/n): " answer

if [ "$answer" = "y" ]; then
    echo "🚀 EKF Comparison Node 실행..."
    ros2 run face_tracking_pkg ekf_comparison_node
else
    echo "⏭️  건너뜀"
fi
