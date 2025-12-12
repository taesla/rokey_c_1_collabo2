#!/bin/bash
# EKF Before/After 간편 테스트 (영상 버전)

echo "🔬 EKF Before/After 실시간 비교 도구 (Camera View)"
echo "=========================================="
echo ""
echo "📋 실행 전 체크리스트:"
echo "  ✅ face_detection_node 실행 중"
echo "  ✅ face_tracking_node 실행 중"
echo ""
echo "📹 이 도구가 보여주는 것:"
echo "  - 상단 좌: BEFORE 영상 (빨간 마커 - 떨림)"
echo "  - 상단 우: AFTER 영상 (초록 마커 - 부드러움)"
echo "  - 하단: 노이즈 감소 통계 및 실시간 그래프"
echo ""
echo "⌨️  종료: ESC 키 또는 Ctrl+C"
echo ""
echo "=========================================="
echo ""

cd ~/ros2_ws
source install/setup.bash

echo "🚀 실행 중..."
echo ""

ros2 run face_tracking_pkg ekf_before_after
