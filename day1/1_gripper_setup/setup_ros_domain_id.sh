#!/bin/bash
# ROS_DOMAIN_ID 설정 스크립트

echo "=== ROS_DOMAIN_ID 설정 ==="

# 기존 설정 확인 및 제거
if grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "기존 설정 제거 중..."
    sed -i '/ROS_DOMAIN_ID/d' ~/.bashrc
fi

# 새 설정 추가
echo "" >> ~/.bashrc
echo "# ROS2 Domain ID - 조별 네트워크 격리" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=60" >> ~/.bashrc

# 현재 세션에 적용
export ROS_DOMAIN_ID=60

echo "✅ 설정 완료!"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""
echo "💡 새 터미널 열거나 'source ~/.bashrc' 실행"
