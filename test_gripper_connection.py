#!/usr/bin/env python3
"""그리퍼 연결 테스트 스크립트"""

from onrobot import RG
import sys

GRIPPER_IP = "192.168.1.1"  # 그리퍼 IP (필요시 변경)
GRIPPER_PORT = "502"

print(f"그리퍼 연결 시도: {GRIPPER_IP}:{GRIPPER_PORT}")

try:
    gripper = RG("rg2", GRIPPER_IP, GRIPPER_PORT)
    print("✓ 그리퍼 연결 성공!")
    
    # 상태 확인
    status = gripper.get_status()
    print(f"그리퍼 상태: {status}")
    
    # 현재 너비 확인
    width = gripper.get_width_with_offset()
    print(f"현재 그리퍼 너비: {width} mm")
    
    gripper.close_connection()
    
except Exception as e:
    print(f"✗ 그리퍼 연결 실패: {e}")
    print("\n확인 사항:")
    print("1. 그리퍼가 이더넷으로 연결되어 있나요?")
    print("2. 그리퍼 전원이 켜져 있나요?")
    print("3. PC 이더넷이 192.168.1.x 대역으로 설정되어 있나요?")
    sys.exit(1)
