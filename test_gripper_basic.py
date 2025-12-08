#!/usr/bin/env python3
"""그리퍼 기본 동작 테스트"""

import sys
sys.path.append('/home/rokey/Tutorial/Calibration_Tutorial')

from onrobot import RG
import time

GRIPPER_IP = "192.168.1.1"
GRIPPER_PORT = "502"

print("=== OnRobot RG2 그리퍼 테스트 시작 ===\n")

try:
    # 그리퍼 연결
    print("1. 그리퍼 연결 중...")
    gripper = RG("rg2", GRIPPER_IP, GRIPPER_PORT)
    print("   ✓ 연결 성공!\n")
    
    # 현재 상태 확인
    print("2. 현재 그리퍼 상태:")
    width = gripper.get_width_with_offset()
    print(f"   현재 너비: {width} mm")
    status = gripper.get_status()
    print(f"   상태: {status}\n")
    
    # 그리퍼 열기
    print("3. 그리퍼 열기...")
    gripper.open_gripper()
    
    # 동작이 끝날 때까지 대기
    while gripper.get_status()[0]:  # busy flag
        time.sleep(0.5)
        print("   열리는 중...")
    
    width = gripper.get_width_with_offset()
    print(f"   ✓ 완전히 열림! 현재 너비: {width} mm\n")
    
    time.sleep(1)
    
    # 그리퍼 닫기
    print("4. 그리퍼 닫기...")
    gripper.close_gripper()
    
    while gripper.get_status()[0]:
        time.sleep(0.5)
        print("   닫히는 중...")
    
    width = gripper.get_width_with_offset()
    print(f"   ✓ 완전히 닫힘! 현재 너비: {width} mm\n")
    
    time.sleep(1)
    
    # 특정 너비로 이동
    print("5. 그리퍼를 50mm로 이동...")
    gripper.move_gripper(width_val=500, force_val=100)  # 50mm, 10N
    
    while gripper.get_status()[0]:
        time.sleep(0.5)
        print("   이동 중...")
    
    width = gripper.get_width_with_offset()
    print(f"   ✓ 이동 완료! 현재 너비: {width} mm\n")
    
    # 연결 종료
    gripper.close_connection()
    print("=== 테스트 완료 ===")
    
except Exception as e:
    print(f"✗ 에러 발생: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
