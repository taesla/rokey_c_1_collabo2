#!/usr/bin/env python3
"""onrobot.py 고급 기능 테스트"""

import sys
sys.path.append('/home/rokey/Tutorial/Calibration_Tutorial')

from onrobot import RG
import time

GRIPPER_IP = "192.168.1.1"
GRIPPER_PORT = "502"

print("=== OnRobot RG2 고급 기능 테스트 ===\n")

try:
    # 그리퍼 연결
    gripper = RG("rg2", GRIPPER_IP, GRIPPER_PORT)
    print("✓ 그리퍼 연결 성공\n")
    
    # 1. Fingertip Offset 확인
    print("1. Fingertip Offset 확인")
    offset = gripper.get_fingertip_offset()
    print(f"   현재 Fingertip Offset: {offset} mm")
    print(f"   (그리퍼 핑거팁 교체 시 오프셋 값)\n")
    
    # 2. Width 측정 (오프셋 미포함 vs 포함)
    print("2. Width 측정 비교")
    width_raw = gripper.get_width()
    width_offset = gripper.get_width_with_offset()
    print(f"   오프셋 미포함 너비: {width_raw} mm")
    print(f"   오프셋 포함 너비: {width_offset} mm")
    print(f"   차이: {width_offset - width_raw} mm\n")
    
    # 3. 다양한 힘으로 그리핑
    print("3. 다양한 힘으로 그리핑 테스트")
    
    # 약한 힘 (50N)
    print("   a) 50N의 힘으로 닫기...")
    gripper.close_gripper(force_val=500)  # 50N (1/10 N 단위)
    while gripper.get_status()[0]:
        time.sleep(0.3)
    print(f"      ✓ 완료 - 너비: {gripper.get_width_with_offset()} mm\n")
    
    time.sleep(1)
    
    # 그리퍼 열기
    gripper.open_gripper()
    while gripper.get_status()[0]:
        time.sleep(0.3)
    
    time.sleep(1)
    
    # 강한 힘 (200N)
    print("   b) 200N의 힘으로 닫기...")
    gripper.close_gripper(force_val=2000)  # 200N
    while gripper.get_status()[0]:
        time.sleep(0.3)
    print(f"      ✓ 완료 - 너비: {gripper.get_width_with_offset()} mm\n")
    
    time.sleep(1)
    
    # 4. 정밀한 위치 제어
    print("4. 정밀 위치 제어 테스트")
    positions = [800, 600, 400, 200]  # 80mm, 60mm, 40mm, 20mm
    
    for pos in positions:
        print(f"   {pos/10}mm로 이동 중...")
        gripper.move_gripper(width_val=pos, force_val=1000)
        
        while gripper.get_status()[0]:
            time.sleep(0.2)
        
        actual_width = gripper.get_width_with_offset()
        print(f"   ✓ 목표: {pos/10}mm → 실제: {actual_width}mm")
        time.sleep(0.5)
    
    print()
    
    # 5. 상태 플래그 상세 확인
    print("5. 그리퍼 상태 플래그 확인")
    status = gripper.get_status()
    print(f"   상태 배열: {status}")
    print("   플래그 의미:")
    print(f"   [0] Busy: {status[0]} (동작 중)")
    print(f"   [1] Grip Detected: {status[1]} (물체 감지)")
    print(f"   [2] Safety Switch 1: {status[2]}")
    print(f"   [3] Safety Circuit 1: {status[3]}")
    print(f"   [4] Safety Switch 2: {status[4]}")
    print(f"   [5] Safety Circuit 2: {status[5]}")
    print(f"   [6] Safety Error: {status[6]}\n")
    
    # 6. 최종 위치로 복귀
    print("6. 중간 위치(50mm)로 복귀...")
    gripper.move_gripper(width_val=500, force_val=1000)
    while gripper.get_status()[0]:
        time.sleep(0.2)
    print(f"   ✓ 최종 위치: {gripper.get_width_with_offset()} mm\n")
    
    # 연결 종료
    gripper.close_connection()
    print("=== 고급 기능 테스트 완료 ===")
    
except Exception as e:
    print(f"✗ 에러 발생: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
