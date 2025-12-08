import os
import cv2
import json
import rclpy
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
DEVICE_NUMBER = 4  # HD Webcam (RealSense가 ROS2에서 사용중이면 접근 불가)

DR_init.dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("dsr_example_demo_py", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    # 로봇 제어 모듈 가져오기
    try:
        from DSR_ROBOT2 import (
            get_current_posx,
            set_tool,
            set_tcp,
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    # 공구 및 TCP 설정 (로봇 드라이버 실행 중이어야 함)
    # 주석 처리: 로봇 드라이버 응답 대기 시간이 너무 길어 프로그램이 멈춤
    # 티치펜던트에서 수동으로 Tool과 TCP를 설정하세요
    print("⚠ Tool/TCP 설정 건너뜀 - 티치펜던트에서 수동 설정 필요")
    # set_tool("Tool Weight_2FG")
    # set_tcp("2FG_TCP")

    # 데이터 저장 경로 설정
    source_path = "./data"
    os.makedirs(source_path, exist_ok=True)
    # 카메라 연결
    print(f"현재 선택된 device number는 {DEVICE_NUMBER}입니다.")
    cap = cv2.VideoCapture(DEVICE_NUMBER)  # 4 is camera number, set your camera number
    
    if not cap.isOpened():
        print(f"카메라를 찾을 수 없습니다. DEVICE_NUMBER={DEVICE_NUMBER}")
        print("사용 가능한 카메라: 0 (HD Webcam) 또는 4 (RealSense RGB)")
        rclpy.shutdown()
        return

    write_data = {}
    write_data["poses"] = []
    write_data["file_name"] = []
    
    print("카메라 연결 성공! 'q'를 눌러 이미지 저장, ESC로 종료")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break
        cv2.imshow("camera", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            print("종료합니다.")
            break
        elif key == ord("q"):
            try:
                # 로봇 위치 가져오기 시도
                pos = get_current_posx()[0]
                file_name = f"{pos[0]:.2f}_{pos[1]:.2f}_{pos[2]:.2f}.jpg"
                print(f"✓ 로봇 위치: {pos[:3]}")
            except Exception as e:
                # 로봇 위치를 못 가져오면 타임스탬프 사용
                import time
                timestamp = int(time.time() * 1000)
                file_name = f"img_{timestamp}.jpg"
                pos = [0, 0, 0, 0, 0, 0]  # 더미 포즈
                print(f"⚠ 로봇 위치 가져오기 실패, 타임스탬프 사용: {e}")
            
            # 이미지 저장
            file_path = f"{source_path}/{file_name}"
            cv2.imwrite(file_path, frame)
            write_data["file_name"].append(file_name)
            write_data["poses"].append(pos)
            
            # JSON 저장
            with open(f"{source_path}/calibrate_data.json", "w") as json_file:
                json.dump(write_data, json_file, indent=4)
            
            print(f"✓ 저장 완료 [{len(write_data['poses'])}장]: {file_path}")

    cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
