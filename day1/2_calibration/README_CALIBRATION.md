# Hand-Eye Calibration 가이드

## 📋 목차
1. [개요](#개요)
2. [환경 설정](#환경-설정)
3. [캘리브레이션 데이터 수집](#캘리브레이션-데이터-수집)
4. [캘리브레이션 실행](#캘리브레이션-실행)
5. [캘리브레이션 검증](#캘리브레이션-검증)
6. [트러블슈팅](#트러블슈팅)

---

## 개요

Hand-Eye Calibration은 로봇 그리퍼에 장착된 카메라의 좌표계를 로봇 베이스 좌표계로 변환하는 과정입니다.

### 목표
- 카메라에서 본 물체의 3D 좌표 → 로봇 베이스 좌표계로 변환
- 정확한 Pick & Place 작업 수행

### 필요 장비
- Doosan M0609 로봇 (192.168.137.100:12345)
- Intel RealSense D435i 카메라
- OnRobot RG2 그리퍼 (192.168.1.1)
- 체커보드 패턴 (10x7 내부 코너, 25mm 정사각형)

---

## 환경 설정

### 1. 네트워크 설정

로봇과 그리퍼는 다른 IP 대역을 사용하므로, PC에 두 IP를 모두 설정해야 합니다.

```bash
# 현재 네트워크 확인
ip addr show

# 로봇 연결용 IP (192.168.137.x 대역)
# 일반적으로 자동 할당됨

# 그리퍼 연결용 IP 추가 (192.168.1.x 대역)
sudo ip addr add 192.168.1.100/24 dev enp3s0  # 또는 해당 인터페이스

# 연결 확인
ping -c 3 192.168.137.100  # 로봇
ping -c 3 192.168.1.1       # 그리퍼
```

### 2. ROS2 환경 준비

**터미널 1: 로봇 드라이버 실행**
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real \
  host:=192.168.137.100 \
  port:=12345 \
  model:=m0609
```

**터미널 2: RealSense 카메라 실행 (aligned depth 활성화)**
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

**터미널 3: 로봇 매뉴얼 모드 설정**
```bash
ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "robot_mode: 0"
```

### 3. 토픽 확인

```bash
# 카메라 토픽 확인
ros2 topic list | grep camera

# 필수 토픽:
# /camera/camera/color/image_raw
# /camera/camera/color/camera_info
# /camera/camera/aligned_depth_to_color/image_raw
```

---

## 캘리브레이션 데이터 수집

### 파일: `data_recording.py`

체커보드를 다양한 각도와 위치에서 촬영하여 캘리브레이션 데이터를 수집합니다.

### 실행 방법

```bash
cd ~/Tutorial/Calibration_Tutorial
python3 data_recording.py
```

### 데이터 수집 절차

1. **로봇 준비**
   - 로봇이 MANUAL 모드(파란색 LED)인지 확인
   - 체커보드를 카메라 시야에 배치

2. **이미지 캡처**
   - 티치펜던트로 로봇을 다양한 위치로 이동
   - 체커보드가 화면에 완전히 보이는 위치에서 `q` 키 눌러 저장
   - 최소 30장, 권장 50-60장 수집

3. **촬영 팁**
   - 다양한 각도: 정면, 좌/우측, 위/아래
   - 다양한 거리: 가까이, 멀리
   - 체커보드가 화면 전체를 채우도록
   - 흐릿하거나 부분만 보이는 이미지 제외

4. **종료**
   - `ESC` 키로 종료

### 출력 파일

```
data/
├── calibrate_data.json     # 로봇 포즈 정보
├── 345.62_-199.75_152.31.jpg
├── 348.50_-195.20_155.80.jpg
└── ...                     # 수집된 이미지들
```

### `calibrate_data.json` 구조

```json
{
  "poses": [
    [345.62, -199.75, 152.31, 180.0, 0.0, 90.0],  // [x, y, z, rx, ry, rz]
    [348.50, -195.20, 155.80, 180.0, 0.0, 90.0],
    ...
  ],
  "file_name": [
    "345.62_-199.75_152.31.jpg",
    "348.50_-195.20_155.80.jpg",
    ...
  ]
}
```

### 코드 주요 부분

```python
DEVICE_NUMBER = 4  # HD Webcam 또는 RealSense 번호
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# 로봇 위치 기록 (Tool/TCP 오류 시 타임스탬프 사용)
try:
    robot_pose = get_current_posx()[0]
    file_name = f"{robot_pose[0]:.2f}_{robot_pose[1]:.2f}_{robot_pose[2]:.2f}.jpg"
except:
    timestamp = int(time.time())
    file_name = f"img_{timestamp}.jpg"
    robot_pose = [0, 0, 0, 0, 0, 0]
```

---

## 캘리브레이션 실행

### 파일: `handeye_calibration.py`

수집된 이미지와 로봇 포즈를 사용하여 Hand-Eye 변환 행렬을 계산합니다.

### 실행 방법

```bash
cd ~/Tutorial/Calibration_Tutorial
python3 handeye_calibration.py
```

### 캘리브레이션 과정

1. **이미지 로드**
   - `data/calibrate_data.json` 읽기
   - 이미지 파일 로드

2. **카메라 내부 파라미터 계산**
   - 체커보드 코너 검출
   - `cv2.calibrateCamera()` 사용
   - 카메라 행렬(fx, fy, cx, cy)과 왜곡 계수 계산

3. **Hand-Eye 캘리브레이션**
   - 각 이미지에서 체커보드 위치 계산
   - 로봇 포즈와 체커보드 위치 매칭
   - `cv2.calibrateHandEye()` 실행 (PARK 방법)

4. **결과 저장**
   - `T_gripper2camera.npy`: 4x4 변환 행렬

### 출력 예시

```
===== Hand-Eye Calibration Results =====
R_gripper2camera:
 [[-0.99992567  0.00307007  0.01179932]
  [-0.00303638 -0.99999127  0.00287154]
  [ 0.01180803  0.0028355   0.99992626]]
T_gripper2camera:
 [30.698806, 57.633802, -218.498580]
```

### 체커보드 설정

```python
checkerboard_size = (10, 7)  # 내부 코너 개수 (가로, 세로)
square_size = 25             # 정사각형 크기 (mm)
```

### 코드 주요 함수

```python
# 1. 로봇 포즈 → 4x4 변환 행렬
def get_robot_pose_matrix(x, y, z, rx, ry, rz):
    R = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

# 2. 체커보드 위치 계산
def find_checkerboard_pose(image, board_size, square_size, camera_matrix, dist_coeffs):
    # 체커보드 코너 검출
    found, corners = cv2.findChessboardCorners(gray, board_size)
    # solvePnP로 카메라→체커보드 변환 구하기
    retval, rvec, tvec = cv2.solvePnP(objp, corners_sub, camera_matrix, dist_coeffs)
    return R, tvec

# 3. Hand-Eye 캘리브레이션
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base_list,
    t_gripper2base_list,
    R_checker2camera_list,
    t_checker2camera_list,
    method=cv2.CALIB_HAND_EYE_PARK
)
```

---

## 캘리브레이션 검증

### 파일: `test.py`

실제 로봇으로 캘리브레이션 정확도를 테스트합니다.

### 실행 방법

```bash
cd ~/Tutorial/Calibration_Tutorial
python3 test.py
```

### 테스트 절차

1. **로봇 드라이버 및 카메라 실행** (위 환경 설정 참고)

2. **프로그램 실행**
   - 카메라 화면이 표시됨
   - "Waiting for camera intrinsics..." 메시지 후 준비 완료

3. **테스트**
   - 로봇 작업 공간에 물체 배치
   - 카메라 화면에서 물체를 마우스 클릭
   - 로봇이 해당 위치로 이동하는지 확인

4. **정확도 평가**
   - ✅ 오차 5mm 이내: 매우 좋은 캘리브레이션
   - ✅ 오차 10mm 이내: 양호한 캘리브레이션
   - ⚠️ 오차 20mm 이상: 재캘리브레이션 필요

5. **종료**
   - `ESC` 키로 종료

### 출력 예시

```
Waiting for camera intrinsics...
Camera intrinsics received: fx=909.48, fy=909.79
img cordinate: (706, 377)
camera cordinate: ((28.62, 8.64, 392))
robot cordinate: ([317.29, 1.24, 22.84])
[Test] Moving above target: X=317.3, Y=1.2, Z=122.8
[Test] Moving to target: Z=22.8
[Test] Returning to ready position
```

### 코드 주요 부분

```python
class TestNode(Node):
    def __init__(self):
        # 카메라 intrinsics 대기
        while self.intrinsics is None:
            rclpy.spin_once(self.img_node)
            self.intrinsics = self.img_node.get_camera_intrinsic()
        
        # 캘리브레이션 결과 로드
        self.gripper2cam = np.load("T_gripper2camera.npy")
    
    def transform_to_base(self, camera_coords):
        """카메라 좌표 → 로봇 베이스 좌표 변환"""
        coord = np.append(np.array(camera_coords), 1)
        base2gripper = self.get_robot_pose_matrix(*get_current_posx()[0])
        base2cam = base2gripper @ self.gripper2cam
        td_coord = np.dot(base2cam, coord)
        return td_coord[:3]
    
    def pick_and_drop(self, x, y, z):
        """안전한 Pick & Place 동작"""
        # 1. 타겟 100mm 위로 이동
        pick_pos_above = posx([x, y, z + 100, ...])
        movel(pick_pos_above, vel=60, acc=60)
        
        # 2. 타겟 위치로 하강
        pick_pos = posx([x, y, z, ...])
        movel(pick_pos, vel=30, acc=30)
        
        # 3. 그리퍼 잡기
        self.gripper.close_gripper()
        
        # 4. 다시 올라가기
        movel(pick_pos_above, vel=30, acc=30)
        
        # 5. 초기 위치로 복귀
        movej(self.JReady, vel=60, acc=60)
        self.gripper.open_gripper()
```

---

## 트러블슈팅

### 1. 카메라 토픽이 안 보일 때

**증상:**
```bash
ros2 topic list | grep camera
# 아무것도 안 나옴
```

**해결:**
```bash
# RealSense 카메라 재시작
pkill -f realsense2_camera_node
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### 2. Depth 이미지를 못 받을 때

**증상:**
```
[INFO] [test_node]: retry get depth img
[INFO] [test_node]: retry get depth img
```

**해결:**
```bash
# aligned_depth 토픽 확인
ros2 topic list | grep aligned

# 없으면 카메라 재시작 (align_depth.enable:=true 필수)
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### 3. 그리퍼 연결 안 될 때

**증상:**
```
pymodbus.exceptions.ConnectionException: Failed to connect[ModbusTcpClient(192.168.1.1:502)]
```

**해결:**
```bash
# 그리퍼 IP 대역 확인
ip addr show | grep "192.168.1"

# 없으면 IP 추가
sudo ip addr add 192.168.1.100/24 dev enp3s0  # 또는 해당 인터페이스

# 연결 확인
ping -c 3 192.168.1.1
```

### 4. 로봇이 이동하지 않을 때

**원인:**
- 로봇이 MANUAL 모드가 아님
- 로봇 드라이버 미실행

**해결:**
```bash
# 로봇 상태 확인
ros2 topic echo /dsr01/state

# MANUAL 모드 설정 (BLUE LED)
ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "robot_mode: 0"
```

### 5. 캘리브레이션 결과에 nan 값이 나올 때

**원인:**
- 이미지 품질 불량
- 체커보드가 부분적으로만 보임
- 체커보드 크기 설정 오류

**해결:**
```bash
# 1. 기존 데이터 삭제
rm -rf data/*

# 2. 체커보드 설정 확인
# handeye_calibration.py에서
checkerboard_size = (10, 7)  # 내부 코너 개수 확인
square_size = 25             # 실제 크기 측정

# 3. 고품질 이미지 재수집
# - 체커보드 전체가 선명하게 보이는 이미지만
# - 다양한 각도와 거리에서 50장 이상
```

### 6. Camera Intrinsics가 None일 때

**증상:**
```
TypeError: 'NoneType' object is not subscriptable
```

**해결:**
```python
# test.py 수정 - __init__에서 대기 루프 추가
while self.intrinsics is None:
    rclpy.spin_once(self.img_node)
    self.intrinsics = self.img_node.get_camera_intrinsic()
    time.sleep(0.1)
```

### 7. Tool/TCP 설정 시 프로그램 멈춤

**원인:**
- 로봇 드라이버가 응답하지 않음
- Tool/TCP 설정 시간이 너무 김

**해결:**
```python
# data_recording.py에서 Tool/TCP 설정 주석 처리
# set_tool("Tool Weight_2FG")
# set_tcp("2FG_TCP")

# 티치펜던트에서 수동 설정:
# 1. [기본] → [도구/TCP] → [도구] → "Tool Weight_2FG" 선택
# 2. [기본] → [도구/TCP] → [TCP] → "2FG_TCP" 선택
```

---

## 파일 구조

```
Tutorial/Calibration_Tutorial/
├── data/                          # 캘리브레이션 데이터
│   ├── calibrate_data.json       # 로봇 포즈 정보
│   └── *.jpg                     # 캘리브레이션 이미지
├── data_recording.py             # 데이터 수집 스크립트
├── handeye_calibration.py        # 캘리브레이션 실행
├── test.py                       # 캘리브레이션 검증
├── realsense.py                  # RealSense 카메라 인터페이스
├── onrobot.py                    # OnRobot 그리퍼 제어
└── T_gripper2camera.npy          # 캘리브레이션 결과 (4x4 행렬)
```

---

## 참고 자료

### 캘리브레이션 이론

- **Eye-in-Hand**: 카메라가 로봇 그리퍼에 장착
- **변환 체인**: Base → Gripper → Camera → Object
- **목표**: T_gripper2camera 변환 행렬 계산

### 좌표계

```
Robot Base (베이스)
    ↓ T_base2gripper (로봇 포즈)
Gripper (그리퍼)
    ↓ T_gripper2camera (캘리브레이션 결과)
Camera (카메라)
    ↓ depth & intrinsics
Object (물체)
```

### OpenCV 함수

- `cv2.findChessboardCorners()`: 체커보드 코너 검출
- `cv2.calibrateCamera()`: 카메라 내부 파라미터 계산
- `cv2.solvePnP()`: 체커보드의 3D 위치 계산
- `cv2.calibrateHandEye()`: Hand-Eye 변환 행렬 계산

---

## 요약

### 전체 워크플로우

1. **환경 설정** → 로봇, 카메라, 네트워크
2. **데이터 수집** → `data_recording.py` (50+ 이미지)
3. **캘리브레이션** → `handeye_calibration.py` (T_gripper2camera.npy 생성)
4. **검증** → `test.py` (실제 로봇으로 정확도 테스트)

### 성공 기준

- ✅ 이미지 수: 50장 이상
- ✅ 체커보드: 전체가 선명하게 보임
- ✅ 다양성: 다양한 각도, 거리, 위치
- ✅ 결과: nan 값 없음, 오차 10mm 이내

### 명령어 요약

```bash
# 1. 환경 설정
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100 port:=12345 model:=m0609
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "robot_mode: 0"

# 2. 데이터 수집
python3 data_recording.py

# 3. 캘리브레이션
python3 handeye_calibration.py

# 4. 검증
python3 test.py
```

---

**작성일:** 2025-12-08  
**작성자:** GitHub Copilot  
**버전:** 1.0
