# Face Tracking Package

RealSense D435i 카메라와 Doosan M0609 로봇을 사용한 얼굴 추적 시스템

## 개요

이 패키지는 3개의 ROS2 노드로 구성됩니다:

1. **face_detection_node** - Haar Cascade로 얼굴 감지
2. **face_tracking_node** - TF2로 카메라→로봇 좌표 변환
3. **robot_control_node** - J1(좌우) + J5(상하) 관절 제어

## 데이터 흐름

```
RealSense D435i 카메라
    ↓ /camera/.../image_raw
[face_detection_node] ← Haar Cascade 얼굴 감지
    ↓ /face_detection/faces (2D 픽셀 좌표)
[face_tracking_node] ← 깊이 + TF2로 3D 변환
    ↓ /face_tracking/marker_robot (3D 좌표, base_link 기준)
[robot_control_node] ← J1/J5 관절 제어
    ↓ DSR_ROBOT2 API
Doosan M0609 로봇팔
```

## 설치

```bash
cd ~/ros2_ws
colcon build --packages-select face_tracking_pkg
source install/setup.bash
```

## 사용법

### 전체 시스템 실행 (로봇 + 비전)

**사전 조건:**
1. RealSense 카메라 실행
2. Doosan 로봇 드라이버 실행

```bash
# 터미널 1: RealSense
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

# 터미널 2: Doosan 로봇
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100 port:=12345 model:=m0609

# 터미널 3: Face Tracking (3개 노드 한번에)
ros2 launch face_tracking_pkg face_tracking.launch.py
```

### 비전만 테스트 (로봇 없이)

```bash
ros2 launch face_tracking_pkg vision_only.launch.py
```

### 개별 노드 실행

```bash
ros2 run face_tracking_pkg face_detection_node
ros2 run face_tracking_pkg face_tracking_node
ros2 run face_tracking_pkg robot_control_node
```

## Launch 파라미터

```bash
# 속도 조절
ros2 launch face_tracking_pkg face_tracking.launch.py velocity:=40 acceleration:=30

# OpenCV 창 숨기기
ros2 launch face_tracking_pkg face_tracking.launch.py show_window:=false
```

## 토픽

### 구독
- `/camera/camera/color/image_raw` - 컬러 이미지
- `/camera/camera/aligned_depth_to_color/image_raw` - 깊이 이미지
- `/camera/camera/color/camera_info` - 카메라 파라미터

### 발행
- `/face_detection/image` - 얼굴 표시된 이미지
- `/face_detection/faces` - 얼굴 2D 좌표
- `/face_tracking/marker` - 카메라 프레임 마커 (초록)
- `/face_tracking/marker_robot` - 로봇 프레임 마커 (빨강)
- `/face_tracking/line` - 카메라→얼굴 라인 (노랑)

## 로봇 제어

robot_control_node 실행 후 터미널에서:
- `s` + Enter: 시작 (사주경계 모드)
- `h` + Enter: 홈 위치로 이동
- `q` + Enter: 종료

### 동작 모드

1. **IDLE**: 대기
2. **PATROL**: 사주경계 (J1 ±60도 스캔)
3. **TRACKING**: 얼굴 추적 (J1+J5 제어)

## 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| velocity | 35 | 로봇 이동 속도 |
| acceleration | 25 | 로봇 가속도 |
| j1_gain | 0.12 | J1 좌우 반응 게인 |
| j5_gain | 0.08 | J5 상하 반응 게인 |
| patrol_step | 10.0 | 사주경계 스텝 각도 |
| detection_timeout | 2.0 | 감지 타임아웃 (초) |
| max_fail_count | 3 | 최대 실패 횟수 |

## 작업 이력

### 2025-12-10
- Hand-Eye Calibration 완료
- DSR 스레딩 문제 해결 (SingleThreadedExecutor)
- J5→J1 관절 인덱스 수정
- 역기구학(ikin) 테스트 및 분석
- J1+J5 관절 직접 제어 구현
- 패키지 통합 완료
