# ROS2 Face Tracking System

Doosan M0609 로봇을 이용한 얼굴 추적 시스템입니다.

## 📦 패키지 구성

### 1. face_tracking
리팩토링된 얼굴 추적 패키지
- `face_detection_node.py` - YOLO 기반 얼굴 감지
- `face_tracking_node.py` - EKF 기반 얼굴 위치 추적 및 3D 좌표 변환
- `joint_tracking_node.py` - 로봇 관절 제어
- `ekf_filter.py` - Extended Kalman Filter 구현
- `yolo_detector.py` - YOLOv8 기반 감지기
- `constants.py` - 상수 정의

### 2. DoosanBootcampCol2
Doosan 로봇 SDK 및 드라이버 (ROS2 Humble)
- `dsr_bringup2` - 로봇 런치 파일
- `dsr_msgs2` - 메시지 정의
- `dsr_controller2` - 로봇 컨트롤러
- `dsr_description2` - URDF/xacro 모델

## 🔧 환경 요구사항

- ROS2 Humble
- Ubuntu 22.04
- Python 3.10+
- Intel RealSense D435 카메라 (또는 rosbag 재생)

### Python 의존성
```bash
pip install ultralytics opencv-python numpy filterpy
```

### ROS2 의존성
```bash
sudo apt install ros-humble-realsense2-camera ros-humble-tf2-ros ros-humble-visualization-msgs
```

## 🚀 실행 방법

### 1. 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select face_tracking DoosanBootcampCol2
source install/setup.bash
```

### 2. 로봇 실행 (가상 로봇)
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609 color:=blue
```

### 3. 카메라 실행

#### 실제 RealSense 카메라 사용:
```bash
ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true
```

#### Rosbag 재생 사용 (카메라 없이 테스트):
```bash
ros2 bag play /path/to/your/bag_file --loop
```

> ⚠️ **참고**: Rosbag 파일(5GB+)은 용량 문제로 포함되어 있지 않습니다.  
> 직접 녹화하거나 별도로 공유받으세요.

### 4. 얼굴 감지 노드 실행
```bash
ros2 run face_tracking face_detection_node
```

### 5. 얼굴 추적 노드 실행
```bash
ros2 run face_tracking face_tracking_node
```

### 6. 관절 제어 노드 실행
```bash
ros2 run face_tracking joint_tracking_node
```

## 📊 시스템 아키텍처

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  RealSense D435 │───▶│  Detection Node │───▶│  Tracking Node  │
│  /camera/color  │    │  (YOLO Face)    │    │  (EKF + TF)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                                      │
                                                      ▼
                       ┌─────────────────┐    ┌─────────────────┐
                       │  Doosan Robot   │◀───│  Joint Control  │
                       │  M0609          │    │  Node           │
                       └─────────────────┘    └─────────────────┘
```

## 🎛️ 파라미터

### face_detection_node
- `confidence_threshold`: 감지 신뢰도 임계값 (기본: 0.5)

### face_tracking_node
- `min_depth`, `max_depth`: 깊이 필터링 범위 (m)
- `roi_margin`: ROI 마진
- `process_noise`, `measurement_noise`: EKF 노이즈 설정

### joint_tracking_node
- `dead_zone_deg`: 불감대 (기본: 5.0도)
- `max_velocity`: 최대 속도 (기본: 20.0 deg/s)
- `smoothing_factor`: 스무딩 팩터 (기본: 0.3)
- `j2_enabled`, `j3_enabled`, `j6_enabled`: 관절별 활성화

## 📁 Rosbag 녹화 방법

카메라 데이터를 녹화하려면:
```bash
ros2 bag record /camera/color/image_raw /camera/depth/image_rect_raw /camera/aligned_depth_to_color/image_raw /camera/color/camera_info -o my_face_tracking_bag
```

## 🔗 관련 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera/color/image_raw` | Image | 컬러 이미지 |
| `/camera/aligned_depth_to_color/image_raw` | Image | 정렬된 깊이 이미지 |
| `/face_detection/faces` | Detection2DArray | 감지된 얼굴 |
| `/face_marker` | Marker | RViz 시각화 마커 |
| `/dsr01m0609/jog_multi` | JogMultiAxis | 로봇 조그 명령 |

## 📝 라이선스

이 프로젝트는 교육 목적으로 제작되었습니다.

## 👥 Contributors

- Rokey Bootcamp Col2 Team
