# Doosan M0609 + OnRobot RG2 Gripper + Intel RealSense D435i URDF

이 패키지는 Doosan M0609 로봇에 OnRobot RG2 그리퍼와 Intel RealSense D435i 카메라를 통합한 URDF 파일들을 포함합니다.

## 📁 폴더 구조

```
urdf_gripper_camera/
├── xacro/
│   ├── m0609.urdf.xacro          # 수정된 M0609 URDF (그리퍼+카메라 포함)
│   └── onrobot_rg2.urdf.xacro    # OnRobot RG2 그리퍼 URDF
├── meshes/
│   ├── visual/                    # 시각화용 메쉬 파일 (.stl)
│   └── collision/                 # 충돌 감지용 메쉬 파일 (.stl)
└── README.md
```

## 🔧 설치 방법

### 1. 파일 복사

```bash
# xacro 파일 복사
cp xacro/*.xacro ~/ros2_ws/src/DoosanBootcampCol2/dsr_description2/xacro/

# 메쉬 파일 복사
cp meshes/visual/*.stl ~/ros2_ws/src/DoosanBootcampCol2/dsr_description2/meshes/visual/
cp meshes/collision/*.stl ~/ros2_ws/src/DoosanBootcampCol2/dsr_description2/meshes/collision/
```

### 2. 원본 백업 (권장)

```bash
cd ~/ros2_ws/src/DoosanBootcampCol2/dsr_description2/xacro/
cp m0609.urdf.xacro m0609.urdf.xacro.original
```

### 3. 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select dsr_description2 --symlink-install
source install/setup.bash
```

## 🚀 실행 방법

```bash
# RViz와 함께 실행 (실제 로봇 연결)
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 model:=m0609

# 시뮬레이션 모드
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 model:=m0609
```

## 📷 카메라 TF 정보

카메라는 `link_6`에 연결되어 있으며, 다음 변환이 적용됩니다:

| 파라미터 | 값 |
|---------|-----|
| x | 0.032630 m |
| y | 0.060100 m |
| z | 0.0 m |
| roll | -1.5708 rad (-90°) |
| pitch | -1.5708 rad (-90°) |
| yaw | 0 rad |

**Quaternion**: `[qx=0.5, qy=0.5, qz=0.5, qw=-0.5]`

### 수동 TF 발행 (참고용)

URDF에 이미 포함되어 있어 별도 실행 불필요하지만, 참고용:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.032630 --y 0.060100 --z 0.0 \
  --qx 0.5 --qy 0.5 --qz 0.5 --qw -0.5 \
  --frame-id link_6 --child-frame-id camera_link
```

## 📦 필수 의존성

- `realsense2_description` - Intel RealSense D435i URDF
- `dsr_description2` - Doosan 로봇 URDF
- `dsr_bringup2` - Doosan 로봇 launch 파일

```bash
# RealSense 패키지 설치 확인
ros2 pkg prefix realsense2_description
```

## 🔗 참고 자료

- OnRobot RG2 URDF 출처: https://github.com/ikalevatykh/onrobot_ros
- Intel RealSense ROS2: https://github.com/IntelRealSense/realsense-ros

## ⚠️ 주의사항

1. `m0609.urdf.xacro`는 **m0609 모델 전용**입니다. 다른 모델(m1013 등)을 사용하려면 해당 모델의 xacro 파일도 수정해야 합니다.

2. 카메라 위치는 Hand-Eye Calibration 결과를 기반으로 합니다. 실제 장착 위치가 다르면 `origin xyz` 값을 조정하세요.

3. 원본 복원 방법:
```bash
cp ~/ros2_ws/src/DoosanBootcampCol2/dsr_description2/xacro/m0609.urdf.xacro.original \
   ~/ros2_ws/src/DoosanBootcampCol2/dsr_description2/xacro/m0609.urdf.xacro
```
