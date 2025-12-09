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

---

## 🔄 워크스페이스 변경사항 (상세)

이 섹션은 `~/ros2_ws/src/DoosanBootcampCol2/` 패키지에서 실제로 변경된 내용을 상세히 기록합니다.

### 1. 변경된 파일 목록

#### 📂 `dsr_description2/xacro/`

| 파일 | 상태 | 설명 |
|------|------|------|
| `m0609.urdf.xacro` | **수정됨** | 그리퍼와 카메라 추가 |
| `m0609.urdf.xacro.original` | **새로 생성** | 원본 백업 파일 |
| `onrobot_rg2.urdf.xacro` | **새로 추가** | OnRobot RG2 그리퍼 URDF |

#### 📂 `dsr_description2/meshes/visual/` (새로 추가된 파일 6개)

| 파일 | 설명 |
|------|------|
| `body.stl` | 그리퍼 본체 |
| `single_bracket.stl` | 로봇 플랜지 연결 브라켓 |
| `moment_arm.stl` | 핑거 모멘트 암 |
| `truss_arm.stl` | 핑거 트러스 암 |
| `flex_finger.stl` | 유연 핑거 |
| `finger_tip.stl` | 핑거 팁 (검은색) |

#### 📂 `dsr_description2/meshes/collision/` (새로 추가된 파일 6개)

위와 동일한 파일들 (충돌 감지용, 저해상도)

---

### 2. `m0609.urdf.xacro` 변경 내용 (Before → After)

#### 🔴 원본 (Before)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="m0609" >
  <!-- ... 기존 코드 ... -->
  
  <xacro:unless value="${gz}">
    <xacro:include filename="$(find dsr_description2)/ros2_control/m0609.ros2_control.xacro" />
    <xacro:m0609_ros2_control name="m0609"/>
  </xacro:unless>
  
</robot>
```

#### 🟢 수정 후 (After)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="m0609" >
  <!-- ... 기존 코드 동일 ... -->
  
  <xacro:unless value="${gz}">
    <xacro:include filename="$(find dsr_description2)/ros2_control/m0609.ros2_control.xacro" />
    <xacro:m0609_ros2_control name="m0609"/>
  </xacro:unless>

  <!-- ============================================== -->
  <!-- OnRobot RG2 Gripper + Intel RealSense D435i   -->
  <!-- ============================================== -->
  
  <!-- RG2 Gripper attached to link_6 (tool flange) -->
  <xacro:include filename="$(find dsr_description2)/xacro/onrobot_rg2.urdf.xacro" />
  <xacro:onrobot_rg2 prefix="gripper" parent="link_6">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:onrobot_rg2>
  
  <!-- Intel RealSense D435i Camera -->
  <xacro:include filename="$(find realsense2_description)/urdf/_d435i.urdf.xacro" />
  
  <!-- Camera mounted on link_6 with calibrated transform -->
  <xacro:sensor_d435i parent="link_6" name="camera" use_nominal_extrinsics="true">
    <origin xyz="0.032630 0.060100 0.0" rpy="-1.5708 -1.5708 0"/>
  </xacro:sensor_d435i>
  
</robot>
```

---

### 3. `onrobot_rg2.urdf.xacro` 주요 내용

- **출처**: https://github.com/ikalevatykh/onrobot_ros (ROS1 패키지)
- **수정사항**:
  1. 패키지 경로 변경: `package://onrobot_description` → `package://dsr_description2`
  2. 조인트 타입 변경: `type="revolute"` → `type="fixed"` (joint_state 발행 불필요하도록)

#### 그리퍼 링크 구조

```
link_6 (로봇 툴 플랜지)
└── gripper_bracket (브라켓)
    └── gripper_body (본체)
        ├── gripper_moment_arm_left
        │   └── gripper_truss_arm_left
        │       └── gripper_finger_tip_left
        ├── gripper_moment_arm_right
        │   └── gripper_truss_arm_right
        │       └── gripper_finger_tip_right
        └── gripper_grasp_frame (TCP)
```

---

### 4. 추가로 생성된 패키지 (사용하지 않음)

작업 과정에서 `~/ros2_ws/src/gripper_camera_description/` 패키지가 생성되었으나, 최종적으로 **dsr_description2에 직접 통합**하는 방식을 선택했습니다.

```
gripper_camera_description/  (참고용, 실제로 사용하지 않음)
├── CMakeLists.txt
├── package.xml
├── launch/view_robot.launch.py
├── config/display.rviz, rg2_v1.yaml
├── meshes/visual/, collision/
└── urdf/onrobot_rg2.urdf.xacro, etc.
```

---

### 5. 삭제된 패키지

ROS1 전용 패키지로 ROS2에서 빌드 불가하여 삭제:

```bash
# 삭제된 패키지들
~/ros2_ws/src/onrobot_ros/
~/ros2_ws/src/onrobot_control/
~/ros2_ws/src/onrobot_description/
~/ros2_ws/src/onrobot_gazebo/
```

---

### 6. 원본 복원 방법

```bash
# m0609.urdf.xacro 원본 복원
cp ~/ros2_ws/src/DoosanBootcampCol2/dsr_description2/xacro/m0609.urdf.xacro.original \
   ~/ros2_ws/src/DoosanBootcampCol2/dsr_description2/xacro/m0609.urdf.xacro

# 추가된 파일 삭제 (선택사항)
rm ~/ros2_ws/src/DoosanBootcampCol2/dsr_description2/xacro/onrobot_rg2.urdf.xacro
rm ~/ros2_ws/src/DoosanBootcampCol2/dsr_description2/meshes/visual/{body,single_bracket,moment_arm,truss_arm,flex_finger,finger_tip}.stl
rm ~/ros2_ws/src/DoosanBootcampCol2/dsr_description2/meshes/collision/{body,single_bracket,moment_arm,truss_arm,flex_finger,finger_tip}.stl

# 다시 빌드
cd ~/ros2_ws && colcon build --packages-select dsr_description2 --symlink-install
```

---

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
