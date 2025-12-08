# 📚 Day 1 세미나 가이드: RGB Camera 로봇 통신환경 구축

> **대상**: 조원들 (세미나 형식 수업)  
> **예상 시간**: 1시간 30분 ~ 2시간  
> **준비물**: 로봇, 그리퍼, 카메라, 체커보드

---

## 🎯 세미나 순서 요약

| 순서 | 주제 | 시간 | 강의자료 | 실습 |
|:---:|------|:----:|:--------:|:----:|
| 0 | 프로젝트 전체 개요 | 10분 | p.1-7 | ❌ |
| 1 | RealSense 카메라 이론 | 15분 | p.8-13 | ❌ |
| 2 | RealSense 실습 | 10분 | p.14-20 | ✅ |
| 3 | Modbus 통신 이론 | 15분 | p.23-25 | ❌ |
| 4 | 그리퍼 제어 실습 | 15분 | p.26-30 | ✅ |
| 5 | Hand-Eye Calibration 이론 | 20분 | p.31-34 | ❌ |
| 6 | Calibration 실습/데모 | 15분 | p.35-44 | ✅ |
| 7 | ROS_DOMAIN_ID & 정리 | 10분 | - | ❌ |

---

# 0️⃣ 프로젝트 전체 개요 (10분)

## 📄 강의자료 참조: p.1-7

### 프로젝트 최종 목표 (p.5)
> **"사람의 음성명령 인식 및 문장을 분석하여 해당 물체를 인식하고 명령한 장소로 이동하는 로봇 제작"**

### 필요 Skill 6가지 (p.6)

| Skill | 설명 | 예시 |
|-------|------|------|
| **Object Detection** | 사용자가 원하는 물체의 위치를 파악 | YOLO로 "망치" 찾기 |
| **Wakeup Word** | "명령 시작" 신호 | "시리야", "오케이 구글" |
| **STT** | Speech-to-Text, 음성→텍스트 변환 | "망치 가져와" → 텍스트 |
| **LangChain** | 핵심 키워드 추출 | "망치 가져와" → 키워드: 망치 |
| **Camera Calibration** | 카메라-로봇 좌표 변환 | 📍 **오늘 학습** |
| **Gripper 제어** | Modbus 통신으로 그리퍼 제어 | 📍 **오늘 학습** |

### 전체 수업 계획 (p.4)

| 일차 | 주제 | Chapter |
|:---:|------|:-------:|
| **1일차** | RGB Camera 로봇 통신환경 구축 (Gripper, Calibration) | 01, 02, 03 |
| 2일차 | Depth Camera Calibration 심화 | 04 |
| 3일차 | Vision AI 모델 기초 (YOLO, Ultralytics) | 05 |
| 4일차 | Object Detection → Pick and Place | 06 |
| 5일차 | Text 입력 기반 Pick and Place | 06 |
| 6일차 | Wakeup Word, Voice Recognition, STT | 07 |
| 7일차 | LLM, LangChain 기반 ROS2 통신 | 08 |
| 8-10일차 | 통합 및 프로젝트 완성 | - |

### 💡 질문 유도
- "로봇이 물체를 집으려면 어떤 정보가 필요할까?"
- "카메라가 물체를 봤는데, 로봇은 어떻게 그 위치를 알까?"

---

# 1️⃣ RealSense 카메라 이론 (15분)

## 📄 강의자료 참조: p.8-13

### RealSense란? (p.8)
> **RGB-D 센서**: 컬러 이미지(RGB)와 깊이(Depth)를 동시에 추출할 수 있는 센서  
> 로봇 비전, SLAM, 3D 매핑 등에 활용

### D435 구성요소 3가지 (p.9-11)

#### 1. Depth Sensor (p.9)
| 항목 | 설명 |
|------|------|
| **원리** | 두 개의 적외선(IR) 카메라를 이용한 **스테레오 방식** |
| **시차(Disparity)** | 두 카메라 간의 시차를 이용해 거리 정보 추정 |
| **장점** | 빛의 변화에 강하게 영향받지 않음 → 실내, 야간에도 안정적 측정 |

```
    [왼쪽 IR 카메라]     [오른쪽 IR 카메라]
           \                 /
            \               /
             \             /
              ▼           ▼
              ┌───────────┐
              │   물체    │  ← 두 카메라의 "시차"로 거리 계산
              └───────────┘
```

**스테레오 비전 원리**:
- 사람의 두 눈처럼 두 카메라가 같은 물체를 다른 각도에서 봄
- 가까운 물체: 시차 큼 (멀리 떨어져 보임)
- 먼 물체: 시차 작음 (비슷한 위치에 보임)
- 시차(disparity) = 기준선(baseline) × 초점거리 / 거리

#### 2. IR Projector (p.10)
| 항목 | 설명 |
|------|------|
| **역할** | 보이지 않는 적외선(IR) 패턴을 주변 환경에 투사 |
| **필요 이유** | 표면에 texture(특징)가 없는 경우 깊이 계산 보조 |
| **예시** | 하얀 벽, 유리, 단색 표면 등 |
| **효과** | 스테레오 매칭 강화, Depth Precision 향상 |

```
    [IR Projector]
         │
         │ 적외선 점 패턴 투사
         ▼
    ┌─────────────┐
    │ ∙ ∙ ∙ ∙ ∙  │  ← 하얀 벽에 점 패턴이 생김
    │  ∙ ∙ ∙ ∙   │     → 스테레오 매칭 가능!
    │ ∙ ∙ ∙ ∙ ∙  │
    └─────────────┘
```

#### 3. Color Camera (p.11)
| 항목 | 설명 |
|------|------|
| **역할** | 일반 RGB 컬러 이미지 캡처 |
| **활용** | Depth 데이터와 매핑(mapping)하여 3D Reconstruction |
| **출력** | `/camera/camera/color/image_raw` 토픽 |

### D435 스펙 (p.12)

| 항목 | Depth | Color |
|------|-------|-------|
| **해상도** | 최대 1280×720 | 최대 1920×1080 |
| **FPS** | 최대 90fps | 최대 30fps |
| **FOV** | 87° × 58° | 69° × 42° |
| **거리** | 0.1m ~ 10m | - |

### 💡 핵심 개념: aligned_depth
```
문제: Color 이미지와 Depth 이미지의 해상도/시야각이 다름
     → 픽셀 위치가 안 맞음!

해결: aligned_depth_to_color
     → Depth를 Color에 맞게 정렬
     → 같은 픽셀 위치 = 같은 물체
```

---

# 2️⃣ RealSense 실습 (10분)

## 📄 강의자료 참조: p.13-20

### 실습 1: 장치 번호 확인
```bash
v4l2-ctl --list-devices
```

**출력 예시**:
```
Intel RealSense D435i
├── /dev/video0   (metadata)
├── /dev/video2   (Depth 848x480)
├── /dev/video4   (Depth 848x100)
├── /dev/video6   (RGB 640x480 YUYV)
├── /dev/video10  (RGB 1280x720 YUYV)  ← 우리가 사용
└── /dev/video12  (IR 카메라)
```

### 실습 2: RealSense 런치 (p.13)
```bash
# 터미널 1: RealSense 노드 실행
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### 실습 3: rqt로 이미지 확인 (p.14-16)
```bash
# 터미널 2: rqt 실행
rqt
# Plugins → Visualization → Image View
```

**확인할 토픽**:
| 토픽 | 설명 |
|------|------|
| `/camera/camera/color/image_raw` | Color 이미지 |
| `/camera/camera/depth/image_rect_raw` | Depth 이미지 (흑백, 거리=밝기) |
| `/camera/camera/aligned_depth_to_color/image_raw` | 정렬된 Depth |

### 실습 4: RViz2로 PointCloud 확인 (p.17-20)
```bash
# 터미널 1: PointCloud 포함 런치
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true

# 터미널 2: RViz2 실행
rviz2
```

**RViz2 설정**:
1. Add 버튼 클릭
2. By topic → camera → depth → color → points → **PointCloud2** 선택
3. Fixed Frame을 `camera_depth_frame`으로 변경

### 💡 조원 참여
- 각자 rqt에서 Color/Depth 이미지 확인
- 손을 카메라 앞에 가져가서 Depth 변화 관찰

---

# 3️⃣ Modbus 통신 이론 (15분)

## 📄 강의자료 참조: p.23-25

### Modbus란? (p.23)

> **1979년**, 지금은 Schneider Electric인 **모디콘(Modicon)**에서 만든 **시리얼 통신 프로토콜**

| 항목 | 설명 |
|------|------|
| **목적** | PLC(프로그래머블 로직 컨트롤러) 통신 |
| **특징** | 단순하지만 장비 제어와 모니터링에 필요한 기능 수행 가능 |
| **지위** | 사실상의 산업용 표준 프로토콜 |

### Modbus가 산업용으로 널리 사용되는 이유 (p.23)
1. ✅ 산업용 통신 프로토콜로 개발됨
2. ✅ **Master-Slave 구조** (단순명료)
3. ✅ 프로토콜이 **공개**되어 있고 **무료**
4. ✅ 설치와 유지보수가 용이
5. ✅ 비트 단위 또는 워드(16비트) 단위로 정보 조작 용이

### Modbus 통신 Packet 구조 (p.24)

```
┌─────────┬───────────┬─────────────────────────┬──────────────┐
│  주소   │ 기능코드  │         데이터          │   오류검증   │
│ (Slave) │(읽기/쓰기)│ (레지스터 주소 및 개수) │    (CRC)     │
└─────────┴───────────┴─────────────────────────┴──────────────┘
```

**통신 방식**:
- **Master**가 특정 **Slave**에게 요청을 보냄
- 해당 Slave만 응답
- 우리의 경우: PC(Master) ↔ 그리퍼(Slave)

### Modbus 프로토콜 종류 (p.25)

| 항목 | Modbus RTU | Modbus ASCII | **Modbus TCP/IP** |
|------|------------|--------------|-------------------|
| **전송방식** | Serial (RS-232, RS-485) | Serial | **Ethernet (TCP/IP)** |
| **Frame 형식** | Binary | ASCII Text | TCP/IP Packet |
| **속도** | 빠름 | 느림 | **매우 빠름** |
| **거리** | 수백 미터 | - | 거리 제한 없음 |
| **응용분야** | 공장 자동화 | 디버깅 | **원격 모니터링** |

> 💡 **우리가 사용하는 것**: **Modbus TCP/IP** (이더넷 기반, 빠름)

### 💡 질문 유도
- "왜 그리퍼는 ROS Topic이 아닌 Modbus를 쓸까?"
  - → 산업용 장비는 Modbus가 표준, ROS는 연구용
- "Holding Register가 뭘까?"
  - → 그리퍼의 상태/명령을 저장하는 메모리 영역

---

# 4️⃣ 그리퍼 제어 실습 (15분)

## 📄 강의자료 참조: p.26-30

### RG2 Gripper 스펙 (p.22)

| 항목 | 값 |
|------|-----|
| **Payload (Force Fit)** | 2kg |
| **Payload (Form Fit)** | 5kg |
| **Total Stroke** | 0 ~ 110mm |
| **Gripping Force** | 3 ~ 40N |

### 네트워크 구성
```
    PC (노트북)                     OnRobot RG2 그리퍼
    192.168.1.100   ◄──이더넷──►   192.168.1.1
    (USB-이더넷)                    (Modbus TCP 포트: 502)
```

### 실습 1: 네트워크 확인
```bash
ping 192.168.1.1
```

### 실습 2: 연결 테스트
```bash
cd ~/day1_gripper_setup/day1/1_gripper_setup
python3 test_gripper_connection.py
```

**예상 출력**:
```
그리퍼 연결 시도: 192.168.1.1:502
✓ 그리퍼 연결 성공!
현재 그리퍼 너비: 101.6 mm
```

### 실습 3: 기본 동작 테스트 (p.28)
```bash
python3 test_gripper_basic.py
```

**테스트 항목**:
- `gripper.open_gripper()` → 최대로 열기
- `gripper.close_gripper()` → 완전히 닫기
- `gripper.move_gripper(500, 400)` → 50mm로 이동, 40N 힘

### 핵심 코드 설명 (onrobot.py)

```python
from pymodbus.client.sync import ModbusTcpClient

class RG:
    def __init__(self, model, ip, port):
        self.client = ModbusTcpClient(ip, port=int(port))
        self.client.connect()
    
    def move_gripper(self, width_val, force_val):
        # 레지스터 0: Target Force (1/10 N 단위)
        # 레지스터 1: Target Width (1/10 mm 단위)
        # 레지스터 2: Control (0x0001 = 이동 명령)
        self.client.write_registers(0, [force_val, width_val, 0x0001])
```

### 그리퍼 상태 플래그 (p.29)

```bash
python3 test_gripper_advanced.py
```

| 비트 | 상태 플래그 | 설명 |
|:---:|-------------|------|
| 0 | **Busy** | 1: 동작 중, 0: 대기 중 (새 명령 가능) |
| 1 | **Grip Detected** | 1: 물체 감지됨, 0: 감지되지 않음 |
| 2 | S1 Pushed | 1: 안전 스위치 1 눌림 |
| 3 | S1 Triggered | 1: 안전 회로 1 활성화됨 (재부팅 필요) |
| 4 | S2 Pushed | 1: 안전 스위치 2 눌림 |
| 5 | S2 Triggered | 1: 안전 회로 2 활성화됨 (재부팅 필요) |
| 6 | Safety Error | 1: 전원 인가 시 안전 스위치 눌려 있음 |

### 💡 조원 참여
- 각자 그리퍼 열기/닫기 명령 실행해보기
- 물체를 집었을 때 `Grip Detected` 플래그 확인

---

# 5️⃣ Hand-Eye Calibration 이론 (20분) ⭐ 핵심!

## 📄 강의자료 참조: p.31-34

### Camera Calibration이란? (p.31)

> 카메라의 **내·외부 파라미터**를 추정하는 과정

| 구분 | 파라미터 | 설명 | 보통 제공 |
|------|---------|------|:--------:|
| **내부** | 초점 거리, 주점, 렌즈 왜곡 계수 | 카메라에 의한 왜곡 보정 | ✅ 제조사 제공 |
| **외부** | 위치, 방향 (회전/이동) | 3D 공간 좌표 정확히 매핑 | ❌ 직접 계산 |

### 왜 외부 파라미터가 필요한가? (p.32)

> **"로봇 좌표계 기준으로 카메라가 어디에 위치해 있는지 알아야 함!"**

```
문제 상황:
- 카메라가 물체를 봄 → "물체가 카메라 기준 (x, y, z)에 있다"
- 로봇에게 전달 → "그 좌표가 로봇 기준으로 어디야?"
- 변환 필요! → T_gripper2camera (그리퍼→카메라 변환 행렬)
```

### 좌표계 Chain Rule (p.32)

```
[Robot Base] ──T_base2tcp──► [TCP] ──T_gripper2camera──► [Camera] ──T_camera2target──► [Target]
     │                                                                                    │
     └────────────────────── T_base2target ───────────────────────────────────────────────┘
                                (항상 동일!)
```

**핵심 원리**:
- 체커보드(Target)는 고정되어 있음
- 로봇을 여러 포즈로 움직여도 `T_base2target`은 항상 같음
- 이 조건을 이용해 `T_gripper2camera`를 계산!

### Eye-in-Hand vs Eye-to-Hand (p.33-34)

| 항목 | Eye-in-Hand | Eye-to-Hand |
|------|-------------|-------------|
| **카메라 위치** | 로봇 팔(그리퍼)에 부착 | 고정된 공간에 위치 |
| **체커보드 위치** | 고정된 공간에 위치 | 로봇 팔에 부착 |
| **촬영 방법** | 로봇 팔을 움직여가며 촬영 | 로봇 팔을 움직여가며 촬영 |
| **우리가 사용** | ✅ **이 방식** | ❌ |

```
Eye-in-Hand 구조:

    [Robot Base]
         │
         │ T_base2tcp (로봇이 알려줌)
         ▼
       [TCP]
         │
         │ T_tcp2gripper (고정, 알고 있음)
         ▼
     [Gripper]
         │
         │ T_gripper2camera (❓ 우리가 구해야 함!)
         ▼
     [Camera] ───────► [Checkerboard]
                        (고정 위치)
```

### 수학적 배경: AX = XB 문제

```
여러 포즈에서 촬영했을 때:

포즈 1: A₁ · X = X · B₁
포즈 2: A₂ · X = X · B₂
  ...
포즈 n: Aₙ · X = X · Bₙ

여기서:
- A = T_base2tcp (로봇 포즈, 로봇이 알려줌)
- B = T_camera2target (카메라→체커보드, OpenCV가 계산)
- X = T_gripper2camera (❓ 우리가 구하려는 것!)

최소 3개 이상의 포즈 필요 (우리는 54장 사용)
```

### OpenCV의 calibrateHandEye 함수

```python
import cv2

R_gripper2cam, t_gripper2cam = cv2.calibrateHandEye(
    R_gripper2base,  # 로봇 TCP 회전 (역변환)
    t_gripper2base,  # 로봇 TCP 위치 (역변환)
    R_target2cam,    # 체커보드→카메라 회전
    t_target2cam,    # 체커보드→카메라 위치
    method=cv2.CALIB_HAND_EYE_TSAI  # 알고리즘 선택 (0~4)
)
```

**사용 가능한 알고리즘** (p.42):
| method | 알고리즘 | 특징 |
|:------:|----------|------|
| 0 | TSAI | 빠름, 일반적 사용 |
| 1 | PARK | 안정적 |
| 2 | HORAUD | 정밀도 높음 |
| 3 | ANDREFF | 노이즈에 강함 |
| 4 | DANIILIDIS | 최신 알고리즘 |

### 💡 질문 유도
- "왜 여러 장의 사진이 필요할까?"
  - → 3D 변환은 6자유도(회전 3 + 이동 3), 방정식을 풀려면 다양한 포즈 필요
- "체커보드는 왜 사용할까?"
  - → 코너 검출이 정확, 크기가 알려져 있음 → 3D 위치 계산 가능

---

# 6️⃣ Calibration 실습/데모 (15분)

## 📄 강의자료 참조: p.35-44

### 준비물
- **체커보드**: 10×7 내부 코너, 25mm 정사각형
- 다운로드: https://raw.githubusercontent.com/MarkHedleyJones/markhedleyjones.github.io/master/media/calibration-checkerboard-collection/Checkerboard-A4-25mm-10x7.pdf

### 실습 1: 로봇 연결 (p.35)
```bash
# 터미널 1: 로봇 bringup
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100
```

### 실습 2: 직접 교시 모드 (p.36)
```bash
# 터미널 2: 직접 교시 모드로 전환
ros2 service call /dsr01/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 2}"
```
- 로봇 LED가 **BLUE**로 바뀌면 성공
- 손으로 로봇 팔을 움직일 수 있음

### 실습 3: 데이터 수집 (p.37)
```bash
cd ~/day1_gripper_setup/day1/2_calibration
python3 data_recording.py
```

**수집 방법**:
1. 체커보드가 카메라에 보이도록 로봇 위치 조정
2. `q` 키 → 현재 이미지와 로봇 TCP 좌표 저장
3. 로봇을 다른 포즈로 이동
4. 반복 (20장 이상, 우리는 54장 수집)

**저장되는 파일** (p.39-40):
```
Calibration_Tutorial/
├── images/
│   ├── 0.png
│   ├── 1.png
│   └── ...
└── calibrate_data.json  # TCP 좌표 목록
```

### 실습 4: 캘리브레이션 실행 (p.41-42)
```bash
python3 handeye_calibration.py
```

**확인 사항** (p.41):
- `checkerboard_size = (10, 7)` 또는 `(7, 10)`
- `square_size = 25` (mm)

**결과**: `T_gripper2camera.npy` 파일 생성

### 결과 확인
```bash
python3 -c "import numpy as np; T = np.load('T_gripper2camera.npy'); print(T)"
```

**우리 팀 결과**:
```
회전(R):
[[ 0.999  -0.012   0.035]
 [ 0.011   0.999   0.023]
 [-0.035  -0.023   0.999]]
 → 거의 단위행렬 (카메라와 그리퍼가 거의 평행)

이동(T):
[ 30.7mm,  57.6mm, -218.5mm ]
  (x: 오른쪽, y: 아래, z: 뒤쪽)
```

### 실습 5: Pick & Place 테스트 (p.43-44)
```bash
python3 test.py
```

### 💡 데모 포인트
- 저장된 54장 이미지 보여주기
- 캘리브레이션 결과 행렬 해석
- 실제 Pick & Place 동작 시연

---

# 7️⃣ ROS_DOMAIN_ID & 정리 (10분)

## ROS_DOMAIN_ID란?

> **같은 네트워크에서 ROS2 노드들을 그룹으로 격리하는 설정**

### 왜 필요한가? (p.44)
- 같은 WiFi에 여러 팀이 있을 때
- 다른 팀의 토픽이 섞이면 안 됨!
- DOMAIN_ID가 다르면 서로 통신 안 됨

### 우리 팀 설정
```bash
# ~/.bashrc에 추가
export ROS_DOMAIN_ID=60

# 적용
source ~/.bashrc

# 확인
echo $ROS_DOMAIN_ID
```

### 조별 DOMAIN_ID 권장
| 조 | DOMAIN_ID |
|:--:|:---------:|
| 1조 | 60 |
| 2조 | 61 |
| 3조 | 62 |
| 4조 | 63 |
| 5조 | 64 |

---

# 📋 마무리 요약

## 오늘 배운 것

| 주제 | 핵심 내용 |
|------|----------|
| **RealSense** | RGB-D 카메라, Depth 원리 (스테레오, IR), aligned_depth |
| **Modbus** | 산업용 통신 표준, Master-Slave, TCP/IP |
| **그리퍼** | RG2 제어, 상태 플래그, pymodbus |
| **Calibration** | Eye-in-Hand, AX=XB 문제, T_gripper2camera |
| **ROS_DOMAIN_ID** | 네트워크 격리 |

## 다음 시간 예고 (Day 2~)
- Object Detection (YOLO, Ultralytics)
- 모델 학습 및 추론
- Pick & Place 통합

---

## 📁 참고 자료

| 자료 | 위치 |
|------|------|
| 강의자료 PDF | [docs/협동로봇2 강의자료.pdf](../docs/협동로봇2%20강의자료.pdf) |
| Day1 통합 정리 | [Day1_Summary.md](Day1_Summary.md) |
| 그리퍼 코드 | [1_gripper_setup/](1_gripper_setup/) |
| 캘리브레이션 코드 | [2_calibration/](2_calibration/) |
| 인터랙티브 슬라이드 | https://taesla.github.io/rokey_c_1_collabo2/ |

---

> 💡 **세미나 팁**: 이론 설명 후 바로 실습으로 연결하면 집중력 유지에 좋습니다!
