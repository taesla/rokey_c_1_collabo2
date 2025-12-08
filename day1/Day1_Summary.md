# 📚 Day 1 학습 정리: RGB Camera 로봇 통신환경 구축

> **작성일**: 2025년 12월 8일  
> **강의 Chapter**: 01, 02, 03, 04 (부분)  
> **주제**: Depth Camera 패키지, Camera Topic, PointCloud, Gripper 제어, Modbus, Calibration

---

## 📋 목차

1. [수업 개요](#1-수업-개요)
2. [RealSense 카메라](#2-realsense-카메라)
3. [RG2 Gripper & Modbus 통신](#3-rg2-gripper--modbus-통신)
4. [Hand-Eye Calibration](#4-hand-eye-calibration)
5. [ROS_DOMAIN_ID 설정](#5-ros_domain_id-설정)
6. [트러블슈팅](#6-트러블슈팅)
7. [실습 결과물](#7-실습-결과물)
8. [다음 단계](#8-다음-단계)

---

## 1. 수업 개요

### 1.1 전체 프로젝트 목표
> 사람의 음성명령 인식 및 문장을 분석하여 해당 물체를 인식하고 명령한 장소로 이동하는 로봇 제작

### 1.2 필요 Skill (강의자료 p.7)
| Skill | 설명 | Day 1 진행 여부 |
|-------|------|:---------------:|
| Object Detection | 물체 위치 파악 | ❌ Day 3-5 |
| Wakeup Word | 명령 시작 신호 (ex: "시리야") | ❌ Day 6-7 |
| STT (Speech-to-Text) | 음성→텍스트 변환 | ❌ Day 6-7 |
| LangChain | 키워드 추출 (ex: "망치 가져와" → 망치) | ❌ Day 7-8 |
| **Camera Calibration** | 카메라-로봇 좌표 변환 | ✅ **오늘 완료** |
| **Gripper 제어** | Modbus 통신으로 그리퍼 제어 | ✅ **오늘 완료** |

### 1.3 Day 1 학습 범위
```
Chapter 01: Realsense Depth Camera, Voice Processing 및 Tutorial 패키지 설치
Chapter 02: Realsense Depth Camera Topic 및 PointCloud with RQt and RViz2
Chapter 03: Gripper 제어와 Modbus 통신
Chapter 04: Depth Camera Calibration (부분 진행)
```

---

## 2. RealSense 카메라

### 2.1 이론 (강의자료 p.9-13)

**RealSense D435 구성요소:**

| 구성요소 | 설명 |
|---------|------|
| **Depth Sensor** | 두 개의 IR 카메라로 스테레오 방식 깊이 계산, 시차(disparity) 이용 |
| **IR Projector** | 적외선 패턴 투사, texture 없는 표면(흰 벽, 유리) 깊이 측정 보조 |
| **Color Camera** | RGB 컬러 이미지 캡처, Depth 데이터와 매핑하여 3D Reconstruction |

### 2.2 실습 - 카메라 설정

**장치 번호 확인:**
```bash
v4l2-ctl --list-devices
```

**오늘 확인된 설정:**
```
Intel RealSense D435i
├── /dev/video0  (metadata)
├── /dev/video2  (Depth, 848x480)
├── /dev/video4  (Depth, 848x100)
├── /dev/video6  (RGB, 640x480 YUYV)
├── /dev/video10 (RGB, 1280x720 YUYV) ← 사용
└── /dev/video12 (IR 카메라)
```

**RealSense 런치 (ROS2):**
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### 2.3 Topic 확인
```bash
# rqt로 이미지 확인
rqt  # Plugins → Visualization → Image View

# 주요 토픽
/camera/camera/color/image_raw           # Color 이미지
/camera/camera/depth/image_rect_raw      # Depth 이미지
/camera/camera/aligned_depth_to_color/image_raw  # 정렬된 Depth
```

---

## 3. RG2 Gripper & Modbus 통신

### 3.1 이론 (강의자료 p.23-30)

**Modbus란?**
> 1979년 Modicon에서 만든 시리얼 통신 프로토콜. PLC 통신에 사용되며 산업용 표준으로 자리잡음.

**Modbus 특징:**
- Master-Slave 구조
- 프로토콜 공개 및 무료
- 비트/워드(16비트) 단위 정보조작 용이

**Modbus 통신 종류:**

| 항목 | Modbus RTU | Modbus TCP/IP |
|------|------------|---------------|
| 전송방식 | Serial (RS-232/485) | Ethernet |
| Frame 형식 | Binary | TCP/IP Packet |
| 속도 | 9600-115200bps | 1Gbps |
| 응용분야 | 공장 자동화 | 원격 모니터링 |

### 3.2 실습 - 그리퍼 연결

**네트워크 설정:**
```bash
# PC에 그리퍼용 IP 추가
sudo nmcli connection modify "Wired connection 2" \
  ipv4.addresses 192.168.1.100/24 \
  ipv4.method manual

sudo nmcli connection down "Wired connection 2"
sudo nmcli connection up "Wired connection 2"

# 연결 확인
ping 192.168.1.1
```

**그리퍼 제어 코드 (pymodbus 2.5.3):**
```python
from onrobot import RG

# 그리퍼 연결
gripper = RG('192.168.1.1')

# 열기/닫기
gripper.open_gripper()
gripper.close_gripper()

# 특정 너비로 이동 (1/10mm 단위)
gripper.move(500)  # 50mm
```

**그리퍼 상태 플래그 (강의자료 p.29):**

| 비트 | 상태 플래그 | 설명 |
|-----|-----------|------|
| 0 | Busy | 1: 동작 중, 0: 대기 중 |
| 1 | Grip Detected | 1: 물체 감지됨 |
| 2-5 | Safety Switch | 안전 스위치 상태 |
| 6 | Safety Error | 전원 인가 시 안전 스위치 눌려 있음 |

---

## 4. Hand-Eye Calibration

### 4.1 이론 (강의자료 p.31-44)

**Camera Calibration이란?**
> 카메라의 내·외부 파라미터를 추정하는 것

| 파라미터 | 설명 | 사용 |
|---------|------|------|
| 내부 파라미터 | 초점 거리, 주점, 렌즈 왜곡 계수 | 왜곡 보정 |
| 외부 파라미터 | 카메라 위치/방향 | 3D 좌표 매핑 |

**Eye-in-Hand vs Eye-to-Hand:**

| 방식 | Eye-in-Hand | Eye-to-Hand |
|------|-------------|-------------|
| 카메라 위치 | 로봇 팔에 부착 | 고정된 외부 위치 |
| Checker Board 위치 | 고정된 공간 | 로봇 팔에 부착 |
| **오늘 사용** | ✅ | ❌ |

**Chain Rule (강의자료 p.32):**
```
Base → Target 변환이 항상 같다는 것을 이용하여 방정식을 풀 수 있음

T_base→camera = T_base→gripper × T_gripper→camera
              = T_base→target × T_target→camera (inverse)
```

### 4.2 실습 - 데이터 수집

**Checker Board 사양:**
- 크기: A4
- 내부 코너: 10 × 7 (또는 7 × 10)
- 정사각형 크기: 25mm

**데이터 수집 과정:**
```bash
# 1. 로봇 bringup
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.137.100 port:=12345

# 2. 직접 교시 모드 활성화
ros2 service call /dsr01m0609/system/set_robot_mode dsr_msgs2/srv/SetRobotMode "{robot_mode: 1}"

# 3. 데이터 수집 (DEVICE_NUMBER=10)
python3 data_recording.py
# 'q' 키로 이미지 + TCP 좌표 저장
# 20장 이상 수집
```

### 4.3 실습 - 캘리브레이션 실행

```bash
# handeye_calibration.py 실행
python3 handeye_calibration.py
```

**오늘 결과:**
- 수집 이미지: 54장
- 유효 이미지: 54장 (100%)
- 생성 파일: `T_gripper2camera.npy`

**변환 행렬 결과:**
```python
# T_gripper2camera (4x4 Matrix)
R_gripper2camera ≈ Identity (거의 회전 없음)
T_gripper2camera = [30.7mm, 57.6mm, -218.5mm]
```

### 4.4 OpenCV calibrateHandEye 메서드 (강의자료 p.42)

```python
cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,    # 로봇 TCP → Base
    R_target2cam, t_target2cam,         # 체커보드 → 카메라
    method=cv2.CALIB_HAND_EYE_PARK     # 0, 1, 2, 3, 4 시도 가능
)
```

| Method | 이름 | 특징 |
|--------|------|------|
| 0 | TSAI | 빠름, 일반적 |
| 1 | PARK | 정확도 높음 ✅ **사용** |
| 2 | HORAUD | - |
| 3 | ANDREFF | - |
| 4 | DANIILIDIS | - |

---

## 5. ROS_DOMAIN_ID 설정

### 5.1 개념 (강의자료 p.44 참조)
> ROS2 DDS 통신에서 네트워크 격리를 위한 ID. 같은 DOMAIN_ID를 가진 노드끼리만 통신

### 5.2 팀별 ID 할당

| 팀/조원 | ROS_DOMAIN_ID |
|--------|:-------------:|
| 1조 | 60 |
| 2조 | 61 |
| 3조 | 62 |
| 4조 | 63 |
| 5조 | 64 |

### 5.3 설정 방법
```bash
# ~/.bashrc에 추가
echo 'export ROS_DOMAIN_ID=60' >> ~/.bashrc
source ~/.bashrc

# 확인
echo $ROS_DOMAIN_ID
```

---

## 6. 트러블슈팅

### 6.1 카메라 접근 오류
**문제**: `cv2.VideoCapture()` 실패
```
Error: Could not open video device
```

**원인**: ROS2 RealSense 노드가 카메라 점유

**해결**:
```bash
# RealSense 노드 종료 후 OpenCV 사용
pkill -f realsense
python3 data_recording.py
```

### 6.2 Tool/TCP 설정 Hanging
**문제**: `set_tool()`, `set_tcp()` 호출 시 응답 없음

**원인**: 로봇 컨트롤러에서 이미 Tool/TCP 설정됨

**해결**:
```python
# 코드에서 주석 처리
# set_tool("Tool Weight")
# set_tcp("GripperTCP")
# → 티치펜던트에서 수동 설정
```

### 6.3 캘리브레이션 nan 값
**문제**: 변환 행렬에 `nan` 값 발생

**원인**: 이미지 품질 문제 (블러, 조명, 체커보드 부분 가림)

**해결**:
1. 이전 데이터 삭제: `rm -rf calibrate_data/ calibrate_data.json`
2. 조명 조절
3. 체커보드 전체가 보이도록 촬영
4. 다양한 각도에서 20장 이상 수집

### 6.4 그리퍼 연결 실패
**문제**: ModbusTcpClient 연결 실패

**원인**: 네트워크 설정 미완료

**해결**:
```bash
# IP 확인
ip addr show

# 그리퍼 서브넷 추가
sudo ip addr add 192.168.1.100/24 dev enp0s20f0u2
```

### 6.5 pymodbus 버전 오류
**문제**: pymodbus 3.x 버전 호환성 문제

**해결**:
```bash
pip3 uninstall pymodbus
pip3 install pymodbus==2.5.3
```

---

## 7. 실습 결과물

### 7.1 생성된 파일

```
day1/
├── 1_gripper_setup/
│   ├── onrobot.py              # RG2 그리퍼 제어 클래스
│   ├── test_gripper_*.py       # 그리퍼 테스트 스크립트
│   ├── ROS_DOMAIN_ID_GUIDE.md  # DOMAIN_ID 설정 가이드
│   └── setup_ros_domain_id.sh  # 자동 설정 스크립트
│
├── 2_calibration/
│   ├── data_recording.py       # 데이터 수집 스크립트
│   ├── handeye_calibration.py  # 캘리브레이션 실행
│   ├── test.py                 # Pick & Place 검증
│   ├── T_gripper2camera.npy    # 변환 행렬 (핵심 결과물)
│   └── README_CALIBRATION.md   # 캘리브레이션 가이드
│
└── docs/
    ├── index.html              # 슬라이드 뷰어
    └── ai_studio_code*.html    # 8장 슬라이드
```

### 7.2 핵심 결과: T_gripper2camera

```python
import numpy as np

T = np.load('T_gripper2camera.npy')
# 4x4 변환 행렬
# R: 거의 단위 행렬 (카메라가 그리퍼와 거의 평행)
# t: [30.7, 57.6, -218.5] mm (카메라 → 그리퍼 오프셋)
```

### 7.3 네트워크 구성

```
┌─────────────────┐      ┌─────────────────┐
│   Doosan M0609  │      │  OnRobot RG2    │
│ 192.168.137.100 │      │   192.168.1.1   │
└────────┬────────┘      └────────┬────────┘
         │                        │
         │ :12345                 │ :502 (Modbus)
         │                        │
    ┌────┴────────────────────────┴────┐
    │              PC                   │
    │  192.168.137.51 (로봇 서브넷)      │
    │  192.168.1.100 (그리퍼 서브넷)     │
    └───────────────────────────────────┘
```

---

## 8. 다음 단계

### Day 2 예정 (강의자료 Chapter 04)
- [ ] Camera Calibration 심화 (3차원 좌표 추정)
- [ ] Eye-to-Hand Calibration 비교
- [ ] 다양한 calibrateHandEye method 비교 (0-4)

### Day 3-5 예정 (강의자료 Chapter 05-06)
- [ ] Object Detection (YOLO, Ultralytics)
- [ ] 모델 학습 및 추론
- [ ] Pick and Place with Object Detection

### Day 6-8 예정 (강의자료 Chapter 07-08)
- [ ] Wakeup Word, STT
- [ ] LangChain 키워드 추출
- [ ] 음성 명령 기반 Pick and Place

---

## 📎 참고 자료

### 강의자료 링크
- [Ultralytics 설정](https://docs.ultralytics.com/quickstart/#understanding-settings)
- [YOLO 모델](https://docs.ultralytics.com/models/)
- [Roboflow 데이터셋](https://universe.roboflow.com/)

### GitHub 자료
- [OnRobot RG Python](https://github.com/takuya-ki/onrobot-rg/blob/main/src/onrobot.py)
- [Checker Board PDF](https://raw.githubusercontent.com/MarkHedleyJones/markhedleyjones.github.io/master/media/calibration-checkerboard-collection/Checkerboard-A4-25mm-10x7.pdf)

### 프로젝트 GitHub
- [Interactive Slides](https://taesla.github.io/rokey_c_1_collabo2/)
- [Repository](https://github.com/taesla/rokey_c_1_collabo2)

---

> 💡 **Tip**: 이 문서는 강의자료(이론)와 실습(실제)을 연계하여 정리한 것입니다.  
> 각 섹션의 "이론" 부분은 강의자료 페이지를, "실습" 부분은 오늘 수행한 내용을 담고 있습니다.
