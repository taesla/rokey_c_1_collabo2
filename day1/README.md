# Day 1: Robot System Setup & Calibration

## 📅 작업 날짜
2025년 12월 8일

## 🎯 목표
- **1부: 그리퍼 시스템 구축**
  - OnRobot RG2 그리퍼 네트워크 연결 및 제어
  - Modbus TCP 통신 구현
  - 그리퍼 기본/고급 기능 테스트
  - ROS2 DOMAIN_ID 설정 (조별 네트워크 격리)

- **2부: 캘리브레이션 ✅ 완료**
  - RealSense D435i 카메라 설정
  - Hand-Eye Calibration (54장 데이터, T_gripper2camera.npy 생성)
  - Pick & Place 검증

---

## 📖 Documentation

| 문서 | 설명 |
|------|------|
| 📚 **[Day1 학습 정리](Day1_Summary.md)** | **강의자료 + 실습 통합 정리 (필독!)** |
| 🎯 [Interactive Slides](https://taesla.github.io/rokey_c_1_collabo2/) | 두 카테고리: Day 1 학습 정리 (9장) + ROS_DOMAIN_ID (8장) |
| 🔧 [Gripper Test Guide](1_gripper_setup/README_GRIPPER_TESTS.md) | 그리퍼 테스트 상세 가이드 |
| 📋 [ROS_DOMAIN_ID Guide](1_gripper_setup/ROS_DOMAIN_ID_GUIDE.md) | ROS2 네트워크 격리 설정 (팀별 개별 ID) |
| 📷 [Calibration Guide](2_calibration/README_CALIBRATION.md) | Hand-Eye Calibration 전체 가이드 |

---

## 📁 프로젝트 구조

```
day1/
├── README.md                          # 메인 문서 (이 파일)
├── Day1_Summary.md                    # 📚 강의자료 + 실습 통합 정리
├── 협동로봇2 강의자료.pdf               # 원본 강의자료
│
├── docs/                              # 슬라이드 원본
│   ├── index.html                     # 두 카테고리 슬라이드 뷰어
│   ├── day1_slides/                   # 📚 Day 1 학습 정리 (9장)
│   └── ros_domain_slides/             # 🌐 ROS_DOMAIN_ID Guide (8장)
│
├── 1_gripper_setup/                   # 1부: 그리퍼 시스템
│   ├── onrobot.py                     # OnRobot RG2 제어 클래스
│   ├── test_gripper_*.py              # 그리퍼 테스트 스크립트 (3개)
│   ├── README_GRIPPER_TESTS.md        # 테스트 가이드
│   ├── setup_ros_domain_id.sh         # ROS_DOMAIN_ID 설정 스크립트
│   ├── ROS_DOMAIN_ID_GUIDE.md         # DOMAIN_ID 설정 가이드 (v2.0 팀별 ID)
│   └── ROS_DOMAIN_ID_SCENARIOS.md     # 분산 시스템 시나리오
│
└── 2_calibration/                     # 2부: 캘리브레이션 ✅ 완료
    ├── data_recording.py              # 데이터 수집 스크립트
    ├── handeye_calibration.py         # 캘리브레이션 실행
    ├── test.py                        # Pick & Place 검증
    ├── realsense.py                   # ROS2 카메라 노드
    ├── onrobot.py                     # 그리퍼 제어
    ├── T_gripper2camera.npy           # 🎯 변환 행렬 결과물
    └── README_CALIBRATION.md          # 캘리브레이션 가이드
```

---

---

## 📚 Part 1: 그리퍼 시스템 구축

### 🔧 환경 설정

#### 1. 필수 패키지 설치

```bash
# pymodbus (그리퍼 통신용)
pip3 install pymodbus==2.5.3

# OpenCV, matplotlib (이미지 처리용)
pip3 install opencv-python matplotlib
```

#### 2. 네트워크 설정

```bash
# 그리퍼 연결 (USB-이더넷)
# PC IP: 192.168.1.100
# 그리퍼 IP: 192.168.1.1

sudo nmcli connection modify "Wired connection 2" \
  ipv4.addresses 192.168.1.100/24 \
  ipv4.method manual

sudo nmcli connection down "Wired connection 2"
sudo nmcli connection up "Wired connection 2"

# 연결 확인
ping 192.168.1.1
```

#### 3. ROS_DOMAIN_ID 설정 (조별 격리)

```bash
# 자동 설정
cd 1_gripper_setup
./setup_ros_domain_id.sh

# 수동 설정
echo 'export ROS_DOMAIN_ID=60' >> ~/.bashrc
source ~/.bashrc
```

### 🚀 사용법

#### 1. 그리퍼 연결 확인

```bash
cd 1_gripper_setup
python3 test_gripper_connection.py
```

**출력 예시:**
```
그리퍼 연결 시도: 192.168.1.1:502
✓ 그리퍼 연결 성공!
그리퍼 상태: [0, 0, 0, 0, 0, 0, 0]
현재 그리퍼 너비: 101.6 mm
```

#### 2. 기본 동작 테스트

```bash
cd 1_gripper_setup
python3 test_gripper_basic.py
```

**테스트 항목:**
- 그리퍼 열기 (100mm)
- 그리퍼 닫기 (0mm)
- 특정 위치로 이동 (50mm)

#### 3. 고급 기능 테스트

```bash
cd 1_gripper_setup
python3 test_gripper_advanced.py
```

**테스트 항목:**
- Fingertip Offset 확인
- 다양한 힘으로 그리핑 (50N, 200N)
- 정밀 위치 제어 (80, 60, 40, 20mm)
- 상태 플래그 분석

### 📚 주요 클래스 및 함수

### `onrobot.RG` 클래스

```python
from onrobot import RG

# 그리퍼 초기화
gripper = RG("rg2", "192.168.1.1", "502")

# 기본 동작
gripper.open_gripper()           # 열기
gripper.close_gripper()          # 닫기
gripper.move_gripper(500, 1000)  # 50mm로 이동, 100N

# 상태 확인
width = gripper.get_width_with_offset()  # 현재 너비
status = gripper.get_status()             # 상태 플래그 [7개]

# 연결 종료
gripper.close_connection()
```

### 주요 메서드

| 메서드 | 설명 | 예시 |
|--------|------|------|
| `open_gripper(force_val)` | 최대로 열기 | `gripper.open_gripper(400)` |
| `close_gripper(force_val)` | 완전히 닫기 | `gripper.close_gripper(1000)` |
| `move_gripper(width, force)` | 특정 위치로 이동 | `move_gripper(500, 1000)` |
| `get_width_with_offset()` | 현재 너비 (mm) | `width = gripper.get_width_with_offset()` |
| `get_status()` | 7개 상태 플래그 | `status = gripper.get_status()` |

---

## 🌐 네트워크 구성

```
PC (노트북)                    OnRobot RG2 그리퍼
192.168.1.100  ◄──이더넷──►  192.168.1.1
(USB-이더넷)                   (Modbus TCP 502)
```

---

## 🎓 학습 내용

### 1. Modbus TCP 통신
- pymodbus 라이브러리 사용
- Holding Register 읽기/쓰기
- 그리퍼 제어 명령 프로토콜

### 2. 그리퍼 제어
- 위치 제어 (0~110mm)
- 힘 제어 (0~400N)
- 상태 모니터링 (busy, grip detected 등)

#### 3. ROS2 네트워크
- DOMAIN_ID를 통한 조별 격리
- 분산 시스템 아키텍처
- 토픽 기반 통신

### 🐛 문제 해결

#### 1. 그리퍼 연결 실패
```bash
# 네트워크 확인
ping 192.168.1.1

# 인터페이스 확인
ip addr show

# 재연결
sudo nmcli connection down "Wired connection 2"
sudo nmcli connection up "Wired connection 2"
```

#### 2. pymodbus ImportError
```bash
# 올바른 버전 설치
pip3 uninstall pymodbus
pip3 install pymodbus==2.5.3
```

#### 3. ROS2 토픽 안 보임
```bash
# DOMAIN_ID 확인
echo $ROS_DOMAIN_ID

# 재설정
export ROS_DOMAIN_ID=60
source ~/.bashrc
```

### 📊 테스트 결과

**성능 지표:**
- 위치 정밀도: ±2mm
- 힘 제어 범위: 0~400N (RG2)
- 응답 시간: ~500ms
- 통신 안정성: 100% (LAN)

**검증 완료 항목:**
- ✅ 네트워크 연결
- ✅ 기본 열기/닫기
- ✅ 정밀 위치 제어
- ✅ 힘 제어
- ✅ 상태 모니터링
- ✅ Fingertip Offset 측정

---

## 📚 Part 2: 캘리브레이션 ✅ 완료

### 🎯 완료된 작업
- ✅ RealSense D435i 카메라 설정 (/dev/video10, aligned_depth)
- ✅ Hand-Eye Calibration (Eye-in-Hand 방식)
- ✅ 54장 체커보드 이미지 수집
- ✅ T_gripper2camera.npy 변환 행렬 생성
- ✅ Pick & Place 검증

### 📊 캘리브레이션 결과
```python
# T_gripper2camera 변환 행렬
R_gripper2camera ≈ 단위 행렬 (카메라-그리퍼 거의 평행)
T_gripper2camera = [30.7mm, 57.6mm, -218.5mm]
```

### 🔧 사용법
```bash
# 1. 데이터 수집
cd 2_calibration
python3 data_recording.py   # 'q' 키로 저장, 20장 이상 수집

# 2. 캘리브레이션 실행
python3 handeye_calibration.py

# 3. 결과 검증
python3 test.py
```

> 📖 상세 가이드: [README_CALIBRATION.md](2_calibration/README_CALIBRATION.md)

---

## 📝 다음 단계 (Day 2~)

- [x] ~~로봇팔과 그리퍼 통합 제어~~ ✅
- [x] ~~RealSense 카메라 연동~~ ✅
- [x] ~~Hand-Eye Calibration~~ ✅
- [x] ~~Pick and Place 시나리오 구현~~ ✅
- [ ] Object Detection (YOLO, Ultralytics)
- [ ] 음성 명령 처리 (Wakeup Word, STT)
- [ ] AI 기반 Pick & Place 통합

---

## 👥 팀원 정보

- 조: Rokey C팀 1조
- ROS_DOMAIN_ID: 60 (팀별 60~64 사용)
- 장비: 
  - 노트북 4대
  - Doosan M0609 로봇팔 (192.168.137.100)
  - OnRobot RG2 그리퍼 (192.168.1.1)
  - Intel RealSense D435i 카메라

---

## 📖 참고 자료

- [OnRobot RG2 Manual](https://onrobot.com/en/products/rg2-gripper)
- [pymodbus Documentation](https://pymodbus.readthedocs.io/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

---

## 📄 License

MIT License

---

## ✨ 기여자

- 작성일: 2025-12-08
- 작성자: Rokey Team
