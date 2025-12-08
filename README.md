# 🤖 Rokey C팀 1조 - 협동로봇 프로젝트

> **음성 명령으로 물체를 인식하고 이동시키는 협동로봇 시스템**

## 🎯 프로젝트 목표

사람의 음성 명령을 인식하고, 해당 물체를 찾아 지정된 장소로 이동시키는 로봇 시스템 구축

## 📅 학습 일정

| Day | 주제 | 상태 | 문서 |
|:---:|------|:----:|------|
| 1 | Gripper 설정 & Hand-Eye Calibration | ✅ 완료 | [📖 Day 1](day1/README.md) |
| 2-5 | Object Detection (YOLO) | 🔜 예정 | - |
| 6-7 | Voice Processing (STT, Wakeup Word) | 🔜 예정 | - |
| 8+ | 시스템 통합 & LangChain | 🔜 예정 | - |

## 🔗 바로가기

- 🎨 **[Interactive Slides](https://taesla.github.io/rokey_c_1_collabo2/)** - Day 1 학습 슬라이드
- 📚 **[Day 1 학습 정리](day1/Day1_Summary.md)** - 강의 + 실습 통합 문서
- 📋 **[강의자료 PDF](docs/협동로봇2%20강의자료.pdf)** - 원본 강의자료

## 🛠️ 사용 장비

| 장비 | IP/모델 |
|------|---------|
| 로봇팔 | Doosan M0609 (192.168.137.100) |
| 그리퍼 | OnRobot RG2 (192.168.1.1) |
| 카메라 | Intel RealSense D435i |

## 👥 팀 정보

- **조**: Rokey C팀 1조
- **ROS_DOMAIN_ID**: 60

---

## 📁 프로젝트 구조

```
rokey_c_1_collabo2/
├── README.md                 # 이 파일
├── docs/                     # GitHub Pages (슬라이드, PDF)
└── day1/                     # Day 1 학습 자료
    ├── README.md             # Day 1 상세 문서
    ├── Day1_Summary.md       # 학습 정리
    ├── 1_gripper_setup/      # 그리퍼 코드
    └── 2_calibration/        # 캘리브레이션 코드
```

## 📄 License

MIT License
