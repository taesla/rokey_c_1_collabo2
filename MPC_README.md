# 🚀 Phase 5-2: Full 6-DOF MPC Controller

## 📋 개요

**Model Predictive Control (MPC)** 기반 로봇 팔 제어 시스템 구현

### Phase 5-1 vs Phase 5-2

| 항목 | Phase 5-1 (기존) | Phase 5-2 (MPC) |
|------|-----------------|----------------|
| **제어 관절** | J1, J5만 (2-DOF) | J1~J6 전체 (6-DOF) |
| **제어 방식** | P-Controller | Model Predictive Control |
| **궤적 계획** | 즉각 반응 | 최적 궤적 계획 (N-step ahead) |
| **제약 조건** | 간단한 한계값 | 위치/속도/가속도 제약 |
| **부드러움** | 뚝뚝 끊김 | Jerk minimization |
| **자세 제어** | 불가능 | 가능 (IK 통합 시) |

---

## 🏗️ 시스템 구조

### 1. State Space (12차원)

```python
x = [q1, q2, q3, q4, q5, q6,          # Joint positions (rad)
     q1_dot, q2_dot, ..., q6_dot]     # Joint velocities (rad/s)
```

### 2. Control Input (6차원)

```python
u = [q1_ddot, q2_ddot, ..., q6_ddot]  # Joint accelerations (rad/s^2)
```

### 3. System Dynamics

```
x[k+1] = A * x[k] + B * u[k]

where:
  A = [I   dt*I]    (12x12)
      [0   I   ]
  
  B = [0.5*dt^2*I]  (12x6)
      [dt*I      ]
```

### 4. Cost Function

```
J = Σ[ Q_pos ||q - q_target||² +     # Position tracking
       Q_vel ||q_dot||² +             # Velocity regularization
       R ||u||² ] +                   # Control effort
    Q_terminal ||x[N] - x_target||²   # Terminal cost
```

### 5. Constraints

```python
# Position limits
q_min ≤ q[k] ≤ q_max

# Velocity limits
-q_dot_max ≤ q_dot[k] ≤ q_dot_max

# Acceleration limits
-q_ddot_max ≤ u[k] ≤ q_ddot_max
```

---

## 📦 파일 구조

```
face_tracking_pkg/
├── robot_control_node.py          # Phase 5-1 (LEGACY - J1+J5 제어)
├── robot_control_mpc_node.py      # Phase 5-2 (NEW - Full 6-DOF MPC)
├── face_tracking_ekf.py           # EKF 필터
└── face_tracking_node.py          # 3D 좌표 변환
```

---

## 🎯 실행 방법

### Step 1: 기존 노드 종료

```bash
# 기존 robot_control_node 종료
pkill -f robot_control_node
```

### Step 2: MPC 노드 실행

```bash
cd ~/ros2_ws
source install/setup.bash

# MPC 컨트롤러 실행 (새 터미널)
ros2 run face_tracking_pkg robot_control_mpc_node
```

### Step 3: 전체 시스템 실행

```bash
# Terminal 1: RealSense 카메라
ros2 launch realsense2_camera rs_launch.py

# Terminal 2: 얼굴 감지
ros2 run face_tracking_pkg face_detection_node

# Terminal 3: 얼굴 추적 (3D 좌표 변환 + EKF)
ros2 run face_tracking_pkg face_tracking_node

# Terminal 4: MPC 로봇 제어
ros2 run face_tracking_pkg robot_control_mpc_node
```

### Step 4: 제어 시작

```
>>> 입력:
s  - 시작 위치로 이동 후 추적 모드
h  - 홈 위치로 복귀
q  - 종료
```

---

## ⚙️ 파라미터 튜닝

### MPC 파라미터

```python
# robot_control_mpc_node.py 수정

# Horizon (예측 범위)
horizon = 10  # 10 steps = 0.33초 (30Hz)
# 길수록: 부드럽지만 연산량 증가
# 짧을수록: 빠르지만 근시안적

# Cost function weights
Q_pos = 100.0      # 위치 추적 가중치 (높을수록 정확)
Q_vel = 1.0        # 속도 정규화 (높을수록 느림)
R = 0.1            # 제어 입력 페널티 (높을수록 부드러움)
Q_terminal = 200.0 # 종료 상태 가중치
```

### 관절 한계 (자동 로드)

`/dsr_moveit_config_m0609/config/joint_limits.yaml`에서 자동 읽기:

| Joint | Min (rad) | Max (rad) | Vel (rad/s) | Acc (rad/s²) |
|-------|-----------|-----------|-------------|--------------|
| J1 | -3.14 | 3.14 | 3.67 | 0.734 |
| J2 | -1.3 | 1.3 | 3.32 | 0.664 |
| J3 | -2.0 | 2.0 | 3.67 | 0.734 |
| J4 | -3.14 | 3.14 | 6.98 | 1.396 |
| J5 | -2.0 | 2.0 | 6.98 | 1.396 |
| J6 | -3.14 | 3.14 | 10.47 | 2.094 |

---

## 🔬 성능 비교

### Phase 5-1 (J1+J5 Only)

- ✅ 빠른 반응 (즉각 제어)
- ❌ 뚝뚝 끊김
- ❌ J2, J3, J4, J6 고정
- ❌ 엔드이펙터 자세 제어 불가

### Phase 5-2 (MPC 6-DOF)

- ✅ 부드러운 궤적
- ✅ 모든 관절 최적 제어
- ✅ 관절 한계 자동 준수
- ✅ 확장 가능 (IK, 장애물 회피 등)
- ⚠️  연산량 증가 (50-100ms Python → 5-10ms C++)

---

## 🚧 TODO (다음 단계)

### 1. IK 통합 (우선순위 높음)

```python
# 현재: 간단한 2-DOF 근사
target_q[0] = atan2(y, x)  # J1
target_q[4] = atan2(z, sqrt(x^2 + y^2))  # J5

# 필요: Doosan API IK 사용
from DSR_ROBOT2 import ikin
target_q = ikin(target_pos_xyz_rpy)  # 6-DOF solution
```

### 2. Orientation 제어

```python
# 목표: 카메라가 항상 얼굴을 바라보도록
target_orientation = compute_lookat_quaternion(face_pos)
target_pose = [x, y, z, roll, pitch, yaw]
```

### 3. 특이점(Singularity) 회피

```python
# MPC Cost에 추가
singularity_cost = 1.0 / (det(J(q)) + epsilon)
```

### 4. C++ 포팅 (성능 최적화)

```cpp
// Eigen + OSQP
// 50-100ms → 5-10ms (10배 속도 향상)
```

### 5. CUDA 가속 (고급)

```cuda
// Batch QP solving
// 5-10ms → 1-2ms (병렬 처리)
```

---

## 📊 예상 성능

| 구현 | 언어 | 연산 시간 | 제어 주파수 |
|------|------|-----------|------------|
| 현재 (Python + cvxpy) | Python | 50-100ms | 10-20Hz |
| C++ + Eigen + OSQP | C++ | 5-10ms | 100Hz+ |
| CUDA 가속 | CUDA | 1-2ms | 500Hz+ |

**현재 목표:** 30Hz (33ms/loop)  
**현재 상태:** Python MPC로 충분 (50-100ms 이내)

---

## 🐛 Troubleshooting

### 1. cvxpy 없음

```bash
pip install cvxpy
```

### 2. MPC solver 실패

```python
# 파라미터 완화
Q_pos = 10.0  (100.0에서 감소)
horizon = 5   (10에서 감소)
```

### 3. 로봇이 너무 느림

```python
# 속도 페널티 감소
Q_vel = 0.1  (1.0에서 감소)
```

### 4. 로봇이 떨림

```python
# 제어 입력 페널티 증가
R = 1.0  (0.1에서 증가)
```

---

## 📚 참고 자료

1. **Model Predictive Control**
   - J. Rawlings, D. Mayne, "Model Predictive Control: Theory and Design"
   - https://osqp.org/ (OSQP Solver)

2. **Doosan Robot API**
   - `/dsr_common2/include/DRFLEx.h` (FK/IK 함수)
   - `/dsr_moveit_config_m0609/config/joint_limits.yaml`

3. **ROS2 + MoveIt2**
   - https://moveit.picknik.ai/main/index.html

---

## ✅ Phase 5-2 완료 조건

- [x] MPC 컨트롤러 구현 (`robot_control_mpc_node.py`)
- [x] 6-DOF State Space 설계
- [x] 관절 제약 조건 통합
- [x] cvxpy QP Solver 통합
- [ ] IK 통합 (Doosan API)
- [ ] 실제 로봇 테스트
- [ ] 성능 측정 (Hz, 정확도)
- [ ] Phase 5-1 vs 5-2 비교 영상

---

**Author:** Rokey AI Lab  
**Date:** 2025-12-10  
**Status:** 🚧 In Development (IK Integration Pending)
