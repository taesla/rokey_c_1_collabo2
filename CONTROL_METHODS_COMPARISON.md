# 헤드 트래킹 로봇 제어 방식 비교

## 📊 제어 방식 종합 비교표

| 항목 | Joint Space Control | **Cartesian Space Control** ⭐ | Visual Servoing | MPC (Model Predictive Control) |
|------|---------------------|-------------------------------|-----------------|--------------------------------|
| **제어 공간** | 관절 공간 (θ₁, θ₂, ..., θ₆) | 작업 공간 (x, y, z, rx, ry, rz) | 이미지 공간 → 작업 공간 | 작업 공간 + 예측 경로 |
| **구현 난이도** | ⭐⭐ 쉬움 | ⭐⭐⭐ 보통 | ⭐⭐⭐⭐ 어려움 | ⭐⭐⭐⭐⭐ 매우 어려움 |
| **계산 복잡도** | O(1) - 단순 관절 각도 계산 | O(n) - 역기구학 필요 | O(n²) - Jacobian 계산 | O(n³) - 최적화 문제 |
| **실시간성 (30Hz)** | ✅ 매우 우수 (0.1ms) | ✅ 우수 (1-5ms) | ⚠️ 보통 (5-10ms) | ❌ 불안정 (10-50ms) |
| **경로 부드러움** | ⚠️ 관절 공간 선형 (작업공간 비선형) | ✅ 직선 경로 가능 | ✅ 매우 부드러움 | ✅ 최적 경로 |
| **정확도** | ⚠️ TCP 위치 오차 누적 | ✅ 목표점 정확 도달 | ✅✅ 피드백 루프로 높은 정확도 | ✅✅ 예측 제어로 최고 정확도 |
| **특이점 회피** | ✅ 자연스럽게 회피 | ⚠️ 특이점 근처 불안정 | ⚠️ Damped Least Squares 필요 | ✅ 제약조건으로 회피 |
| **속도 제어** | ✅ 관절 속도 직접 제어 | ✅ TCP 속도 직관적 | ✅ 이미지 속도 기반 | ✅✅ 최적 속도 프로파일 |
| **안전성** | ✅ 관절 한계 명확 | ⚠️ 작업공간 한계 검증 필요 | ⚠️ 불안정 진동 위험 | ✅ 제약조건 내장 |
| **외란 대응** | ⚠️ 보통 | ⚠️ 보통 | ✅ 피드백 루프로 우수 | ✅✅ 예측으로 선제 대응 |
| **튜닝 파라미터** | Kp (1개) | Kp, vmax, deadzone (3개) | Kp, Ki, λ (damping) (5개+) | Q, R, horizon (10개+) |
| **직관성** | ⚠️ 관절 각도 이해 어려움 | ✅ 3D 좌표 직관적 | ✅ 시각적 피드백 명확 | ⚠️ 수학적 배경 필요 |
| **Overshoot** | ⚠️ 보통 | ⚠️ 보통 | ✅ 피드백으로 최소화 | ✅✅ 예측으로 거의 없음 |
| **적용 시나리오** | 단순 관절 제어, 빠른 동작 | **일반 추적 작업** | 정밀 조립, 용접 | 복잡한 제약 조건, 최적화 필요 |

---

## 🎯 각 방식 상세 분석

### 1. Joint Space Control (관절 공간 제어)

**원리:**
```python
# 관절 각도로 직접 제어
θ_target = θ_current + K_p * error_joint
movej(θ_target)
```

**장점:**
- ✅ 구현 초간단 (역기구학 불필요)
- ✅ 관절 한계 명확 (θ_min, θ_max)
- ✅ 특이점 자연 회피
- ✅ 가장 빠른 계산 속도

**단점:**
- ❌ TCP 경로 예측 불가 (관절 공간 선형 ≠ 작업 공간 선형)
- ❌ 목표 위치 직선 도달 불가능
- ❌ 사람이 보기에 부자연스러운 동작
- ❌ 속도 제어 직관성 부족

**적용 사례:**
- 단순 pick-and-place (경로 중요하지 않음)
- 고속 동작 (계산 시간 최소화)
- 관절 단위 제어가 필요한 경우

---

### 2. Cartesian Space Control (작업 공간 제어) ⭐ **현재 선택**

**원리:**
```python
# TCP 위치 오차로 속도 생성
error = target_xyz - current_tcp_xyz
velocity = K_p * error  # 비례 제어
velocity = clip(velocity, -v_max, v_max)
movel_speed(velocity)  # 직선 속도 제어
```

**장점:**
- ✅ 직선 경로 보장 (predictable)
- ✅ TCP 속도 직관적 제어 (mm/s)
- ✅ 사람이 보기에 자연스러운 동작
- ✅ 구현 복잡도 적절 (역기구학 내장 함수 사용)
- ✅ 디버깅 쉬움 (3D 좌표 명확)

**단점:**
- ⚠️ 특이점 근처 불안정 (singularity avoidance 필요)
- ⚠️ 역기구학 계산 오버헤드 (1-5ms)
- ⚠️ 관절 한계 간접 관리

**핵심 파라미터:**
```python
K_p = 0.4              # 비례 게인 (응답 속도)
v_max = 200            # 최대 속도 [mm/s]
dead_zone = 10         # 데드존 [mm]
control_freq = 30      # 제어 주파수 [Hz]
safety_margin = 50     # 안전 여유 [mm]
```

**적용 사례:**
- ✅ **헤드 트래킹** (부드러운 추적)
- ✅ 협동 로봇 작업 (사람 주변 안전)
- ✅ 직선 경로 작업 (레이저 커팅)

---

### 3. Visual Servoing (시각 서보잉)

**원리:**
```python
# 이미지 특징점으로 직접 제어
s = extract_features(image)  # 현재 이미지 특징
s_desired = target_features   # 목표 이미지 특징
error = s - s_desired

# Jacobian: 이미지 → 로봇 속도 매핑
J = compute_interaction_matrix()
velocity = -λ * pinv(J) * error
```

**장점:**
- ✅ 카메라 캘리브레이션 오차 무시
- ✅ 이미지 피드백 루프로 높은 정확도
- ✅ 외란에 강건 (실시간 보정)
- ✅ 3D 위치 추정 불필요 (이미지만으로 제어)

**단점:**
- ❌ Jacobian 계산 복잡 (수치 미분 or 모델 기반)
- ❌ Gain 튜닝 민감 (발산 위험)
- ❌ 카메라 프레임율 의존 (30Hz 제약)
- ❌ 지역 최소값 함정 (local minima)
- ❌ 특징점 소실 시 제어 불가

**적용 사례:**
- 정밀 조립 (peg-in-hole)
- 용접 심 추적 (seam tracking)
- 비전 기반 그리핑

---

### 4. MPC (Model Predictive Control) - **구현 실패**

**원리:**
```python
# 예측 구간(horizon)에서 최적 제어 시퀀스 계산
for t in range(horizon):
    x[t+1] = f(x[t], u[t])  # 시스템 모델
    cost += ||x[t] - target||² + ||u[t]||²

u_optimal = minimize(cost, constraints)
apply(u_optimal[0])  # 첫 번째 제어만 적용
```

**장점:**
- ✅✅ 미래 예측으로 최적 경로
- ✅✅ 제약조건 내장 (관절 한계, 속도, 가속도)
- ✅✅ Overshoot 최소화
- ✅ 외란 선제 대응 (robustness)
- ✅ 다목적 최적화 (여러 목표 동시 만족)

**단점:**
- ❌❌ 구현 매우 복잡 (최적화 라이브러리 필요)
- ❌❌ 실시간 계산 어려움 (30Hz 달성 실패)
- ❌ 로봇 동역학 모델 필요 (파라미터 식별)
- ❌ 튜닝 파라미터 많음 (Q, R, horizon, constraint)
- ❌ 디버깅 어려움 (블랙박스 최적화)

**구현 실패 원인:**
```
1. Doosan API의 실시간 동역학 모델 부재
2. 최적화 solver 계산 시간 > 33ms (30Hz 불가)
3. 제약조건 formulation 복잡도
4. CasADi/ACADO 등 라이브러리 의존성
```

**적용 사례:**
- 산업용 CNC (경로 최적화)
- 자율 주행 (장애물 회피)
- 드론 비행 제어

---

## 📈 성능 지표 비교 (30Hz 헤드 트래킹 기준)

| 지표 | Joint Space | **Cartesian Space** | Visual Servoing | MPC |
|------|-------------|---------------------|-----------------|-----|
| 계산 시간 | 0.1ms | 2-3ms | 8-10ms | 30-100ms ❌ |
| 위치 정확도 | ±5mm | ±2mm | ±1mm | ±0.5mm |
| 응답 속도 | 빠름 (50ms) | 보통 (100ms) | 느림 (200ms) | 매우 느림 (500ms) |
| 안정성 | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ (튜닝 민감) | ⭐⭐⭐⭐⭐ (이론상) |
| 구현 시간 | 1일 | **2일** | 1주 | 2주+ |
| 유지보수성 | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |

---

## 🎯 최종 선택: Cartesian Space Control (Velocity-based)

### 선택 이유:
1. ✅ **구현 복잡도 vs 성능 균형** - 2일 내 구현 가능, 충분한 성능
2. ✅ **30Hz 실시간 달성 가능** - 계산 시간 2-3ms
3. ✅ **부드러운 추적** - 직선 경로, 자연스러운 동작
4. ✅ **디버깅 용이** - 3D 좌표 직관적, RViz 시각화 쉬움
5. ✅ **안전성** - Dead zone, 속도 제한, 안전 영역 검증 명확

### 구현 전략:
```python
def track_face(target_pos):
    # Phase 1: 기본 비례 제어
    error = target_pos - current_tcp
    velocity = K_p * error
    velocity = clip(velocity, -v_max, v_max)
    
    # Phase 2: 안전 검증
    if not is_safe_position(target_pos):
        return
    
    # Phase 3: Dead zone
    if norm(error) < dead_zone:
        return
    
    # Phase 4: 실행
    movel(target_pos, vel=velocity_norm)
```

---

## 📝 참고: 방식별 제어 루프 주기

| 방식 | 권장 제어 주기 | 이유 |
|------|---------------|------|
| Joint Space | 10-50 Hz | 관절 속도 제한 |
| Cartesian Space | **20-30 Hz** | TCP 속도 제한, 역기구학 계산 |
| Visual Servoing | 10-20 Hz | 카메라 프레임율, Jacobian 계산 |
| MPC | 5-10 Hz | 최적화 계산 시간 |

---

*생성일: 2025-12-12*  
*프로젝트: Doosan m0609 헤드 트래킹 시스템*
