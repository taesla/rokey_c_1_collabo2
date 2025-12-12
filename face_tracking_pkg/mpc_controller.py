#!/usr/bin/env python3
"""
Model Predictive Control (MPC) for Face Tracking
얼굴 추적을 위한 모델 예측 제어

동적 타겟을 추적하기 위한 Receding Horizon 방식
제약 조건을 명시적으로 처리하는 QP 최적화
"""
import numpy as np
import cvxpy as cp
from typing import Tuple, Optional


class MPCFaceTracker:
    """MPC 기반 얼굴 추적 컨트롤러"""
    
    def __init__(self, 
                 prediction_horizon=10, 
                 control_horizon=3,
                 dt=0.033):
        """
        초기화
        
        Args:
            prediction_horizon: 예측 스텝 수 (N)
            control_horizon: 제어 스텝 수 (M), M <= N
            dt: 샘플링 시간 (초)
        """
        self.N = prediction_horizon  # 예측 호라이즌
        self.M = control_horizon      # 제어 호라이즌
        self.dt = dt                  # 샘플링 시간
        
        # 시스템 모델 (간소화된 관절-TCP 야코비안)
        # Δtcp ≈ J * Δq
        self.A = np.eye(2)  # 상태: [y_tcp, z_tcp]
        self.B = np.array([
            [0.8, 0.0],    # J1 → y_tcp (좌우)
            [0.0, 0.5]     # J5 → z_tcp (상하)
        ])  # 야코비안 근사
        
        # 비용 함수 가중치 행렬
        self.Q = np.eye(2) * 100.0     # 추적 오차 페널티
        self.R = np.eye(2) * 1.0       # 제어 입력 크기 페널티
        self.S = np.eye(2) * 10.0      # 제어 변화율 페널티 (부드러움)
        
        # 관절 제약 조건
        self.j1_limits = {
            'min': -80.0,      # 최소 각도 (deg)
            'max': 5.0,        # 최대 각도 (deg)
            'vel_max': 50.0,   # 최대 속도 (deg/s)
            'acc_max': 100.0   # 최대 가속도 (deg/s²)
        }
        
        self.j5_limits = {
            'min': -200.0,
            'max': 200.0,
            'vel_max': 40.0,
            'acc_max': 80.0
        }
        
        # CVXPY 변수 (warm start를 위해 재사용)
        self.u = cp.Variable((self.N, 2))  # 제어 입력 시퀀스
        self.x = cp.Variable((self.N+1, 2))  # 상태 궤적
        
        # 이전 제어 입력 (초기값)
        self.prev_control = np.zeros(2)
        
    def solve(self, 
              current_state: np.ndarray,
              predicted_targets: np.ndarray,
              current_joints: np.ndarray,
              prev_control: Optional[np.ndarray] = None) -> np.ndarray:
        """
        MPC 최적화 문제 해결
        
        Args:
            current_state: [y_tcp, z_tcp] 현재 TCP 위치 (mm)
            predicted_targets: (N, 2) 예측된 타겟 궤적 [y, z] (mm)
            current_joints: [j1, j5] 현재 관절 각도 (deg)
            prev_control: [Δj1, Δj5] 이전 제어 입력 (deg)
        
        Returns:
            optimal_control: [Δj1, Δj5] 최적 제어 입력 (deg)
        """
        if prev_control is None:
            prev_control = self.prev_control
        
        # 제약 조건 리스트
        constraints = []
        
        # 초기 상태 고정
        constraints.append(self.x[0] == current_state)
        
        # 비용 함수 초기화
        cost = 0.0
        
        for k in range(self.N):
            # 상태 전이 제약: x[k+1] = A*x[k] + B*u[k]
            constraints.append(
                self.x[k+1] == self.A @ self.x[k] + self.B @ self.u[k]
            )
            
            # [1] 추적 오차 비용
            tracking_error = self.x[k+1] - predicted_targets[k]
            cost += cp.quad_form(tracking_error, self.Q)
            
            # [2] 제어 입력 크기 비용
            cost += cp.quad_form(self.u[k], self.R)
            
            # [3] 제어 변화율 비용 (부드러움, Jerk 최소화)
            if k == 0:
                delta_u = self.u[k] - prev_control
            else:
                delta_u = self.u[k] - self.u[k-1]
            cost += cp.quad_form(delta_u, self.S)
            
            # 제어 호라이즌 제한 (M 이후는 마지막 값 유지)
            if k >= self.M:
                constraints.append(self.u[k] == self.u[self.M-1])
            
            # 관절 각도 제약
            j1_next = current_joints[0] + cp.sum(self.u[:k+1, 0])
            j5_next = current_joints[1] + cp.sum(self.u[:k+1, 1])
            
            constraints.append(j1_next >= self.j1_limits['min'])
            constraints.append(j1_next <= self.j1_limits['max'])
            constraints.append(j5_next >= self.j5_limits['min'])
            constraints.append(j5_next <= self.j5_limits['max'])
            
            # 속도 제약 (제어 입력 = 각속도 * dt)
            max_delta_j1 = self.j1_limits['vel_max'] * self.dt
            max_delta_j5 = self.j5_limits['vel_max'] * self.dt
            
            constraints.append(self.u[k, 0] <= max_delta_j1)
            constraints.append(self.u[k, 0] >= -max_delta_j1)
            constraints.append(self.u[k, 1] <= max_delta_j5)
            constraints.append(self.u[k, 1] >= -max_delta_j5)
            
            # 가속도 제약 (제어 변화율 / dt)
            if k > 0:
                acc = (self.u[k] - self.u[k-1]) / self.dt
                constraints.append(acc[0] <= self.j1_limits['acc_max'])
                constraints.append(acc[0] >= -self.j1_limits['acc_max'])
                constraints.append(acc[1] <= self.j5_limits['acc_max'])
                constraints.append(acc[1] >= -self.j5_limits['acc_max'])
        
        # 최적화 문제 정의
        problem = cp.Problem(cp.Minimize(cost), constraints)
        
        # QP 문제 해결 (OSQP solver - 빠르고 robust)
        try:
            problem.solve(
                solver=cp.OSQP,
                warm_start=True,    # 이전 해를 초기값으로 사용
                max_iter=4000,      # 최대 반복 횟수
                eps_abs=1e-4,       # 절대 오차 허용치
                eps_rel=1e-4        # 상대 오차 허용치
            )
            
            if problem.status == cp.OPTIMAL or problem.status == cp.OPTIMAL_INACCURATE:
                # 첫 번째 제어 입력만 반환 (Receding Horizon)
                optimal_control = self.u.value[0].copy()
                self.prev_control = optimal_control
                return optimal_control
            else:
                print(f"⚠️ MPC 최적화 실패: {problem.status}")
                # Fallback: 이전 제어의 절반
                return prev_control * 0.5
                
        except Exception as e:
            print(f"❌ MPC 오류: {e}")
            # Fallback: 영 제어
            return np.zeros(2)
    
    def set_weights(self, Q=None, R=None, S=None):
        """
        비용 함수 가중치 업데이트
        
        Args:
            Q: 추적 오차 가중치 (높을수록 정확도 우선)
            R: 제어 입력 가중치 (높을수록 에너지 절약)
            S: 제어 변화율 가중치 (높을수록 부드러움 우선)
        """
        if Q is not None:
            self.Q = Q if isinstance(Q, np.ndarray) else np.eye(2) * Q
        if R is not None:
            self.R = R if isinstance(R, np.ndarray) else np.eye(2) * R
        if S is not None:
            self.S = S if isinstance(S, np.ndarray) else np.eye(2) * S
    
    def set_limits(self, j1_limits=None, j5_limits=None):
        """
        관절 제약 조건 업데이트
        
        Args:
            j1_limits: dict with 'min', 'max', 'vel_max', 'acc_max'
            j5_limits: dict with 'min', 'max', 'vel_max', 'acc_max'
        """
        if j1_limits is not None:
            self.j1_limits.update(j1_limits)
        if j5_limits is not None:
            self.j5_limits.update(j5_limits)
