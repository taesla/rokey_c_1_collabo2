#!/usr/bin/env python3
"""
Integrated Face Tracking Controller
MediaPipe + EKF + MPC + Adaptive Gain 통합 컨트롤러

전체 파이프라인:
  MediaPipe → EKF → MPC → Adaptive Gain → Robot
"""
import numpy as np
from typing import Optional, Dict

from .face_tracking_ekf import FaceTrackingEKF
from .mpc_controller import MPCFaceTracker
from .adaptive_gain import AdaptiveGainScheduler


class IntegratedFaceTrackingController:
    """통합 얼굴 추적 컨트롤러"""
    
    def __init__(self,
                 prediction_horizon=10,
                 control_horizon=3,
                 dt=0.033):
        """
        초기화
        
        Args:
            prediction_horizon: MPC 예측 호라이즌
            control_horizon: MPC 제어 호라이즌
            dt: 샘플링 시간 (초)
        """
        # 서브시스템 초기화
        self.ekf = FaceTrackingEKF(dt=dt)
        self.mpc = MPCFaceTracker(
            prediction_horizon=prediction_horizon,
            control_horizon=control_horizon,
            dt=dt
        )
        self.gain_scheduler = AdaptiveGainScheduler(
            far_threshold=300.0,
            near_threshold=50.0,
            far_gain=1.0,
            near_gain=0.3
        )
        
        # 상태 변수
        self.prev_control = np.zeros(2)
        self.initialized = False
        self.dt = dt
        
    def update(self,
               face_position_raw: np.ndarray,
               current_tcp: np.ndarray,
               current_joints: np.ndarray) -> np.ndarray:
        """
        한 제어 주기 업데이트
        
        Args:
            face_position_raw: [x, y, z] MediaPipe에서 받은 얼굴 위치 (mm)
            current_tcp: [x, y, z] 현재 TCP 위치 (mm)
            current_joints: [j1, j2, j3, j4, j5, j6] 현재 관절 각도 (deg)
        
        Returns:
            target_joints: [j1, j2, j3, j4, j5, j6] 목표 관절 각도 (deg)
        """
        # [1단계] 칼만 필터 업데이트
        self.ekf.predict()
        self.ekf.update(face_position_raw)
        
        # [2단계] 미래 궤적 예측 (MPC 호라이즌만큼)
        predicted_trajectory = self.ekf.predict_trajectory(n_steps=self.mpc.N)
        
        # [3단계] MPC 입력 준비
        current_state = np.array([current_tcp[1], current_tcp[2]])  # [y, z]
        predicted_targets = predicted_trajectory[:, 1:]  # [y, z]만 사용
        current_j1j5 = np.array([current_joints[0], current_joints[4]])
        
        # [4단계] MPC 최적 제어 계산
        optimal_control = self.mpc.solve(
            current_state=current_state,
            predicted_targets=predicted_targets,
            current_joints=current_j1j5,
            prev_control=self.prev_control
        )
        
        # [5단계] 적응형 게인 적용
        error = face_position_raw - current_tcp
        distance = np.linalg.norm(error)
        adaptive_gain = self.gain_scheduler.compute(distance)
        
        scaled_control = optimal_control * adaptive_gain
        
        # [6단계] 관절 각도 업데이트
        target_joints = current_joints.copy()
        target_joints[0] += scaled_control[0]  # J1
        target_joints[4] += scaled_control[1]  # J5
        
        # 상태 저장
        self.prev_control = scaled_control
        self.initialized = True
        
        return target_joints
    
    def get_diagnostics(self) -> Dict:
        """
        진단 정보 반환
        
        Returns:
            dict: EKF 상태, MPC 제어 등
        """
        ekf_state = self.ekf.get_state()
        
        return {
            'ekf_position': ekf_state['position'],
            'ekf_velocity': ekf_state['velocity'],
            'ekf_acceleration': ekf_state['acceleration'],
            'prev_control': self.prev_control,
            'initialized': self.initialized
        }
    
    def reset(self):
        """컨트롤러 리셋"""
        self.ekf.reset()
        self.prev_control = np.zeros(2)
        self.initialized = False
    
    def tune_mpc(self, Q=None, R=None, S=None):
        """
        MPC 가중치 튜닝
        
        Args:
            Q: 추적 오차 가중치 (높을수록 정확도 우선)
            R: 제어 입력 가중치 (높을수록 에너지 절약)
            S: 제어 변화율 가중치 (높을수록 부드러움)
        """
        self.mpc.set_weights(Q=Q, R=R, S=S)
    
    def tune_gain_scheduler(self, 
                           far_threshold=None,
                           near_threshold=None,
                           far_gain=None,
                           near_gain=None):
        """
        게인 스케줄러 튜닝
        
        Args:
            far_threshold: 먼 거리 임계값 (mm)
            near_threshold: 가까운 거리 임계값 (mm)
            far_gain: 먼 거리 게인
            near_gain: 가까운 거리 게인
        """
        if far_threshold is not None:
            self.gain_scheduler.far_threshold = far_threshold
        if near_threshold is not None:
            self.gain_scheduler.near_threshold = near_threshold
        if far_gain is not None:
            self.gain_scheduler.far_gain = far_gain
        if near_gain is not None:
            self.gain_scheduler.near_gain = near_gain


class SimplifiedController:
    """간소화된 컨트롤러 (MPC 없이 EKF만 사용)"""
    
    def __init__(self, dt=0.033):
        """EKF + 적응형 게인만 사용하는 간단한 버전"""
        self.ekf = FaceTrackingEKF(dt=dt)
        self.gain_scheduler = AdaptiveGainScheduler()
        
        # PID 게인
        self.Kp = np.array([0.15, 0.10])  # [J1, J5]
        self.Kd = np.array([0.05, 0.03])
        
    def update(self, face_position_raw, current_tcp, current_joints):
        """간단한 PD 제어"""
        # 칼만 필터
        self.ekf.predict()
        self.ekf.update(face_position_raw)
        
        filtered_pos = self.ekf.get_position()
        filtered_vel = self.ekf.get_velocity()
        
        # PD 제어
        error = filtered_pos - current_tcp
        error_yz = np.array([error[1], error[2]])
        vel_yz = np.array([filtered_vel[1], filtered_vel[2]])
        
        control = self.Kp * error_yz + self.Kd * vel_yz
        
        # 적응형 게인
        distance = np.linalg.norm(error)
        gain = self.gain_scheduler.compute(distance)
        
        scaled_control = control * gain
        
        # 관절 업데이트
        target_joints = current_joints.copy()
        target_joints[0] += scaled_control[0]
        target_joints[4] += scaled_control[1]
        
        return target_joints
