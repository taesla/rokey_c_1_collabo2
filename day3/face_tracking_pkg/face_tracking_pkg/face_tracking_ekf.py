#!/usr/bin/env python3
"""
Extended Kalman Filter for Face Tracking
얼굴 위치 추적을 위한 확장 칼만 필터

상태 벡터: [x, y, z, vx, vy, vz, ax, ay, az]
측정 벡터: [x, y, z] (MediaPipe에서 받은 위치)
"""
import numpy as np
from filterpy.kalman import KalmanFilter


class FaceTrackingEKF:
    """얼굴 위치 추적용 확장 칼만 필터"""
    
    def __init__(self, dt=0.033):
        """
        초기화
        
        Args:
            dt: 샘플링 시간 (기본 30Hz = 0.033s)
        """
        self.dt = dt
        
        # 9차원 상태, 3차원 측정
        self.kf = KalmanFilter(dim_x=9, dim_z=3)
        
        # 상태 전이 행렬 (등가속도 모델)
        self.kf.F = np.array([
            [1, 0, 0, dt, 0,  0,  0.5*dt**2, 0,        0       ],
            [0, 1, 0, 0,  dt, 0,  0,        0.5*dt**2, 0       ],
            [0, 0, 1, 0,  0,  dt, 0,        0,        0.5*dt**2],
            [0, 0, 0, 1,  0,  0,  dt,       0,        0       ],
            [0, 0, 0, 0,  1,  0,  0,        dt,       0       ],
            [0, 0, 0, 0,  0,  1,  0,        0,        dt      ],
            [0, 0, 0, 0,  0,  0,  1,        0,        0       ],
            [0, 0, 0, 0,  0,  0,  0,        1,        0       ],
            [0, 0, 0, 0,  0,  0,  0,        0,        1       ]
        ])
        
        # 관측 행렬 (위치만 측정)
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0]
        ])
        
        # 프로세스 노이즈 공분산 (사람의 움직임 특성)
        q = 0.1  # 기본 노이즈
        self.kf.Q = np.eye(9) * q
        self.kf.Q[6:9, 6:9] *= 2.0  # 가속도 노이즈는 더 큼
        
        # 측정 노이즈 공분산 (MediaPipe 센서 노이즈)
        r = 10.0  # mm 단위 (MediaPipe의 떨림 정도)
        self.kf.R = np.eye(3) * r
        
        # 초기 불확실성
        self.kf.P *= 100.0
        
        # 초기 상태 (0으로 시작)
        self.kf.x = np.zeros(9)
        
        self.initialized = False
        
    def initialize(self, initial_position):
        """
        첫 측정값으로 초기화
        
        Args:
            initial_position: [x, y, z] (mm)
        """
        self.kf.x[:3] = initial_position
        self.kf.x[3:] = 0  # 속도, 가속도는 0
        self.initialized = True
        
    def predict(self):
        """한 스텝 예측"""
        self.kf.predict()
        
    def update(self, measurement):
        """
        측정값으로 상태 업데이트
        
        Args:
            measurement: [x, y, z] (mm)
        """
        if not self.initialized:
            self.initialize(measurement)
            return
        
        self.kf.update(measurement)
        
    def predict_trajectory(self, n_steps):
        """
        미래 N 스텝 궤적 예측
        
        Args:
            n_steps: 예측할 스텝 수
            
        Returns:
            trajectory: (n_steps, 3) array - 예측된 위치들
        """
        trajectory = []
        state = self.kf.x.copy()
        F = self.kf.F.copy()
        
        for i in range(n_steps):
            # 상태 전이
            state = F @ state
            trajectory.append(state[:3].copy())  # 위치만 저장
            
        return np.array(trajectory)
    
    def get_state(self):
        """
        현재 추정 상태 반환
        
        Returns:
            dict: 위치, 속도, 가속도
        """
        return {
            'position': self.kf.x[:3].copy(),
            'velocity': self.kf.x[3:6].copy(),
            'acceleration': self.kf.x[6:9].copy()
        }
    
    def get_position(self):
        """현재 추정 위치 [x, y, z]"""
        return self.kf.x[:3].copy()
    
    def get_velocity(self):
        """현재 추정 속도 [vx, vy, vz]"""
        return self.kf.x[3:6].copy()
    
    def get_acceleration(self):
        """현재 추정 가속도 [ax, ay, az]"""
        return self.kf.x[6:9].copy()
    
    def reset(self):
        """필터 리셋"""
        self.kf.x = np.zeros(9)
        self.kf.P = np.eye(9) * 100.0
        self.initialized = False
