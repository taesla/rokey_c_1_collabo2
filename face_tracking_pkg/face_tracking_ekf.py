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
    
    def __init__(self, dt=0.033, dim=3):
        """
        초기화
        
        Args:
            dt: 샘플링 시간 (기본 30Hz = 0.033s)
            dim: 측정 차원 (2=2D 픽셀, 3=3D 위치)
        """
        self.dt = dt
        self.dim = dim
        
        # 상태 차원: dim * 3 (위치, 속도, 가속도)
        dim_x = dim * 3
        dim_z = dim
        
        self.kf = KalmanFilter(dim_x=dim_x, dim_z=dim_z)
        
        # 상태 전이 행렬 (등가속도 모델)
        if dim == 2:
            # 2D: [x, y, vx, vy, ax, ay]
            self.kf.F = np.array([
                [1, 0, dt, 0,  0.5*dt**2, 0       ],
                [0, 1, 0,  dt, 0,        0.5*dt**2],
                [0, 0, 1,  0,  dt,       0        ],
                [0, 0, 0,  1,  0,        dt       ],
                [0, 0, 0,  0,  1,        0        ],
                [0, 0, 0,  0,  0,        1        ]
            ])
            
            # 관측 행렬 (위치만 측정)
            self.kf.H = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0]
            ])
        else:
            # 3D: [x, y, z, vx, vy, vz, ax, ay, az]
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
        self.kf.Q = np.eye(dim_x) * q
        if dim == 3:
            self.kf.Q[6:9, 6:9] *= 2.0  # 가속도 노이즈는 더 큼
        else:
            self.kf.Q[4:6, 4:6] *= 2.0  # 2D 가속도
        
        # 측정 노이즈 공분산 (MediaPipe 센서 노이즈)
        # 낮을수록 센서 신뢰 → 반응 빠름 | 높을수록 예측 신뢰 → 부드러움
        if dim == 3:
            r = 5.0  # mm 단위 (기존 10.0 → 5.0: 반응속도 개선)
        else:
            r = 2.0  # 픽셀 단위 (기존 3.0 → 2.0: 반응속도 개선)
        self.kf.R = np.eye(dim_z) * r
        
        # 초기 불확실성
        self.kf.P *= 100.0
        
        # 초기 상태 (0으로 시작)
        self.kf.x = np.zeros(dim_x)
        
        self.initialized = False
        
    def initialize(self, initial_position):
        """
        첫 측정값으로 초기화
        
        Args:
            initial_position: [x, y] or [x, y, z]
        """
        self.kf.x[:self.dim] = initial_position
        self.kf.x[self.dim:] = 0  # 속도, 가속도는 0
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
            trajectory: (n_steps, dim) array - 예측된 위치들
        """
        trajectory = []
        state = self.kf.x.copy()
        F = self.kf.F.copy()
        
        for i in range(n_steps):
            # 상태 전이
            state = F @ state
            trajectory.append(state[:self.dim].copy())  # 위치만 저장
            
        return np.array(trajectory)
    
    def get_state(self):
        """
        현재 추정 상태 반환
        
        Returns:
            dict: 위치, 속도, 가속도
        """
        return {
            'position': self.kf.x[:self.dim].copy(),
            'velocity': self.kf.x[self.dim:self.dim*2].copy(),
            'acceleration': self.kf.x[self.dim*2:self.dim*3].copy()
        }
    
    def get_position(self):
        """현재 추정 위치"""
        return self.kf.x[:self.dim].copy()
    
    def get_velocity(self):
        """현재 추정 속도"""
        return self.kf.x[self.dim:self.dim*2].copy()
    
    def get_acceleration(self):
        """현재 추정 가속도"""
        return self.kf.x[self.dim*2:self.dim*3].copy()
    
    def reset(self):
        """필터 리셋"""
        dim_x = self.dim * 3
        self.kf.x = np.zeros(dim_x)
        self.kf.P = np.eye(dim_x) * 100.0
        self.initialized = False
