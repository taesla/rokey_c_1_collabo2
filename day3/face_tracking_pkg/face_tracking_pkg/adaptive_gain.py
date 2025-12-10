#!/usr/bin/env python3
"""
Adaptive Gain Scheduler for Face Tracking
거리 기반 적응형 게인 스케줄러

멀리 있으면 빠르게, 가까이 있으면 천천히 정밀 제어
"""
import numpy as np


class AdaptiveGainScheduler:
    """거리 기반 적응형 게인 스케줄러"""
    
    def __init__(self, 
                 far_threshold=300.0, 
                 near_threshold=50.0,
                 far_gain=1.0,
                 near_gain=0.3,
                 transition_power=1.5):
        """
        초기화
        
        Args:
            far_threshold: 먼 거리 임계값 (mm)
            near_threshold: 가까운 거리 임계값 (mm)
            far_gain: 먼 거리에서의 게인
            near_gain: 가까운 거리에서의 게인
            transition_power: 전환 곡선의 지수 (높을수록 급격한 변화)
        """
        self.far_threshold = far_threshold
        self.near_threshold = near_threshold
        self.far_gain = far_gain
        self.near_gain = near_gain
        self.transition_power = transition_power
    
    def compute(self, distance: float) -> float:
        """
        거리에 따른 게인 계산
        
        Args:
            distance: 타겟까지의 거리 (mm)
        
        Returns:
            gain: 적응형 게인 (0.3 ~ 1.0)
        """
        if distance >= self.far_threshold:
            # 멀리 있으면 최대 게인 (빠르게 접근)
            return self.far_gain
        
        elif distance <= self.near_threshold:
            # 가까이 있으면 최소 게인 (정밀 제어)
            return self.near_gain
        
        else:
            # 중간 거리: 부드러운 전환
            # ratio: 0 (near) → 1 (far)
            ratio = (distance - self.near_threshold) / \
                    (self.far_threshold - self.near_threshold)
            
            # 비선형 전환 (3차 곡선 등)
            smooth_ratio = ratio ** self.transition_power
            
            # 선형 보간
            gain = self.near_gain + (self.far_gain - self.near_gain) * smooth_ratio
            
            return gain
    
    def compute_vector(self, error_vector: np.ndarray) -> float:
        """
        벡터 오차로부터 게인 계산
        
        Args:
            error_vector: [dx, dy, dz] 오차 벡터 (mm)
        
        Returns:
            gain: 적응형 게인
        """
        distance = np.linalg.norm(error_vector)
        return self.compute(distance)


class VelocityBasedGainScheduler:
    """속도 기반 게인 스케줄러 (고급)"""
    
    def __init__(self,
                 distance_scheduler: AdaptiveGainScheduler,
                 velocity_threshold=100.0):
        """
        초기화
        
        Args:
            distance_scheduler: 거리 기반 스케줄러
            velocity_threshold: 속도 임계값 (mm/s)
        """
        self.distance_scheduler = distance_scheduler
        self.velocity_threshold = velocity_threshold
    
    def compute(self, 
                distance: float, 
                relative_velocity: float) -> float:
        """
        거리 + 속도 기반 게인 계산
        
        Args:
            distance: 타겟까지의 거리 (mm)
            relative_velocity: 상대 속도 (양수=멀어짐, 음수=가까워짐) (mm/s)
        
        Returns:
            gain: 적응형 게인
        """
        # 기본 거리 기반 게인
        base_gain = self.distance_scheduler.compute(distance)
        
        # 속도 보정
        if relative_velocity > 0:
            # 타겟이 멀어지고 있으면 게인 증가 (빠르게 추적)
            velocity_factor = 1.0 + min(relative_velocity / self.velocity_threshold, 0.5)
        else:
            # 타겟이 가까워지고 있으면 게인 감소 (오버슈트 방지)
            velocity_factor = 1.0 + max(relative_velocity / self.velocity_threshold, -0.3)
        
        return base_gain * velocity_factor


class MultiZoneGainScheduler:
    """다중 구역 게인 스케줄러"""
    
    def __init__(self):
        """
        3개 구역으로 나누어 제어
        - Far zone (300mm+): 빠른 접근
        - Mid zone (50-300mm): 부드러운 추적
        - Near zone (0-50mm): 정밀 제어
        """
        self.zones = [
            {'threshold': 300.0, 'gain': 1.0,  'name': 'FAR'},
            {'threshold': 150.0, 'gain': 0.7,  'name': 'MID'},
            {'threshold': 50.0,  'gain': 0.4,  'name': 'NEAR'},
            {'threshold': 0.0,   'gain': 0.3,  'name': 'VERY_NEAR'}
        ]
    
    def compute(self, distance: float) -> tuple:
        """
        거리에 따른 게인 및 구역 정보
        
        Args:
            distance: 타겟까지의 거리 (mm)
        
        Returns:
            (gain, zone_name): 게인 값과 구역 이름
        """
        for zone in self.zones:
            if distance >= zone['threshold']:
                return zone['gain'], zone['name']
        
        # 최소 구역
        return self.zones[-1]['gain'], self.zones[-1]['name']
