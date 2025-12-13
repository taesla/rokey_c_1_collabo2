#!/usr/bin/env python3
"""
Joint-Space Face Tracking Node - 조인트 직접 제어 방식

핵심 철학:
  얼굴 추적은 "방향 추적" 문제 → TCP 위치가 아닌 조인트 각도로 직접 제어
  
장점:
  - IK(역기구학) 계산 없음 → 빠른 응답
  - 조인트 속도 한계 직접 활용 (J1: 150°/s, J4-J6: 225°/s)
  - 특이점(Singularity) 문제 없음
  - 단순 삼각함수 계산만 필요

제어 전략:
  J1: 수평 방향 추적 (베이스 회전) - 가장 중요!
  J4: 수직 방향 추적 (손목 피치)
  J6: 미세 좌우 보정 (이미지 X 기반)
  
  J2, J3, J5: 고정 (적절한 팔 자세 유지)

로봇 스펙 (Doosan M0609):
  - J1, J2: 150°/s (±360°)
  - J3: 180°/s (±160°)
  - J4, J5, J6: 225°/s (±360°, ±135°, ±360°)
  
Subscribed Topics:
  /face_tracking/marker_robot - 얼굴 3D 위치 (로봇 좌표계)
  /face_detection/faces - 얼굴 이미지 좌표 (J6 미세 조정용)
  /dsr01/joint_states - 현재 조인트 상태
"""
import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import DR_init

# EKF 필터 import
try:
    from face_tracking_pkg.face_tracking_ekf import FaceTrackingEKF
    EKF_AVAILABLE = True
except ImportError:
    EKF_AVAILABLE = False


class JointTrackingNode(Node):
    """조인트 공간 얼굴 추적 노드"""
    
    def __init__(self):
        super().__init__('joint_tracking_node')
        
        # ========================================
        # 파라미터 선언
        # ========================================
        self.declare_parameter('robot_id', 'dsr01')
        self.declare_parameter('robot_model', 'm0609')
        
        # 조인트 속도 제한 (deg/s) - 현실적인 부드러운 추적용
        self.declare_parameter('j1_vel_limit', 30.0)   # 베이스 회전 (느림)
        self.declare_parameter('j2_vel_limit', 20.0)   # 어깨 높이 (더 느림)
        self.declare_parameter('j3_vel_limit', 25.0)   # 팔 뮸기 (느림)
        self.declare_parameter('j4_vel_limit', 40.0)   # 손목 피치
        self.declare_parameter('j5_vel_limit', 40.0)   # 손목 롤 (좌우 회전)
        self.declare_parameter('j6_vel_limit', 30.0)   # 손목 요 (카메라 수평)
        
        # 제어 게인 (줄임)
        self.declare_parameter('j1_gain', 0.5)  # J1 반응성 (0~1)
        self.declare_parameter('j2_gain', 0.3)  # J2 반응성 (높이)
        self.declare_parameter('j3_gain', 0.3)  # J3 반응성 (거리)
        self.declare_parameter('j4_gain', 0.4)  # J4 반응성 (피치)
        self.declare_parameter('j5_gain', 0.3)  # J5 반응성 (좌우)
        self.declare_parameter('j6_gain', 0.2)  # J6 반응성 (수평 유지)
        
        # 데드존 (deg) - 떨림 방지
        self.declare_parameter('dead_zone_deg', 2.0)
        
        # TCP 오프셋 (그리퍼 + 카메라)
        self.declare_parameter('tcp_offset_z', 228.6)  # mm (RG2 그리퍼)
        
        # ========================================
        # 파라미터 로드
        # ========================================
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_model = self.get_parameter('robot_model').value
        
        self.j1_vel_limit = self.get_parameter('j1_vel_limit').value
        self.j2_vel_limit = self.get_parameter('j2_vel_limit').value
        self.j3_vel_limit = self.get_parameter('j3_vel_limit').value
        self.j4_vel_limit = self.get_parameter('j4_vel_limit').value
        self.j5_vel_limit = self.get_parameter('j5_vel_limit').value
        self.j6_vel_limit = self.get_parameter('j6_vel_limit').value
        
        self.j1_gain = self.get_parameter('j1_gain').value
        self.j2_gain = self.get_parameter('j2_gain').value
        self.j3_gain = self.get_parameter('j3_gain').value
        self.j4_gain = self.get_parameter('j4_gain').value
        self.j5_gain = self.get_parameter('j5_gain').value
        self.j6_gain = self.get_parameter('j6_gain').value
        
        self.dead_zone_deg = self.get_parameter('dead_zone_deg').value
        self.tcp_offset_z = self.get_parameter('tcp_offset_z').value
        
        # ========================================
        # 조인트 범위 제한 (로봇 스펙 테이블 기준)
        # ========================================
        # Doosan M0609 스펙
        self.joint_limits = {
            'j1': (0.0, 50.0),       # 제한: 0~50° (사용자 설정)
            'j2': (-95.0, 95.0),     # 스펙: ±95°
            'j3': (-160.0, 160.0),   # 스펙: ±160°
            'j4': (-360.0, 360.0),   # 스펙: ±360°
            'j5': (-135.0, 135.0),   # 스펙: ±135°
            'j6': (-360.0, 360.0),   # 스펙: ±360°
        }
        
        # 조인트 속도 제한 (로봇 스펙 테이블 기준, deg/s)
        self.joint_vel_limits = {
            'j1': 150.0,   # 스펙: 150°/s
            'j2': 150.0,   # 스펙: 150°/s
            'j3': 180.0,   # 스펙: 180°/s
            'j4': 225.0,   # 스펙: 225°/s
            'j5': 225.0,   # 스펙: 225°/s
            'j6': 225.0,   # 스펙: 225°/s
        }
        
        # ========================================
        # 최적 제어 가중치 (Optimal Control Weights)
        # ========================================
        # J = Σ w_i * (q_target - q_current)² + Σ r_i * q_dot²
        # w: 위치 오차 가중치 (클수록 빠르게 수렴)
        # r: 제어 입력 가중치 (클수록 부드럽게 움직임)
        
        self.w = [1.0, 0.0, 0.0, 0.8, 0.5, 0.0]  # 위치 오차 가중치 [J1~J6]
        # J1: 1.0 (수평 추적 - 중요)
        # J2: 0.0 (고정)
        # J3: 0.0 (고정)
        # J4: 0.8 (상하 추적)
        # J5: 0.5 (미세 조정)
        # J6: 0.0 (고정)
        
        self.r = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # 제어 입력 가중치 [J1~J6]
        # 클수록 속도 변화 억제 (부드러움)
        
        # ========================================
        # 로봇 파라미터 (Doosan M0609)
        # ========================================
        # 링크 길이 (mm) - 대략적인 값
        self.L1 = 135.0   # 베이스 높이
        self.L2 = 411.0   # 어깨 링크
        self.L3 = 368.0   # 팔꿈치 링크  
        self.L4 = 113.0   # 손목 링크
        
        # 목표 작업 거리 (mm) - 얼굴과의 이상적인 거리
        self.target_distance = 700.0  # 70cm
        
        # ========================================
        # 상태 변수 (State Machine) - 이산 목표점 추적
        # ========================================
        # IDLE: 대기 (얼굴 감지 시 → MOVING)
        # MOVING: 목표 위치로 이동 중 (완료 시 → WAITING)
        # WAITING: 이동 완료, 새 감지 대기 (감지 시 → MOVING)
        # RETURN_HOME: 시작 위치로 복귀
        self.state = "IDLE"
        
        # 타임아웃 설정 (초)
        self.waiting_timeout = 2.0   # WAITING 상태에서 감지 없으면 → RETURN_HOME
        self.waiting_start_time = None
        
        # 현재 조인트 상태 (deg)
        self.current_joints = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.joints_received = False
        
        # ========== 핵심: 이산 목표점 ===========
        # 목표 조인트 (deg) - 한 번 계산해서 저장, 이동 완료까지 유지
        self.command_joints = None  # 현재 명령된 목표
        self.command_time = None    # 명령 시간
        self.command_face_pos = None  # 명령 시점의 얼굴 위치
        
        # ========== First Detection Lock (FDL) ===========
        # 정적 물체용: 첨 감지 위치를 base_link 좌표계로 고정
        self.locked_target_pos = None  # 고정된 목표 3D 위치 (mm, base_link)
        self.locked_target_joints = None  # 고정된 목표 조인트 각도
        self.is_static_mode = True  # True: 정적 모드 (FDL), False: 동적 추적
        
        # 이동 완료 판정 기준 (deg)
        self.arrival_threshold = 3.0  # 3도 이내면 도착
        
        # 얼굴 3D 위치 (로봇 좌표계, mm)
        self.face_pos = None
        self.face_time = None
        
        # 마지막 유효 얼굴 위치 (Target Memory)
        self.last_valid_face_pos = None
        self.last_valid_face_time = None
        
        # 얼굴 이미지 좌표 (J6 미세 조정용)
        self.face_image_x = None
        self.image_center_x = 320.0  # 640x480 기준
        
        # 제어 주기
        self.control_period = 0.02  # 50Hz (20ms)
        self.last_control_time = time.time()
        
        # ========================================
        # EKF 필터 초기화 (자체 처리)
        # ========================================
        self.ekf = None
        if EKF_AVAILABLE:
            self.ekf = FaceTrackingEKF(dt=0.033, dim=3)  # 30Hz
            self.get_logger().info("✅ EKF 필터 초기화 완료")
        else:
            self.get_logger().warn("⚠️ EKF 필터 사용 불가 - 원본 마커 사용")
        
        # ========================================
        # ROS2 구독자/발행자
        # ========================================
        # 얼굴 3D 위치 구독 (TF2 변환된 마커 - 자체 EKF 적용)
        self.face_marker_topic = '/face_tracking/marker_robot'  # 원본 마커 구독
        self.face_marker_sub = self.create_subscription(
            Marker, self.face_marker_topic, self.face_marker_callback, 10)
        
        # 얼굴 이미지 좌표 구독 (J6 미세 조정)
        self.face_image_sub = self.create_subscription(
            Float32MultiArray, '/face_detection/faces', self.face_image_callback, 10)
        
        # 조인트 상태 구독
        self.joint_state_sub = self.create_subscription(
            JointState, '/dsr01/joint_states', self.joint_state_callback, 10)
        
        # 시각화 마커 발행
        self.aim_line_pub = self.create_publisher(
            Marker, '/joint_tracking/aim_line', 10)
        
        # ========================================
        # 초기화 로그
        # ========================================
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎯 Joint-Space Face Tracking Node")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  Robot: {self.robot_id} / {self.robot_model}")
        self.get_logger().info(f"  Control Period: {self.control_period*1000:.0f}ms (50Hz)")
        self.get_logger().info(f"  J1 (Horizontal): gain={self.j1_gain}, vel_limit={self.j1_vel_limit}°/s")
        self.get_logger().info(f"  J4 (Vertical): gain={self.j4_gain}, vel_limit={self.j4_vel_limit}°/s")
        self.get_logger().info(f"  J6 (Fine-tune): gain={self.j6_gain}, vel_limit={self.j6_vel_limit}°/s")
        self.get_logger().info(f"  Dead Zone: {self.dead_zone_deg}°")
        self.get_logger().info("  키: 's'=시작, 'h'=홈, 'q'=종료")
        self.get_logger().info("=" * 60)
    
    # ========================================
    # 콜백 함수들
    # ========================================
    def face_marker_callback(self, msg):
        """얼굴 3D 위치 수신 (로봇 좌표계) + EKF 필터링"""
        raw_pos = np.array([
            msg.pose.position.x * 1000.0,  # m → mm
            msg.pose.position.y * 1000.0,
            msg.pose.position.z * 1000.0
        ])
        
        # 유효성 검사: (0,0,0)이 아닌 경우에만 처리
        if np.linalg.norm(raw_pos) < 10.0:  # 1cm 미만은 무시
            return
        
        # ========================================
        # EKF 필터링 (자체 처리)
        # ========================================
        if self.ekf is not None:
            if not self.ekf.initialized:
                self.ekf.initialize(raw_pos.tolist())
                filtered_pos = raw_pos
            else:
                self.ekf.predict()
                self.ekf.update(raw_pos.tolist())
                filtered_pos = np.array(self.ekf.get_position())
        else:
            filtered_pos = raw_pos
        
        # 디버그: 마커 정보 확인
        self.get_logger().info(
            f"📡 마커수신 topic='{self.face_marker_topic}' EKF={'ON' if self.ekf else 'OFF'} | "
            f"Raw:({raw_pos[0]:.0f},{raw_pos[1]:.0f}) → Filtered:({filtered_pos[0]:.0f},{filtered_pos[1]:.0f})mm",
            throttle_duration_sec=2.0)
        
        # 필터링된 위치 저장
        self.face_pos = filtered_pos
        self.face_time = time.time()
        # 마지막 유효 위치 저장 (Target Memory)
        self.last_valid_face_pos = filtered_pos.copy()
        self.last_valid_face_time = time.time()
    
    def face_image_callback(self, msg):
        """얼굴 이미지 좌표 수신 (J6 미세 조정용)"""
        if len(msg.data) >= 2:
            self.face_image_x = msg.data[0]  # center_x
    
    def joint_state_callback(self, msg):
        """현재 조인트 상태 수신"""
        if len(msg.position) >= 6:
            # 라디안 → 도 변환
            new_joints = [math.degrees(p) for p in msg.position[:6]]
            
            # ========================================
            # 유효성 검사 1: 명백히 잘못된 값 무시
            # ========================================
            # 시작 위치가 [3.06, 2.84, 92.13, 86.07, -1.43, 8.33]인데
            # J3, J4가 거의 0이면 로봇이 아직 초기화 안 된 상태
            # (기계적 홈이 아니라 토픽 초기값)
            if abs(new_joints[2]) < 5.0 and abs(new_joints[3]) < 5.0:
                # J3≈0 AND J4≈0 → 초기화 전 상태, 무시
                return
            
            # ========================================
            # 유효성 검사 2: 급격한 변화 필터링
            # ========================================
            if self.joints_received:
                # 이전 값과 비교 - J4가 50° 이상 급변하면 무시 (노이즈)
                j4_diff = abs(new_joints[3] - self.current_joints[3])
                if j4_diff > 50.0:
                    self.get_logger().warn(
                        f"⚠️ 조인트 급변 무시: J4 {self.current_joints[3]:.1f}→{new_joints[3]:.1f} (Δ{j4_diff:.1f}°)",
                        throttle_duration_sec=0.5)
                    return  # 이 값은 무시
            
            self.current_joints = new_joints
            self.joints_received = True
    
    # ========================================
    # 목표 조인트 각도 계산 (얼굴 → 조인트)
    # ========================================
    def compute_desired_joints(self):
        """
        얼굴 위치에서 "절대 목표" 조인트 각도 계산
        
        핵심: base_link 좌표계의 목표 위치 → 절대 조인트 각도
        - 카메라가 목표를 정면으로 바라보는 조인트 각도
        - 현재 조인트와 무관하게 계산 (First Detection Lock용)
        
        Returns:
            q_desired: [J1, J2, J3, J4, J5, J6] 목표 각도 (deg)
        """
        if self.face_pos is None:
            return None
        
        fx, fy, fz = self.face_pos  # mm, base_link 기준
        
        # 유효성 검사
        if abs(fx) < 1 and abs(fy) < 1 and abs(fz) < 1:
            return None
        
        # 시작 조인트 값 (고정된 팔 자세)
        # J2, J3, J6는 시작 위치 그대로 유지
        start_joints = [3.06, 2.84, 92.13, 86.07, -1.43, 8.33]
        q = start_joints.copy()
        
        # ========================================
        # J1: 목표 방향 (절대 각도)
        # ========================================
        # base_link에서 목표까지의 수평 방향
        distance_xy = math.sqrt(fx**2 + fy**2)
        if distance_xy > 100:  # 10cm 이상
            # 목표 방향 = atan2(y, x)
            j1_target = math.degrees(math.atan2(fy, fx))
            q[0] = j1_target  # 절대 각도!
        
        # J2, J3: 시작 위치 유지 (팔 형태 고정)
        # q[1], q[2]는 start_joints 그대로
        
        # ========================================
        # J4: 목표 높이 방향 (절대 각도)
        # ========================================
        # 카메라가 목표를 바라보는 피치 각도
        # TCP 높이 ≈ 550mm (시작 자세 기준)
        tcp_z_approx = 550.0
        if distance_xy > 100:
            # 목표까지의 피치 각도
            pitch_to_target = math.degrees(math.atan2(fz - tcp_z_approx, distance_xy))
            # J4 = 시작값(86°) + 피치 보정
            # 피치가 +면 위를 봐야함 → J4 감소
            q[3] = start_joints[3] - pitch_to_target
        
        # ========================================
        # J5: 미세 조정 (이미지 기반)
        # ========================================
        if self.face_image_x is not None:
            error_pixel = self.face_image_x - self.image_center_x
            # 중앙에서 오른쪽에 있으면 J5 감소
            q[4] = start_joints[4] - error_pixel * 0.03
        
        # J6: 시작 위치 유지 (카메라 수평)
        if self.face_image_x is not None:
            error_pixel = self.face_image_x - self.image_center_x
            # 중앙에서 오른쪽에 있으면 J5 감소 (왼쪽으로 회전)
            q[4] = q[4] - error_pixel * 0.03  # pixel → deg
        
        # J6: 고정 (수평 유지)
        # q[5]는 그대로
        
        return q
    
    # ========================================
    # 최적 제어: 비용 함수 최소화
    # ========================================
    def calculate_optimal_control(self, q_desired, dt):
        """
        최적 제어 입력 계산
        
        비용 함수:
            J = Σ w_i * (q_desired_i - q_current_i)² + Σ r_i * q_dot_i²
        
        제약 조건:
            - |q_dot_i| ≤ q_dot_max_i (속도 제한, 로봇 스펙)
            - q_min_i ≤ q_next_i ≤ q_max_i (범위 제한, 로봇 스펙)
        
        해석적 해 (Closed-form solution):
            최적 q_dot = w_i / (w_i + r_i) * (q_desired - q_current) / dt
            
        Args:
            q_desired: 목표 조인트 각도 [6] (deg)
            dt: 제어 주기 (sec)
        
        Returns:
            q_next: 다음 목표 조인트 각도 [6] (deg)
        """
        if q_desired is None:
            return None
        
        q_current = self.current_joints
        q_next = [0.0] * 6
        
        limits_keys = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        
        for i in range(6):
            # 위치 오차
            error = q_desired[i] - q_current[i]
            
            # 가중치가 0이면 현재값 유지
            if self.w[i] == 0:
                q_next[i] = q_current[i]
                continue
            
            # 최적 속도 계산 (비용 함수 미분 = 0)
            # dJ/d(q_dot) = 2*r*q_dot - 2*w*error/dt = 0
            # q_dot_optimal = w/(r) * error / dt
            # 하지만 안정성을 위해 (w + r) 로 정규화
            optimal_gain = self.w[i] / (self.w[i] + self.r[i])
            q_dot_optimal = optimal_gain * error / dt
            
            # 속도 제한 적용 (로봇 스펙)
            vel_limit = self.joint_vel_limits[limits_keys[i]]
            q_dot_clamped = max(-vel_limit, min(vel_limit, q_dot_optimal))
            
            # 다음 조인트 각도
            q_next_raw = q_current[i] + q_dot_clamped * dt
            
            # 범위 제한 적용 (로봇 스펙)
            q_min, q_max = self.joint_limits[limits_keys[i]]
            q_next[i] = max(q_min, min(q_max, q_next_raw))
        
        return q_next
    
    # ========================================
    # 통합 목표 계산 (기존 인터페이스 유지)
    # ========================================
    def calculate_target_joints(self, dt=0.02):
        """
        최적 제어를 통한 목표 조인트 계산
        
        Args:
            dt: 제어 주기 (default 20ms)
        
        Returns:
            최적화된 목표 조인트 각도 [6]
        """
        q_desired = self.compute_desired_joints()
        if q_desired is None:
            return None
        
        q_optimal = self.calculate_optimal_control(q_desired, dt)
        return q_optimal
    
    # ========================================
    # 속도 제한 적용
    # ========================================
    def apply_velocity_limits(self, current, target, dt):
        """
        조인트 속도 제한 적용
        
        Args:
            current: 현재 조인트 각도 [6]
            target: 목표 조인트 각도 [6]
            dt: 시간 간격 (초)
        
        Returns:
            속도 제한이 적용된 목표 조인트 [6]
        """
        vel_limits = [
            self.j1_vel_limit,  # J1: 베이스 회전 (30°/s)
            self.j2_vel_limit,  # J2: 어깨 높이 (20°/s)
            self.j3_vel_limit,  # J3: 팔 뻗기 (25°/s)
            self.j4_vel_limit,  # J4: 손목 피치 (40°/s)
            self.j5_vel_limit,  # J5: 손목 좌우 (40°/s)
            self.j6_vel_limit,  # J6: 카메라 수평 (30°/s)
        ]
        
        result = current.copy()
        
        for i in range(6):
            error = target[i] - current[i]
            
            # 데드존 적용 (모든 움직이는 조인트)
            if abs(error) < self.dead_zone_deg:
                continue
            
            # 최대 이동량 계산
            max_delta = vel_limits[i] * dt
            
            # 속도 제한 적용
            if abs(error) > max_delta:
                delta = max_delta if error > 0 else -max_delta
            else:
                delta = error
            
            result[i] = current[i] + delta
        
        return result


def main():
    """메인 함수"""
    rclpy.init()
    
    # 추적 노드 먼저 생성
    node = JointTrackingNode()
    
    # DR_init 설정
    DR_init.__dsr__id = node.robot_id
    DR_init.__dsr__model = node.robot_model
    
    # DSR 노드 생성 (로봇 제어용)
    dsr_node = rclpy.create_node("dsr_joint_control", namespace=node.robot_id)
    DR_init.__dsr__node = dsr_node
    
    # DSR 함수 import
    try:
        from DSR_ROBOT2 import movej, amovej, get_current_posj, get_current_posx, mwait
        print("✅ DSR 모듈 import 성공")
    except ImportError as e:
        print(f"❌ DSR 모듈 import 실패: {e}")
        sys.exit(1)
    
    # 멀티스레드 실행
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(dsr_node)
    
    print("\n" + "="*60)
    print("🎯 Joint-Space Face Tracking")
    print("="*60)
    print("  's': 추적 시작")
    print("  'h': 홈 위치")
    print("  'q': 종료")
    print("="*60)
    
    # 홈 위치 / 시작 위치
    home_joints = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
    # 추적 시작 자세 (robot_control_node.py와 동일)
    start_joints = [3.06, 2.84, 92.13, 86.07, -1.43, 8.33]
    
    try:
        while rclpy.ok():
            # ROS 콜백 처리
            executor.spin_once(timeout_sec=0.001)
            
            # 키보드 입력 (논블로킹)
            import select
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.readline().strip().lower()
                
                if key == 'q':
                    print("\n종료합니다...")
                    break
                elif key == 'h':
                    print("\n🏠 홈 위치로 이동...")
                    node.state = "IDLE"
                    movej(home_joints, vel=30, acc=30)
                    mwait()
                    print("✅ 홈 위치 도착")
                elif key == 's':
                    if node.state == "IDLE":
                        print("\n📍 시작 위치로 이동 중...")
                        movej(start_joints, vel=30, acc=30)
                        mwait()
                        print("✅ 시작 위치 도착!")
                        
                        # 조인트 값 동기화 대기 (최대 2초)
                        print("⏳ 조인트 상태 동기화 중...")
                        node.joints_received = False  # 리셋
                        sync_start = time.time()
                        while time.time() - sync_start < 2.0:
                            executor.spin_once(timeout_sec=0.05)
                            if node.joints_received:
                                print(f"✅ 조인트 동기화 완료: J4={node.current_joints[3]:.1f}°")
                                break
                        
                        if not node.joints_received:
                            print("⚠️ 조인트 동기화 실패, 시작 위치로 초기화")
                            node.current_joints = start_joints.copy()
                            node.joints_received = True
                        
                        print("🎯 추적 시작! (얼굴 감지 시 자동 추적)")
                        node.state = "TRACKING"  # WAITING 대신 TRACKING으로 시작
                        node.waiting_start_time = time.time()  # 타임아웃 시작
                        node.command_joints = None
                    else:
                        print("\n⏸️ 추적 중지")
                        node.state = "IDLE"
                        node.command_joints = None
            
            # ========================================
            # 연속 추적 상태 머신 (Continuous Tracking)
            # ========================================
            # 핵심: 이동 중에도 목표 갱신 → 부드러운 추적
            # ========================================
            current_time = time.time()
            
            # 조인트 수신 확인
            if not node.joints_received:
                continue
            
            # 얼굴 감지 상태 확인
            face_detected = False
            if node.face_time is not None:
                time_since_face = current_time - node.face_time
                if time_since_face < 0.5:  # 0.5초 이내 감지
                    face_detected = True
            
            # ========================================
            # TRACKING: 연속 추적 모드 (이동 중에도 목표 갱신)
            # ========================================
            if node.state == "TRACKING":
                # ========================================
                # 정적 모드 (First Detection Lock)
                # ========================================
                # 첨 감지 시 목표를 고정하고, 도착할 때까지 유지
                # ========================================
                
                if node.locked_target_joints is None:
                    # 아직 목표 고정 안됨 → 첨 감지 대기
                    if face_detected:
                        # 첨 감지! → 목표 고정 (Lock)
                        target = node.compute_desired_joints()
                        if target is not None:
                            node.locked_target_pos = node.face_pos.copy()
                            node.locked_target_joints = target
                            node.command_joints = target
                            node.command_time = current_time
                            
                            # 이동 명령
                            try:
                                amovej(target, vel=30.0, acc=50.0)
                                node.get_logger().info(
                                    f"🔒 목표 고정! J1:{target[0]:+.1f}° J4:{target[3]:+.1f}° | "
                                    f"Pos:({node.face_pos[0]:.0f},{node.face_pos[1]:.0f},{node.face_pos[2]:.0f})")
                            except Exception as e:
                                node.get_logger().error(f"amovej 오류: {e}")
                            
                            node.waiting_start_time = None
                    else:
                        # 감지 없음 → 타임아웃 체크
                        if node.waiting_start_time is None:
                            node.waiting_start_time = current_time
                        wait_duration = current_time - node.waiting_start_time
                        if wait_duration > node.waiting_timeout:
                            node.get_logger().info(f"⏰ {node.waiting_timeout}초 감지 없음 → RETURN_HOME")
                            node.state = "RETURN_HOME"
                
                else:
                    # 목표 고정됨 → 도착 확인 (새 감지 무시!)
                    arrived = True
                    for i in [0, 3, 4]:  # J1, J4, J5
                        error = abs(node.current_joints[i] - node.locked_target_joints[i])
                        if error > node.arrival_threshold:
                            arrived = False
                            break
                    
                    if arrived:
                        # 도착! → 재감지 대기
                        node.get_logger().info(
                            f"✅ 도착! J1:{node.current_joints[0]:+.1f}° J4:{node.current_joints[3]:+.1f}°")
                        
                        # 목표 해제 (Unlock) → 재감지 가능
                        node.locked_target_joints = None
                        node.locked_target_pos = None
                        node.command_joints = None
                        node.waiting_start_time = current_time  # 타임아웃 시작
                    else:
                        # 아직 이동 중 → 고정된 목표로 계속 (새 감지 무시)
                        node.get_logger().info(
                            f"🚀 이동중 J1:{node.current_joints[0]:+.1f}→{node.locked_target_joints[0]:+.1f}° "
                            f"J4:{node.current_joints[3]:+.1f}→{node.locked_target_joints[3]:+.1f}° | "
                            f"고정위치:({node.locked_target_pos[0]:.0f},{node.locked_target_pos[1]:.0f})",
                            throttle_duration_sec=0.5)
            
            # ========================================
            # WAITING: 시작 위치에서 얼굴 감지 대기
            # ========================================
            elif node.state == "WAITING":
                if face_detected:
                    # 얼굴 감지! → TRACKING 시작
                    node.get_logger().info("👤 얼굴 감지! → TRACKING 시작")
                    node.state = "TRACKING"
                    node.command_joints = None
                    node.waiting_start_time = None
                else:
                    # 얼굴 없음 - 타임아웃 체크
                    if node.waiting_start_time is None:
                        node.waiting_start_time = current_time
                    
                    wait_duration = current_time - node.waiting_start_time
                    if wait_duration > node.waiting_timeout:
                        node.get_logger().info(f"⏰ {node.waiting_timeout}초 감지 없음 → RETURN_HOME")
                        node.state = "RETURN_HOME"
            
            # ========================================
            # RETURN_HOME: 시작 위치로 복귀
            # ========================================
            elif node.state == "RETURN_HOME":
                node.get_logger().info("🏠 시작 위치로 복귀 중...")
                movej(start_joints, vel=30, acc=30)
                mwait()
                node.get_logger().info("✅ 시작 위치 도착! 얼굴 감지 대기...")
                node.state = "WAITING"  # IDLE 대신 WAITING으로 → 계속 감지 대기
                node.command_joints = None
                node.waiting_start_time = None
            
            # ========================================
            # IDLE: 대기 ('s' 키로 시작)
            # ========================================
            elif node.state == "IDLE":
                pass  # 's' 키 입력 대기
            
            time.sleep(0.001)
    
    except KeyboardInterrupt:
        print("\n키보드 인터럽트")
    finally:
        executor.shutdown()
        node.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("정상 종료")


if __name__ == "__main__":
    main()
