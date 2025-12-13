#!/usr/bin/env python3
"""
Joint-Space Face Tracking Node - 조인트 직접 제어

핵심 철학:
    얼굴 추적 = "방향 추적" 문제 → 조인트 각도로 직접 제어
    
장점:
    - IK 계산 없음 → 빠른 응답
    - 특이점(Singularity) 문제 없음
    - 조인트 속도 한계 직접 활용

제어 전략:
    J1: 수평 방향 (베이스 회전) - 가장 중요
    J4: 수직 방향 (손목 피치)
    J5: 미세 좌우 보정
    J2, J3, J6: 고정 (팔 자세 유지)

Subscribed Topics:
    /face_tracking/marker_robot - 얼굴 3D 위치 (로봇 좌표계)
    /face_detection/faces - 얼굴 이미지 좌표
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

from ..tracking.ekf_filter import EKFFilter
from ..utils.constants import (
    JOINT_LIMITS, JOINT_VEL_MAX, HOME_JOINTS, START_JOINTS,
    DEAD_ZONE_DEG, ARRIVAL_THRESHOLD_DEG
)


class JointTrackingNode(Node):
    """조인트 공간 얼굴 추적 노드"""
    
    def __init__(self):
        super().__init__('joint_tracking_node')
        
        # ========================================
        # 파라미터
        # ========================================
        self.declare_parameter('robot_id', 'dsr01')
        self.declare_parameter('robot_model', 'm0609')
        self.declare_parameter('j1_vel_limit', 30.0)
        self.declare_parameter('j2_vel_limit', 20.0)  # 원본 복원
        self.declare_parameter('j3_vel_limit', 25.0)  # 원본 복원
        self.declare_parameter('j4_vel_limit', 40.0)
        self.declare_parameter('j5_vel_limit', 40.0)
        self.declare_parameter('j6_vel_limit', 30.0)  # 원본 복원
        self.declare_parameter('j1_gain', 0.5)
        self.declare_parameter('j4_gain', 0.4)
        self.declare_parameter('j5_gain', 0.3)
        self.declare_parameter('dead_zone_deg', 2.0)  # 원본 복원
        
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_model = self.get_parameter('robot_model').value
        self.j1_vel_limit = self.get_parameter('j1_vel_limit').value
        self.j2_vel_limit = self.get_parameter('j2_vel_limit').value
        self.j3_vel_limit = self.get_parameter('j3_vel_limit').value
        self.j4_vel_limit = self.get_parameter('j4_vel_limit').value
        self.j5_vel_limit = self.get_parameter('j5_vel_limit').value
        self.j6_vel_limit = self.get_parameter('j6_vel_limit').value
        self.j1_gain = self.get_parameter('j1_gain').value
        self.j4_gain = self.get_parameter('j4_gain').value
        self.j5_gain = self.get_parameter('j5_gain').value
        self.dead_zone_deg = self.get_parameter('dead_zone_deg').value
        
        # ========================================
        # 제어 모드 (1: 직접 제어, 2: 최적 제어)
        # ========================================
        self.control_mode = 1
        
        # 최적 제어 가중치 (Optimal Control Weights)
        # 비용 함수: J = Σ w_i * (q_target - q_current)² + Σ r_i * q_dot²
        self.w = [1.0, 0.0, 0.0, 0.8, 0.5, 0.0]  # 위치 오차 가중치 [J1~J6]
        self.r = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # 제어 입력 가중치 [J1~J6]
        
        # 조인트 속도 제한 (로봇 스펙, deg/s)
        self.joint_vel_limits = {
            'j1': 150.0, 'j2': 150.0, 'j3': 180.0,
            'j4': 225.0, 'j5': 225.0, 'j6': 225.0,
        }
        
        # 조인트 범위 제한 (deg)
        self.joint_limits = {
            'j1': (-360.0, 360.0), 'j2': (-95.0, 95.0), 'j3': (-160.0, 160.0),
            'j4': (-360.0, 360.0), 'j5': (-135.0, 135.0), 'j6': (-360.0, 360.0),
        }
        
        # 제어 주기 (최적 제어용)
        self.control_dt = 0.02  # 50Hz
        
        # ========================================
        # 상태 변수
        # ========================================
        self.state = "IDLE"  # IDLE, TRACKING, RETURN_HOME
        self.waiting_timeout = 2.0
        self.waiting_start_time = None
        
        # 조인트 상태
        self.current_joints = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.joints_received = False
        
        # 목표 (First Detection Lock)
        self.locked_target_joints = None
        self.locked_target_pos = None
        
        # 얼굴 위치
        self.face_pos = None
        self.face_time = None
        self.face_image_x = None
        self.image_center_x = 320.0
        
        # EKF 필터
        self.ekf = EKFFilter(dt=0.033, dim=3)
        
        # ========================================
        # ROS2 인터페이스
        # ========================================
        self.face_marker_sub = self.create_subscription(
            Marker, '/face_tracking/marker_robot', self.face_marker_callback, 10)
        self.face_image_sub = self.create_subscription(
            Float32MultiArray, '/face_detection/faces', self.face_image_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/dsr01/joint_states', self.joint_state_callback, 10)
        
        # 시각화 마커 발행 (원본 복원)
        self.aim_line_pub = self.create_publisher(
            Marker, '/joint_tracking/aim_line', 10)
        
        self._print_startup_info()
    
    def _print_startup_info(self):
        mode_str = "직접 제어" if self.control_mode == 1 else "최적 제어"
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎯 Joint-Space Face Tracking Node")
        self.get_logger().info(f"  Robot: {self.robot_id} / {self.robot_model}")
        self.get_logger().info(f"  Mode: {self.control_mode} ({mode_str})")
        self.get_logger().info(f"  J1: gain={self.j1_gain}, vel={self.j1_vel_limit}°/s")
        self.get_logger().info(f"  J4: gain={self.j4_gain}, vel={self.j4_vel_limit}°/s")
        self.get_logger().info("  키: 's'=시작, 'h'=홈, '1'=직접제어, '2'=최적제어, 'q'=종료")
        self.get_logger().info("=" * 60)
    
    # ========================================
    # 콜백
    # ========================================
    def face_marker_callback(self, msg):
        """얼굴 3D 위치 수신 + EKF 필터링"""
        raw_pos = np.array([
            msg.pose.position.x * 1000.0,
            msg.pose.position.y * 1000.0,
            msg.pose.position.z * 1000.0
        ])
        
        if np.linalg.norm(raw_pos) < 10.0:
            return
        
        # EKF 필터링
        if not self.ekf.initialized:
            self.ekf.initialize(raw_pos.tolist())
            filtered_pos = raw_pos
        else:
            self.ekf.predict()
            self.ekf.update(raw_pos.tolist())
            filtered_pos = np.array(self.ekf.get_position())
        
        # 디버그 로그 (원본 복원)
        self.get_logger().info(
            f"📡 마커수신 EKF={'ON' if self.ekf else 'OFF'} | "
            f"Raw:({raw_pos[0]:.0f},{raw_pos[1]:.0f}) → Filtered:({filtered_pos[0]:.0f},{filtered_pos[1]:.0f})mm",
            throttle_duration_sec=2.0)
        
        self.face_pos = filtered_pos
        self.face_time = time.time()
    
    def face_image_callback(self, msg):
        """얼굴 이미지 좌표 (J5 미세 조정)"""
        if len(msg.data) >= 2:
            self.face_image_x = msg.data[0]
    
    def joint_state_callback(self, msg):
        """현재 조인트 상태"""
        if len(msg.position) >= 6:
            new_joints = [math.degrees(p) for p in msg.position[:6]]
            
            # 초기화 전 상태 무시
            if abs(new_joints[2]) < 5.0 and abs(new_joints[3]) < 5.0:
                return
            
            # 급격한 변화 필터링
            if self.joints_received:
                if abs(new_joints[3] - self.current_joints[3]) > 50.0:
                    return
            
            self.current_joints = new_joints
            self.joints_received = True
    
    # ========================================
    # 목표 조인트 계산
    # ========================================
    def compute_desired_joints(self):
        """
        얼굴 위치 → 목표 조인트 각도
        
        J1: atan2(y, x) - 수평 방향
        J4: 피치 보정 - 수직 방향
        J5: 이미지 기반 미세 조정
        """
        if self.face_pos is None:
            return None
        
        fx, fy, fz = self.face_pos
        
        if abs(fx) < 1 and abs(fy) < 1:
            return None
        
        # 시작 자세 기준
        q = list(START_JOINTS)
        
        # J1: 목표 방향 (수평)
        distance_xy = math.sqrt(fx**2 + fy**2)
        if distance_xy > 100:
            q[0] = math.degrees(math.atan2(fy, fx))
        
        # J4: 피치 보정 (수직)
        tcp_z_approx = 550.0
        if distance_xy > 100:
            pitch = math.degrees(math.atan2(fz - tcp_z_approx, distance_xy))
            q[3] = START_JOINTS[3] - pitch
        
        # J5: 이미지 기반 미세 조정
        if self.face_image_x is not None:
            error_pixel = self.face_image_x - self.image_center_x
            q[4] = START_JOINTS[4] - error_pixel * 0.03
        
        return q
    
    # ========================================
    # 최적 제어 (Mode 2)
    # ========================================
    def calculate_optimal_control(self, q_desired, dt):
        """
        최적 제어 입력 계산
        
        비용 함수:
            J = Σ w_i * (q_desired_i - q_current_i)² + Σ r_i * q_dot_i²
        
        해석적 해 (Closed-form solution):
            ∂J/∂q_dot = 2*r*q_dot - 2*w*(q_desired - q_current)/dt = 0
            q_dot_optimal = w/(w+r) * (q_desired - q_current) / dt
        
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
            error = q_desired[i] - q_current[i]
            
            # 가중치가 0이면 현재값 유지
            if self.w[i] == 0:
                q_next[i] = q_current[i]
                continue
            
            # 최적 속도 계산
            optimal_gain = self.w[i] / (self.w[i] + self.r[i])
            q_dot_optimal = optimal_gain * error / dt
            
            # 속도 제한 적용 (로봇 스펙)
            vel_limit = self.joint_vel_limits[limits_keys[i]]
            q_dot_clamped = max(-vel_limit, min(vel_limit, q_dot_optimal))
            
            # 다음 조인트 각도
            q_next_raw = q_current[i] + q_dot_clamped * dt
            
            # 범위 제한 적용
            q_min, q_max = self.joint_limits[limits_keys[i]]
            q_next[i] = max(q_min, min(q_max, q_next_raw))
        
        return q_next
    
    def calculate_target_joints(self, dt=0.02):
        """
        모드에 따른 목표 조인트 계산
        
        Mode 1: 직접 제어 (compute_desired_joints)
        Mode 2: 최적 제어 (calculate_optimal_control)
        """
        q_desired = self.compute_desired_joints()
        if q_desired is None:
            return None
        
        if self.control_mode == 1:
            return q_desired
        else:
            return self.calculate_optimal_control(q_desired, dt)
    
    def check_arrival(self, target):
        """도착 여부 확인"""
        for i in [0, 3, 4]:  # J1, J4, J5
            if abs(self.current_joints[i] - target[i]) > ARRIVAL_THRESHOLD_DEG:
                return False
        return True


def main():
    """메인 함수 - DSR SDK 패턴 유지"""
    rclpy.init()
    
    node = JointTrackingNode()
    
    # DR_init 설정
    DR_init.__dsr__id = node.robot_id
    DR_init.__dsr__model = node.robot_model
    
    # DSR 노드
    dsr_node = rclpy.create_node("dsr_joint_control", namespace=node.robot_id)
    DR_init.__dsr__node = dsr_node
    
    # DSR 함수 import
    try:
        from DSR_ROBOT2 import movej, amovej, mwait
        print("✅ DSR 모듈 import 성공")
    except ImportError as e:
        print(f"❌ DSR 모듈 import 실패: {e}")
        sys.exit(1)
    
    # Executor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(dsr_node)
    
    print("\n" + "="*60)
    print("🎯 Joint-Space Face Tracking")
    print(f"  현재 모드: {node.control_mode} ({'직접 제어' if node.control_mode == 1 else '최적 제어'})")
    print("  's': 추적 시작/중지")
    print("  'h': 홈 위치")
    print("  '1': 모드 1 (직접 제어)")
    print("  '2': 모드 2 (최적 제어)")
    print("  'q': 종료")
    print("="*60)
    
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.001)
            
            # 키보드 입력
            import select
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.readline().strip().lower()
                
                if key == 'q':
                    print("\n종료합니다...")
                    break
                elif key == 'h':
                    print("\n🏠 홈 위치로 이동...")
                    node.state = "IDLE"
                    movej(HOME_JOINTS, vel=30, acc=30)
                    mwait()
                    print("✅ 홈 도착")
                elif key == '1':
                    node.control_mode = 1
                    print("\n🔧 모드 1: 직접 제어 (목표 → amovej)")
                elif key == '2':
                    node.control_mode = 2
                    print("\n🔧 모드 2: 최적 제어 (비용함수 최소화)")
                    print(f"   w={node.w}")
                    print(f"   r={node.r}")
                elif key == 's':
                    if node.state == "IDLE":
                        print("\n📍 시작 위치로 이동...")
                        movej(START_JOINTS, vel=30, acc=30)
                        mwait()
                        
                        # 조인트 동기화
                        node.joints_received = False
                        sync_start = time.time()
                        while time.time() - sync_start < 2.0:
                            executor.spin_once(timeout_sec=0.05)
                            if node.joints_received:
                                break
                        
                        mode_str = "직접 제어" if node.control_mode == 1 else "최적 제어"
                        print(f"🎯 추적 시작! (모드 {node.control_mode}: {mode_str})")
                        node.state = "TRACKING"
                        node.waiting_start_time = time.time()
                    else:
                        print("\n⏸️ 추적 중지")
                        node.state = "IDLE"
                        node.locked_target_joints = None
            
            # ========================================
            # 상태 머신
            # ========================================
            current_time = time.time()
            
            if not node.joints_received:
                continue
            
            # 얼굴 감지 확인
            face_detected = False
            if node.face_time and (current_time - node.face_time) < 0.5:
                face_detected = True
            
            # TRACKING
            if node.state == "TRACKING":
                if node.locked_target_joints is None:
                    # 첫 감지 대기
                    if face_detected:
                        # 모드에 따라 다른 계산 방식
                        target = node.calculate_target_joints(dt=node.control_dt)
                        if target:
                            node.locked_target_pos = node.face_pos.copy()
                            node.locked_target_joints = target
                            
                            amovej(target, vel=30.0, acc=50.0)
                            mode_str = "직접" if node.control_mode == 1 else "최적"
                            node.get_logger().info(
                                f"🔒 [{mode_str}] 목표 고정! J1:{target[0]:+.1f}° J4:{target[3]:+.1f}°")
                            node.waiting_start_time = None
                    else:
                        if node.waiting_start_time is None:
                            node.waiting_start_time = current_time
                        if current_time - node.waiting_start_time > node.waiting_timeout:
                            node.get_logger().info("⏰ 타임아웃 → RETURN_HOME")
                            node.state = "RETURN_HOME"
                else:
                    # 도착 확인
                    if node.check_arrival(node.locked_target_joints):
                        node.get_logger().info("✅ 도착!")
                        node.locked_target_joints = None
                        node.locked_target_pos = None
                        node.waiting_start_time = current_time
                    else:
                        node.get_logger().info(
                            f"🚀 이동중 J1:{node.current_joints[0]:+.1f}→{node.locked_target_joints[0]:+.1f}°",
                            throttle_duration_sec=0.5)
            
            # RETURN_HOME
            elif node.state == "RETURN_HOME":
                node.get_logger().info("🏠 시작 위치로 복귀...")
                movej(START_JOINTS, vel=30, acc=30)
                mwait()
                node.state = "TRACKING"
                node.locked_target_joints = None
                node.waiting_start_time = None
            
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
