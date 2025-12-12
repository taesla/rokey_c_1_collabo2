#!/usr/bin/env python3
"""
Robot Control Node - Cartesian Space Velocity Control for Head Tracking

제어 방식: Velocity-based Cartesian Space Control
- 비례 제어로 속도 벡터 생성
- 안전 영역 검증
- Dead zone 및 속도 제한 적용

Subscribed Topics:
  /face_tracking/marker_ekf_filtered - EKF 필터링된 목표 위치 (Blue Cube)

제어 파이프라인:
  MediaPipe(Raw) → Camera EKF → Robot EKF → Cartesian Controller → Robot
"""
import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from visualization_msgs.msg import Marker
import DR_init
from face_tracking_pkg.face_tracking_ekf import FaceTrackingEKF


class RobotControlNode(Node):
    """로봇 제어 노드 - Cartesian Space Velocity Control"""
    
    def __init__(self):
        super().__init__('robot_control_node')
        
        # 파라미터 선언
        self.declare_parameter('robot_id', 'dsr01')
        self.declare_parameter('robot_model', 'm0609')
        self.declare_parameter('velocity', 200.0)  # mm/s
        self.declare_parameter('acceleration', 400.0)  # mm/s²
        self.declare_parameter('k_p', 0.4)  # 비례 게인
        self.declare_parameter('dead_zone', 10.0)  # mm
        self.declare_parameter('use_ekf', True)
        self.declare_parameter('ekf_process_noise', 0.1)
        self.declare_parameter('ekf_measurement_noise', 10.0)
        
        # 파라미터 로드
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_model = self.get_parameter('robot_model').value
        self.v_max = self.get_parameter('velocity').value
        self.a_max = self.get_parameter('acceleration').value
        self.k_p = self.get_parameter('k_p').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.use_ekf = self.get_parameter('use_ekf').value
        self.ekf_process_noise = self.get_parameter('ekf_process_noise').value
        self.ekf_measurement_noise = self.get_parameter('ekf_measurement_noise').value
        
        # 시작/홈 위치 [J1, J2, J3, J4, J5, J6]
        self.start_joints = [3.06, 2.84, 92.13, 86.07, -1.43, 8.33]
        self.home_joints = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        # 안전 범위 (mm) - 보수적으로 설정
        self.safe_r_min = 350.0  # 최소 반경 (너무 가까우면 충돌)
        self.safe_r_max = 1100.0  # 최대 반경 (도달 범위)
        self.safe_z_min = 150.0  # 최소 높이 (테이블 충돌 방지)
        self.safety_margin = 50.0  # 안전 여유 (mm)
        
        # 상태
        self.state = "IDLE"  # IDLE, TRACKING
        self.target_pos = None  # EKF 필터링된 목표 위치
        self.last_move_time = time.time()
        self.control_period = 0.033  # 30Hz
        
        # EKF 초기화 (30Hz)
        self.ekf = None
        if self.use_ekf:
            self.ekf = FaceTrackingEKF(dt=0.033)
            # 노이즈 파라미터 설정
            self.ekf.kf.Q = np.eye(9) * self.ekf_process_noise
            self.ekf.kf.Q[6:9, 6:9] *= 2.0  # 가속도 노이즈 더 크게
            self.ekf.kf.R = np.eye(3) * self.ekf_measurement_noise
        
        # 마커 구독 & 발행
        self.marker_sub = self.create_subscription(
            Marker, '/face_tracking/marker_ekf_filtered', self.marker_callback, 10)
        self.ekf_marker_pub = self.create_publisher(
            Marker, '/face_tracking/marker_ekf_filtered', 10)
        self.ekf_text_pub = self.create_publisher(
            Marker, '/face_tracking/text_ekf_filtered', 10)
        
        # TCP 위치 시각화 마커 퍼블리셔 (보라색 큐브)
        self.tcp_marker_pub = self.create_publisher(
            Marker, '/robot_control/tcp_marker', 10)
        self.tcp_text_pub = self.create_publisher(
            Marker, '/robot_control/tcp_text', 10)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🤖 Robot Control Node - Cartesian Velocity Control")
        self.get_logger().info(f"  Robot: {self.robot_id} / {self.robot_model}")
        self.get_logger().info(f"  Control: K_p={self.k_p}, v_max={self.v_max}mm/s")
        self.get_logger().info(f"  Safety: Dead zone={self.dead_zone}mm, Margin={self.safety_margin}mm")
        if self.use_ekf:
            self.get_logger().info(f"  🔬 EKF: ON (Q={self.ekf_process_noise}, R={self.ekf_measurement_noise})")
        else:
            self.get_logger().info("  ⚠️  EKF: OFF")
        self.get_logger().info("  키: 's'=시작 추적, 'h'=홈, 'q'=종료")
        self.get_logger().info("=" * 60)
    
    def marker_callback(self, msg):
        """Blue 큐브 마커 수신 - EKF 필터링된 목표 위치"""
        # 위치 (mm)
        target_x = msg.pose.position.x * 1000.0
        target_y = msg.pose.position.y * 1000.0
        target_z = msg.pose.position.z * 1000.0
        
        self.target_pos = [target_x, target_y, target_z]
        
        # TRACKING 모드로 자동 전환
        if self.state == "IDLE":
            self.get_logger().info("🎯 얼굴 감지! → 추적 모드")
            self.state = "TRACKING"
    
    def track_face(self, target_pos, current_tcp):
        """
        Cartesian Space Velocity Control
        
        Args:
            target_pos: 목표 위치 [x, y, z] in mm
            current_tcp: 현재 TCP 위치 [x, y, z, rx, ry, rz]
        
        Returns:
            velocity: 속도 벡터 [vx, vy, vz] in mm/s, None if not safe
        """
        # 1. 안전 영역 체크
        if not self.is_safe_position(target_pos):
            self.get_logger().warn(f"⚠️ 안전 영역 밖: {target_pos}")
            return None
        
        # 2. 현재 위치 추출 (x, y, z만)
        current_pos = np.array(current_tcp[:3])
        target_array = np.array(target_pos)
        
        # 3. 오차 계산
        error = target_array - current_pos
        distance = np.linalg.norm(error)
        
        # 4. Dead zone (너무 가까우면 무시)
        if distance < self.dead_zone:
            return None
        
        # 5. 속도 벡터 계산 (비례 제어)
        velocity = error * self.k_p
        
        # 6. 속도 크기 제한
        velocity_norm = np.linalg.norm(velocity)
        if velocity_norm > self.v_max:
            velocity = velocity * (self.v_max / velocity_norm)
        
        return velocity.tolist()
    
    def publish_ekf_marker(self, filtered_pos):
        """EKF 필터링된 마커 퍼블리시 (파란색 큐브)"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_ekf_filtered"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = filtered_pos[0] / 1000.0
        marker.pose.position.y = filtered_pos[1] / 1000.0
        marker.pose.position.z = filtered_pos[2] / 1000.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.5, 1.0, 0.5
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200000000  # 0.2초
        self.ekf_marker_pub.publish(marker)
        
        # 텍스트 마커 (별도 토픽)
        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "face_ekf_text"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = filtered_pos[0] / 1000.0
        text.pose.position.y = filtered_pos[1] / 1000.0
        text.pose.position.z = filtered_pos[2] / 1000.0 + 0.12
        text.pose.orientation.w = 1.0
        text.scale.z = 0.05
        text.color.r, text.color.g, text.color.b, text.color.a = 0.0, 0.5, 1.0, 1.0
        text.text = "Filtered"
        text.lifetime.sec = 0
        text.lifetime.nanosec = 200000000  # 0.2초
        self.ekf_text_pub.publish(text)
    
    def publish_tcp_marker(self, tcp_pos):
        """
        현재 로봇 TCP 위치 시각화 (보라색 큐브)
        
        Args:
            tcp_pos: [x, y, z, rx, ry, rz] in mm and degrees
        """
        # 큐브 마커
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_tcp"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = tcp_pos[0] / 1000.0
        marker.pose.position.y = tcp_pos[1] / 1000.0
        marker.pose.position.z = tcp_pos[2] / 1000.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.8, 0.0, 0.8, 0.7  # 보라색
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200000000  # 0.2초
        self.tcp_marker_pub.publish(marker)
        
        # 텍스트 마커
        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "robot_tcp_text"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = tcp_pos[0] / 1000.0
        text.pose.position.y = tcp_pos[1] / 1000.0
        text.pose.position.z = tcp_pos[2] / 1000.0 + 0.10
        text.pose.orientation.w = 1.0
        text.scale.z = 0.05
        text.color.r, text.color.g, text.color.b, text.color.a = 0.8, 0.0, 0.8, 1.0
        text.text = "TCP"
        text.lifetime.sec = 0
        text.lifetime.nanosec = 200000000  # 0.2초
        self.tcp_text_pub.publish(text)
    
    def is_safe_position(self, pos):
        """
        위치 안전 확인 (보수적 설정)
        
        Args:
            pos: [x, y, z] in mm
        
        Returns:
            bool: True if safe
        """
        x, y, z = pos[0], pos[1], pos[2]
        
        # 원점 거리 (반경)
        r = math.sqrt(x*x + y*y + z*z)
        
        # 안전 반경 체크 (여유 포함)
        if not (self.safe_r_min + self.safety_margin <= r <= self.safe_r_max - self.safety_margin):
            return False
        
        # 높이 체크 (테이블 충돌 방지)
        if z < self.safe_z_min + self.safety_margin:
            return False
        
        # 전방 범위 체크 (로봇 앞쪽만)
        if x < 200:  # 로봇 뒤쪽 제외
            return False
        
        return True
    
    # ========================================================================
    # 🚧 LEGACY: J1+J5 Only Control (Phase 5-1)
    # 주석 처리됨 - 새로운 MPC 컨트롤러(robot_control_mpc_node.py) 사용 권장
    # ========================================================================
    
    def calculate_joint_deltas(self, current_tcp, target_pos):
        """J1, J5 변화량 계산 (LEGACY - 사용 안 함)"""
        dy = target_pos[1] - current_tcp[1]
        dz = target_pos[2] - current_tcp[2]
        
        dist = math.sqrt(target_pos[0]**2 + target_pos[1]**2)
        dist_factor = 500.0 / max(dist, 300.0)
        
        delta_j1 = dy * self.j1_gain * dist_factor
        delta_j5 = -dz * self.j5_gain * dist_factor
        
        delta_j1 = max(-10.0, min(10.0, delta_j1))
        delta_j5 = max(-8.0, min(8.0, delta_j5))
        
        return delta_j1, delta_j5


def main(args=None):
    rclpy.init(args=args)
    
    # RobotControlNode 먼저 생성
    node = RobotControlNode()
    
    # DSR 초기화
    DR_init.__dsr__id = node.robot_id
    DR_init.__dsr__model = node.robot_model
    
    # DSR 노드 생성
    dsr_node = rclpy.create_node("robot_control_dsr", namespace=node.robot_id)
    DR_init.__dsr__node = dsr_node
    
    # DSR 함수 import
    try:
        from DSR_ROBOT2 import movej, movel, get_current_posx, mwait
        print("✅ DSR 모듈 import 성공")
    except ImportError as e:
        print(f"❌ DSR 모듈 import 실패: {e}")
        sys.exit(1)
    
    print("\n>>> 키 입력:")
    print("  's' = 시작 위치로 이동 후 추적 시작")
    print("  'h' = 홈 위치로 이동")
    print("  'q' = 종료\n")
    
    last_loop_time = time.time()
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(dsr_node)
    
    try:
        while rclpy.ok():
            for _ in range(10):
                executor.spin_once(timeout_sec=0.01)
            
            # 키보드 입력
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.readline().strip().lower()
                
                if key == 's':
                    node.get_logger().info("📍 시작 위치로 이동 중...")
                    movej(node.start_joints, vel=60, acc=60)
                    mwait()
                    current_tcp = list(get_current_posx()[0])
                    node.get_logger().info(f"✅ 추적 준비 완료! TCP: ({current_tcp[0]:.0f}, {current_tcp[1]:.0f}, {current_tcp[2]:.0f})mm")
                    node.state = "IDLE"  # 얼굴 감지 시 자동 TRACKING 전환
                
                elif key == 'h':
                    node.get_logger().info("🏠 홈 위치로 이동...")
                    node.state = "IDLE"
                    movej(node.home_joints, vel=60, acc=60)
                    mwait()
                    node.get_logger().info("✅ 홈 도착!")
                
                elif key == 'q':
                    print("종료합니다...")
                    break
            
            # 제어 루프 (30Hz)
            now = time.time()
            if now - node.last_move_time < node.control_period:
                continue
            
            node.last_move_time = now
            
            # TRACKING 모드: Cartesian 제어
            if node.state == "TRACKING" and node.target_pos is not None:
                # 현재 TCP 위치
                current_tcp = list(get_current_posx()[0])
                
                # TCP 위치 시각화 (보라색 큐브)
                node.publish_tcp_marker(current_tcp)
                
                # Velocity 계산
                velocity = node.track_face(node.target_pos, current_tcp)
                
                if velocity is not None:
                    # 목표 위치 계산 (현재 + 속도 * dt)
                    target_tcp = current_tcp.copy()
                    target_tcp[0] += velocity[0] * node.control_period
                    target_tcp[1] += velocity[1] * node.control_period
                    target_tcp[2] += velocity[2] * node.control_period
                    
                    # 속도 노름 계산
                    v_norm = np.linalg.norm(velocity)
                    
                    # 로봇 이동 (movel - 직선 경로)
                    movel(target_tcp, vel=v_norm, acc=node.a_max)
                    
                    # 오차 계산 (로그용)
                    error = np.array(node.target_pos) - np.array(current_tcp[:3])
                    error_norm = np.linalg.norm(error)
                    
                    node.get_logger().info(
                        f"🎯 Error: {error_norm:.1f}mm | "
                        f"Vel: {v_norm:.1f}mm/s | "
                        f"TCP: [{current_tcp[0]:.0f}, {current_tcp[1]:.0f}, {current_tcp[2]:.0f}]"
                    )
    
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
