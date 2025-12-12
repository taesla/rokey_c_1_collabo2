#!/usr/bin/env python3
"""
Robot Control Node - J1(Base) + J5(Wrist2) 관절 제어로 얼굴 추적

사주경계 모드: J1을 ±60도 범위에서 스캔
추적 모드: 얼굴 위치 기반으로 J1(좌우) + J5(상하) 조절

Subscribed Topics:
  /face_tracking/marker_robot - 로봇 좌표계 얼굴 위치 (from face_tracking_node)

조인트 명명 규칙 (Doosan API):
- J1: Base (좌우) - index [0]
- J5: Wrist2 (상하) - index [4]

제어 파이프라인:
  MediaPipe(Raw) → EKF(필터링) → P-Controller → Robot
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
    """로봇 제어 노드 - J1(좌우) + J5(상하)"""
    
    def __init__(self):
        super().__init__('robot_control_node')
        
        # 파라미터 선언
        self.declare_parameter('robot_id', 'dsr01')
        self.declare_parameter('robot_model', 'm0609')
        self.declare_parameter('velocity', 45)
        self.declare_parameter('acceleration', 45)
        self.declare_parameter('j1_gain', 0.12)
        self.declare_parameter('j5_gain', 0.08)
        self.declare_parameter('patrol_step', 10.0)
        self.declare_parameter('detection_timeout', 2.0)
        self.declare_parameter('max_fail_count', 3)
        self.declare_parameter('use_ekf', True)
        self.declare_parameter('ekf_process_noise', 0.1)
        self.declare_parameter('ekf_measurement_noise', 10.0)
        
        # 파라미터 로드
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_model = self.get_parameter('robot_model').value
        self.velocity = self.get_parameter('velocity').value
        self.acceleration = self.get_parameter('acceleration').value
        self.j1_gain = self.get_parameter('j1_gain').value
        self.j5_gain = self.get_parameter('j5_gain').value
        self.patrol_step = self.get_parameter('patrol_step').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.max_fail_count = self.get_parameter('max_fail_count').value
        self.use_ekf = self.get_parameter('use_ekf').value
        self.ekf_process_noise = self.get_parameter('ekf_process_noise').value
        self.ekf_measurement_noise = self.get_parameter('ekf_measurement_noise').value
        
        # 시작/홈 위치 [J1, J2, J3, J4, J5, J6]
        self.start_joints = [3.06, 2.84, 92.13, 86.07, -1.43, 8.33]
        self.home_joints = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        # J1, J5 범위
        self.j1_min = self.start_joints[0] - 80.0
        self.j1_max = self.start_joints[0] + 0.0
        self.j5_min = self.start_joints[4] - 200.0
        self.j5_max = self.start_joints[4] + 200.0
        
        # 안전 범위 (mm)
        self.safe_r_min = 300.0
        self.safe_r_max = 1200.0
        self.safe_z_min = 100.0
        
        # 상태
        self.state = "IDLE"
        self.target_pos = None
        self.raw_pos = None  # EKF 비교용 Raw 위치
        self.last_detection_time = time.time()
        self.detection_fail_count = 0
        self.reference_tcp = None
        self.patrol_j1_current = self.start_joints[0]
        self.patrol_direction = 1
        
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
            Marker, '/face_tracking/marker_robot', self.marker_callback, 10)
        self.ekf_marker_pub = self.create_publisher(
            Marker, '/face_tracking/marker_ekf_filtered', 10)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("🤖 Robot Control Node (J1+J5)")
        self.get_logger().info(f"  Robot: {self.robot_id} / {self.robot_model}")
        self.get_logger().info(f"  Velocity: {self.velocity}")
        if self.use_ekf:
            self.get_logger().info(f"  🔬 EKF: ON (Q={self.ekf_process_noise}, R={self.ekf_measurement_noise})")
        else:
            self.get_logger().info("  ⚠️  EKF: OFF (Raw 신호 사용)")
        self.get_logger().info("  키: 's'=시작, 'p'=사주경계 재개, 'h'=홈, 'q'=종료")
        self.get_logger().info("=" * 50)
    
    def marker_callback(self, msg):
        """마커 수신 콜백 - EKF 필터링 적용"""
        # Raw 위치 (mm)
        raw_x = msg.pose.position.x * 1000.0
        raw_y = msg.pose.position.y * 1000.0
        raw_z = msg.pose.position.z * 1000.0
        
        # 기본 필터링 (이상치 제거)
        if not (200 < raw_x < 1000 and -400 < raw_y < 600 and 200 < raw_z < 800):
            return
        
        # 급격한 변화 제거
        if self.target_pos is not None:
            dx = abs(raw_x - self.target_pos[0])
            dy = abs(raw_y - self.target_pos[1])
            dz = abs(raw_z - self.target_pos[2])
            if dx > 200 or dy > 200 or dz > 200:
                return
        
        raw_pos = [raw_x, raw_y, raw_z]
        self.raw_pos = raw_pos  # 비교용 저장
        
        # EKF 필터링
        if self.use_ekf and self.ekf is not None:
            if not self.ekf.initialized:
                # 첫 측정값으로 초기화
                self.ekf.initialize(raw_pos)
                self.target_pos = raw_pos
                self.get_logger().info(f"🔬 EKF 초기화: [{raw_x:.1f}, {raw_y:.1f}, {raw_z:.1f}]")
            else:
                # 예측 단계
                self.ekf.predict()
                
                # 업데이트 단계
                self.ekf.update(raw_pos)
                
                # 필터링된 위치 사용
                filtered_pos = self.ekf.get_position()
                filtered_vel = self.ekf.get_velocity()
                
                self.target_pos = filtered_pos.tolist()
                
                # 주기적으로 Raw vs Filtered 비교 출력 (5초마다)
                if not hasattr(self, '_last_log_time'):
                    self._last_log_time = time.time()
                
                if time.time() - self._last_log_time > 5.0:
                    raw_array = np.array(raw_pos)
                    filtered_array = np.array(filtered_pos)
                    noise = np.linalg.norm(raw_array - filtered_array)
                    vel_norm = np.linalg.norm(filtered_vel)
                    
                    self.get_logger().info(
                        f"📊 Raw: [{raw_x:.1f}, {raw_y:.1f}, {raw_z:.1f}] | "
                        f"Filtered: [{filtered_pos[0]:.1f}, {filtered_pos[1]:.1f}, {filtered_pos[2]:.1f}] | "
                        f"Noise: {noise:.1f}mm | Vel: {vel_norm:.1f}mm/s"
                    )
                    self._last_log_time = time.time()
        else:
            # EKF 미사용 시 Raw 그대로 사용
            self.target_pos = raw_pos
        
        # EKF 필터링된 마커 퍼블리시 (파란색) - Raw 마커 있을 때만
        if self.use_ekf and self.ekf is not None and self.ekf.initialized:
            filtered_pos = self.ekf.get_position()
            self.publish_ekf_marker(filtered_pos)
        
        self.last_detection_time = time.time()
        self.detection_fail_count = 0
        
        if self.state == "PATROL":
            self.get_logger().info("🎯 얼굴 감지! → 추적 모드")
            self.state = "TRACKING"
    
    def publish_ekf_marker(self, filtered_pos):
        """EKF 필터링된 마커 퍼블리시 (파란색)"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_ekf_filtered"
        marker.id = 10
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = filtered_pos[0] / 1000.0
        marker.pose.position.y = filtered_pos[1] / 1000.0
        marker.pose.position.z = filtered_pos[2] / 1000.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.12
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.5, 1.0, 0.8
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        self.ekf_marker_pub.publish(marker)
        
        # 짧은 딜레이
        import time
        time.sleep(0.001)
        
        # 텍스트 마커
        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "face_ekf_text"
        text.id = 110
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
        text.lifetime.nanosec = 0
        self.ekf_marker_pub.publish(text)
    
    def is_safe_position(self, pos):
        """위치 안전 확인"""
        x, y, z = pos[0], pos[1], pos[2]
        r = math.sqrt(x*x + y*y + z*z)
        return self.safe_r_min <= r <= self.safe_r_max and z >= self.safe_z_min
    
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
        from DSR_ROBOT2 import movej, get_current_posx, get_current_posj, mwait
        print("✅ DSR 모듈 import 성공")
    except ImportError as e:
        print(f"❌ DSR 모듈 import 실패: {e}")
        sys.exit(1)
    
    print("\n>>> 's' 입력 후 Enter: ")
    
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
                    node.state = "IDLE"
                    movej(node.start_joints, vel=node.velocity, acc=node.acceleration)
                    mwait()
                    node.reference_tcp = list(get_current_posx()[0])
                    node.get_logger().info(f"✅ 시작 완료! TCP: ({node.reference_tcp[0]:.0f}, {node.reference_tcp[1]:.0f}, {node.reference_tcp[2]:.0f})mm")
                    node.state = "PATROL"
                    node.patrol_j1_current = node.start_joints[0]
                    node.patrol_direction = 1
                    node.last_detection_time = time.time()
                
                elif key == 'p':
                    if node.state == "TRACKING":
                        node.get_logger().info("🔄 사주경계 모드로 전환...")
                        node.state = "IDLE"
                        movej(node.start_joints, vel=node.velocity, acc=node.acceleration)
                        mwait()
                        node.state = "PATROL"
                        node.patrol_j1_current = node.start_joints[0]
                        node.patrol_direction = 1
                        node.get_logger().info("✅ 사주경계 재개!")
                    else:
                        node.get_logger().info("⚠️ 추적 모드가 아닙니다.")
                
                elif key == 'h':
                    node.get_logger().info("🏠 홈 위치로 이동...")
                    node.state = "IDLE"
                    movej(node.home_joints, vel=node.velocity, acc=node.acceleration)
                    mwait()
                    node.get_logger().info("✅ 홈 도착!")
                
                elif key == 'q':
                    print("종료합니다...")
                    break
            
            now = time.time()
            
            if node.state == "PATROL":
                if now - last_loop_time < 0.8:
                    continue
            elif node.state == "TRACKING":
                if now - last_loop_time < 0.15:
                    continue
            else:
                continue
            
            last_loop_time = now
            
            # PATROL 모드
            if node.state == "PATROL":
                node.patrol_j1_current += node.patrol_step * node.patrol_direction
                
                if node.patrol_j1_current >= node.j1_max:
                    node.patrol_j1_current = node.j1_max
                    node.patrol_direction = -1
                elif node.patrol_j1_current <= node.j1_min:
                    node.patrol_j1_current = node.j1_min
                    node.patrol_direction = 1
                
                target_joints = list(node.start_joints)
                target_joints[0] = node.patrol_j1_current
                
                node.get_logger().info(f"🔍 사주경계: J1={node.patrol_j1_current:.1f}°")
                movej(target_joints, vel=node.velocity, acc=node.acceleration)
            
            # TRACKING 모드
            elif node.state == "TRACKING":
                # 디텍션 실패 시 경고만 출력하고 추적 모드 유지
                if time.time() - node.last_detection_time > node.detection_timeout:
                    if node.detection_fail_count == 0:
                        node.get_logger().warn("⚠️ 얼굴 미감지 - 마지막 위치에서 대기 중... ('p'=사주경계 재개)")
                    node.detection_fail_count += 1
                    node.last_detection_time = time.time()
                    continue
                
                if node.target_pos is not None:
                    if not node.is_safe_position(node.target_pos):
                        continue
                    
                    current_joints = list(get_current_posj()[0])
                    current_pose = list(get_current_posx()[0])
                    
                    delta_j1, delta_j5 = node.calculate_joint_deltas(current_pose, node.target_pos)
                    
                    new_j1 = max(node.j1_min, min(node.j1_max, current_joints[0] + delta_j1))
                    new_j5 = max(node.j5_min, min(node.j5_max, current_joints[4] + delta_j5))
                    
                    target_joints = list(current_joints)
                    target_joints[0] = new_j1
                    target_joints[4] = new_j5
                    
                    node.get_logger().info(f"🎯 J1: {current_joints[0]:.1f}→{new_j1:.1f}° | J5: {current_joints[4]:.1f}→{new_j5:.1f}°")
                    movej(target_joints, vel=node.velocity, acc=node.acceleration)
    
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
