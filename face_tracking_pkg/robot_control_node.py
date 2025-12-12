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
from dsr_msgs2.msg import ServolRtStream
import DR_init
from face_tracking_pkg.face_tracking_ekf import FaceTrackingEKF


class RobotControlNode(Node):
    """로봇 제어 노드 - Cartesian Space Velocity Control"""
    
    def __init__(self):
        super().__init__('robot_control_node')
        
        # 파라미터 선언
        self.declare_parameter('robot_id', 'dsr01')
        self.declare_parameter('robot_model', 'm0609')
        self.declare_parameter('velocity', 60.0)  # mm/s (적당히 빠르게: 40→60)
        self.declare_parameter('acceleration', 120.0)  # mm/s² (부드럽게: 100→120)
        self.declare_parameter('k_p', 0.2)  # 비례 게인 (더 낮춤: 0.25→0.2)
        self.declare_parameter('dead_zone', 50.0)  # mm (진동 방지: 30→50)
        self.declare_parameter('tcp_offset_z', 228.6)  # mm (RG2 그리퍼 TCP offset)
        self.declare_parameter('use_servol_rt', False)  # 실시간 제어 모드 (1kHz)
        self.declare_parameter('use_ekf', False)  # face_tracking_node에서 이미 필터링됨
        self.declare_parameter('ekf_process_noise', 0.1)
        self.declare_parameter('ekf_measurement_noise', 10.0)
        
        # 파라미터 로드
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_model = self.get_parameter('robot_model').value
        self.v_max = self.get_parameter('velocity').value
        self.a_max = self.get_parameter('acceleration').value
        self.k_p = self.get_parameter('k_p').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.tcp_offset_z = self.get_parameter('tcp_offset_z').value
        self.use_servol_rt = self.get_parameter('use_servol_rt').value
        self.use_ekf = self.get_parameter('use_ekf').value
        self.ekf_process_noise = self.get_parameter('ekf_process_noise').value
        self.ekf_measurement_noise = self.get_parameter('ekf_measurement_noise').value
        
        # 시작/홈 위치 [J1, J2, J3, J4, J5, J6]
        self.start_joints = [3.06, 2.84, 92.13, 86.07, -1.43, 8.33]
        self.home_joints = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        # 안전 범위 (mm) - 완화된 설정
        self.safe_r_min = 300.0  # 최소 반경 (350→300)
        self.safe_r_max = 1100.0  # 최대 반경 (도달 범위)
        self.safe_z_min = 100.0  # 최소 높이 (150→100, 테이블 충돌 방지)
        self.safety_margin = 30.0  # 안전 여유 (50→30mm)
        
        # 상태
        self.state = "IDLE"  # IDLE, TRACKING
        self.target_pos = None  # EKF 필터링된 목표 위치
        self.last_move_time = time.time()
        self.control_period = 0.1  # 10Hz (현실화: amovel 한계 고려)
        
        # EKF 초기화 (10Hz)
        self.ekf = None
        if self.use_ekf:
            self.ekf = FaceTrackingEKF(dt=0.1)
            # 노이즈 파라미터 설정
            self.ekf.kf.Q = np.eye(9) * self.ekf_process_noise
            self.ekf.kf.Q[6:9, 6:9] *= 2.0  # 가속도 노이즈 더 크게
            self.ekf.kf.R = np.eye(3) * self.ekf_measurement_noise
        
        # 마커 구독 & 발행 (Red 큐브 구독 → Blue 큐브 퍼블리시)
        self.marker_sub = self.create_subscription(
            Marker, '/face_tracking/marker_robot', self.marker_callback, 10)
        
        # Filtered 얼굴 마커 구독 (조준선용 - 실제 얼굴 위치)
        self.face_marker_sub = self.create_subscription(
            Marker, '/face_tracking/marker_ekf', self.face_marker_callback, 10)
        
        self.ekf_marker_pub = self.create_publisher(
            Marker, '/face_tracking/marker_ekf_filtered', 10)
        self.ekf_text_pub = self.create_publisher(
            Marker, '/face_tracking/text_ekf_filtered', 10)
        
        # TCP 위치 시각화 마커 퍼블리셔 (보라색 큐브)
        self.tcp_marker_pub = self.create_publisher(
            Marker, '/robot_control/tcp_marker', 10)
        self.tcp_text_pub = self.create_publisher(
            Marker, '/robot_control/tcp_text', 10)
        
        # 비비탄 조준선 퍼블리셔 (빨간 라인: TCP → 얼굴)
        self.aim_line_pub = self.create_publisher(
            Marker, '/robot_control/aim_line', 10)
        
        # 얼굴 위치 저장 (조준선용)
        self.filtered_face_pos = None
        self.filtered_face_time = None  # 마지막 수신 시간
        
        # Servol 실시간 제어 퍼블리셔 (1kHz)
        if self.use_servol_rt:
            self.servol_pub = self.create_publisher(
                ServolRtStream, f'/{self.robot_id}/servol_rt_stream', 10)
            self.get_logger().info("⚡ Servol RT mode enabled (1kHz)")
        
        self.get_logger().info("="*60)
        mode_str = "Servol RT (1kHz)" if self.use_servol_rt else "amovel (10Hz)"
        self.get_logger().info(f"🤖 Robot Control Node - Cartesian Velocity Control [{mode_str}]")
        self.get_logger().info(f"  Robot: {self.robot_id} / {self.robot_model}")
        self.get_logger().info(f"  Control: K_p={self.k_p}, v_max={self.v_max}mm/s, Period={self.control_period*1000:.0f}ms")
        self.get_logger().info(f"  Safety: Dead zone={self.dead_zone}mm, Margin={self.safety_margin}mm, Z_min={self.safe_z_min}mm")
        if self.use_ekf:
            self.get_logger().info(f"  🔬 EKF: ON (Q={self.ekf_process_noise}, R={self.ekf_measurement_noise})")
        else:
            self.get_logger().info("  ⚠️  EKF: OFF")
        self.get_logger().info("  키: 's'=시작 추적, 'h'=홈, 'q'=종료")
        self.get_logger().info("=" * 60)
    
    def marker_callback(self, msg):
        """Red 큐브 마커 수신 - 로봇 좌표계 목표 위치 (6DOF)"""
        # 위치 (mm)
        target_x = msg.pose.position.x * 1000.0
        target_y = msg.pose.position.y * 1000.0
        target_z = msg.pose.position.z * 1000.0
        
        # Quaternion에서 Euler 각도로 변환
        q = msg.pose.orientation
        # Yaw (RZ)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        rz = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        
        # Pitch (RY)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            ry = math.degrees(math.copysign(math.pi / 2, sinp))
        else:
            ry = math.degrees(math.asin(sinp))
        
        raw_pos = [target_x, target_y, target_z]
        raw_rot = [0.0, ry, rz]  # Rx는 0으로 고정
        
        # face_tracking_node에서 이미 EKF 필터링된 데이터이므로 직접 사용
        self.target_pos = raw_pos + raw_rot  # XYZ + RxRyRz
        
        # TRACKING 모드로 자동 전환
        if self.state == "IDLE":
            self.get_logger().info("🎯 얼굴 감지! → 추적 모드")
            self.state = "TRACKING"
    
    def face_marker_callback(self, msg):
        """
        Filtered 얼굴 마커 수신 (조준선용 - 실제 얼굴 위치)
        /face_tracking/marker_ekf 토픽에서 Green 큐브 마커 수신
        """
        # 위치 (m -> mm)
        self.filtered_face_pos = [
            msg.pose.position.x * 1000.0,
            msg.pose.position.y * 1000.0,
            msg.pose.position.z * 1000.0
        ]
        self.filtered_face_time = time.time()  # 수신 시간 저장
    
    def track_face(self, target_pos, current_tcp):
        """
        Cartesian Space Velocity Control (6DOF)
        
        Args:
            target_pos: 그리퍼 끝점 목표 위치 [x, y, z, rx, ry, rz] in mm and degrees
            current_tcp: 현재 그리퍼 끝점 위치 (offset 적용됨) [x, y, z, rx, ry, rz]
        
        Returns:
            velocity: 속도 벡터 [vx, vy, vz] in mm/s, None if not safe
        """
        # 1. 안전 영역 체크 (XYZ만)
        if not self.is_safe_position(target_pos[:3]):
            self.get_logger().warn(f"⚠️ 안전 영역 밖: {target_pos}")
            return None
        
        # 2. 현재 그리퍼 끝점과 목표 그리퍼 끝점 비교
        current_pos = np.array(current_tcp[:3])
        target_array = np.array(target_pos[:3])  # XYZ만 사용
        
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
        tcp_pos는 이미 offset이 적용된 그리퍼 끝점 위치
        
        Args:
            tcp_pos: 그리퍼 끝점 [x, y, z, rx, ry, rz] in mm and degrees
        """
        # 큐브 마커 (그리퍼 끝점)
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
        marker.lifetime.nanosec = 500000000  # 0.5초
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
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
        text.text = "TCP"
        text.lifetime.sec = 0
        text.lifetime.nanosec = 500000000  # 0.5초
        self.tcp_text_pub.publish(text)
    
    def publish_aim_line(self, tcp_pos, face_pos):
        """
        비비탄 조준선 시각화 (빨간 점선: TCP 중앙 → Filtered 얼굴 마커 중앙)
        
        Args:
            tcp_pos: 그리퍼 끝점 [x, y, z, ...] in mm
            face_pos: 얼굴 위치 [x, y, z, ...] in mm
        """
        from geometry_msgs.msg import Point
        import numpy as np
        
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "aim_line"
        marker.id = 0
        marker.type = Marker.LINE_LIST  # 점선을 위해 LINE_LIST 사용
        marker.action = Marker.ADD
        
        # TCP (그리퍼 중앙 = 비비탄 발사점)
        tcp = np.array([tcp_pos[0], tcp_pos[1], tcp_pos[2]]) / 1000.0
        
        # 얼굴 위치 (Filtered 마커 중앙)
        face = np.array([face_pos[0], face_pos[1], face_pos[2]]) / 1000.0
        
        # 점선 생성: TCP → 얼굴을 여러 세그먼트로 나눔
        direction = face - tcp
        distance = np.linalg.norm(direction)
        
        if distance < 0.01:  # 너무 가까우면 무시
            return
        
        # 점선 파라미터
        dash_length = 0.03  # 30mm 선분
        gap_length = 0.02   # 20mm 간격
        segment_length = dash_length + gap_length
        num_segments = int(distance / segment_length)
        
        unit_dir = direction / distance
        
        points = []
        for i in range(num_segments + 1):
            start_t = i * segment_length
            end_t = start_t + dash_length
            
            if end_t > distance:
                end_t = distance
            if start_t >= distance:
                break
            
            p_start = tcp + unit_dir * start_t
            p_end = tcp + unit_dir * end_t
            
            points.append(Point(x=p_start[0], y=p_start[1], z=p_start[2]))
            points.append(Point(x=p_end[0], y=p_end[1], z=p_end[2]))
        
        marker.points = points
        marker.scale.x = 0.006  # 선 굵기 (6mm)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 0.9  # 빨간색
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000  # 0.5초
        self.aim_line_pub.publish(marker)
    
    def send_servol_command(self, target_tcp, velocity_norm):
        """
        Servol 실시간 명령 전송 (1kHz)
        
        Args:
            target_tcp: [x, y, z, rx, ry, rz] in mm and degrees
            velocity_norm: 속도 크기 (mm/s)
        """
        msg = ServolRtStream()
        msg.pos = target_tcp  # [x, y, z, rx, ry, rz]
        msg.vel = [velocity_norm, 0.0, 0.0, 0.0, 0.0, 0.0]  # 선속도만
        msg.acc = [self.a_max, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.time = 0.0  # 실시간 모드
        self.servol_pub.publish(msg)
    
    def is_safe_position(self, pos):
        """
        위치 안전 확인 (완화된 설정)
        
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
        
        # Y축 좌우 범위 제한 (로봇 작업 공간)
        if abs(y) > 800:  # 좌우 ±800mm 이내 (완화)
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
        from DSR_ROBOT2 import movej, movel, amovel, get_current_posx, mwait
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
                    try:
                        movej(node.start_joints, vel=60, acc=60)
                        mwait()
                        tcp_result = get_current_posx()
                        if tcp_result and len(tcp_result) > 0 and len(tcp_result[0]) >= 6:
                            current_tcp = list(tcp_result[0])
                            node.get_logger().info(f"✅ 추적 준비 완료! TCP: ({current_tcp[0]:.0f}, {current_tcp[1]:.0f}, {current_tcp[2]:.0f})mm")
                        else:
                            node.get_logger().warn("⚠️ TCP 읽기 실패 (빈 응답)")
                        node.state = "IDLE"  # 얼굴 감지 시 자동 TRACKING 전환
                    except Exception as e:
                        node.get_logger().error(f"❌ 시작 위치 이동 실패: {e}")
                        node.state = "IDLE"
                
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
                # 현재 TCP 위치 (에러 처리 강화)
                try:
                    tcp_result = get_current_posx()
                    if tcp_result and len(tcp_result) > 0 and len(tcp_result[0]) >= 6:
                        wrist_tcp = list(tcp_result[0])  # 손목 중심
                        # TCP offset 적용 (그리퍼 끝점 계산)
                        import math
                        rx_rad = math.radians(wrist_tcp[3])
                        ry_rad = math.radians(wrist_tcp[4])
                        rz_rad = math.radians(wrist_tcp[5])
                        # Z축 방향 벡터 계산 (228.6mm 앞으로)
                        offset_x = node.tcp_offset_z * (-math.sin(ry_rad))
                        offset_y = node.tcp_offset_z * (math.sin(rx_rad) * math.cos(ry_rad))
                        offset_z = node.tcp_offset_z * (math.cos(rx_rad) * math.cos(ry_rad))
                        
                        current_tcp = wrist_tcp.copy()
                        current_tcp[0] += offset_x
                        current_tcp[1] += offset_y
                        current_tcp[2] += offset_z
                        
                        # 디버그: offset 적용 확인
                        offset_magnitude = math.sqrt(offset_x**2 + offset_y**2 + offset_z**2)
                        node.get_logger().info(
                            f"🔧 Wrist: [{wrist_tcp[0]:.0f}, {wrist_tcp[1]:.0f}, {wrist_tcp[2]:.0f}] → "
                            f"Gripper: [{current_tcp[0]:.0f}, {current_tcp[1]:.0f}, {current_tcp[2]:.0f}] "
                            f"(offset: [{offset_x:.0f}, {offset_y:.0f}, {offset_z:.0f}] = {offset_magnitude:.1f}mm)",
                            throttle_duration_sec=2.0
                        )
                    else:
                        node.get_logger().warn("⚠️ TCP 읽기 실패 (빈 응답)")
                        continue
                except Exception as e:
                    node.get_logger().error(f"❌ TCP 읽기 에러: {e}")
                    continue
                
                # TCP 위치 시각화 (보라색 큐브 - 손목 위치)
                # 빨간색 Target과 같은 위치 (손목 기준)
                node.publish_tcp_marker(wrist_tcp)
                
                # 비비탄 조준선 시각화 (빨간 점선: TCP → 얼굴)
                # 얼굴 마커가 0.5초 이상 업데이트 없으면 자동 소멸
                face_timeout = 0.5  # 초
                if (node.filtered_face_pos is not None and 
                    node.filtered_face_time is not None and
                    (time.time() - node.filtered_face_time) < face_timeout):
                    node.publish_aim_line(wrist_tcp, node.filtered_face_pos)
                else:
                    # 타임아웃 - 얼굴 위치 초기화
                    node.filtered_face_pos = None
                
                # Velocity 계산 (손목 기준으로 제어)
                velocity = node.track_face(node.target_pos, wrist_tcp)
                
                # 안전 경고 간소화 (중복 제거)
                if velocity is None:
                    continue
                
                if velocity is not None:
                    # 그리퍼 끝점 목표 계산 (위치만 제어)
                    target_gripper = current_tcp.copy()
                    target_gripper[0] += velocity[0] * node.control_period
                    target_gripper[1] += velocity[1] * node.control_period
                    target_gripper[2] += velocity[2] * node.control_period
                    # 회전 완전 고정 (J6 최소 사용, J1~J5만 적극 사용)
                    target_gripper[3] = current_tcp[3]
                    target_gripper[4] = current_tcp[4]
                    target_gripper[5] = current_tcp[5]
                    
                    # 손목 중심 계산 (그리퍼 끝점에서 228.6mm 뒤로)
                    import math
                    rx_rad = math.radians(target_gripper[3])
                    ry_rad = math.radians(target_gripper[4])
                    offset_x = node.tcp_offset_z * (-math.sin(ry_rad))
                    offset_y = node.tcp_offset_z * (math.sin(rx_rad) * math.cos(ry_rad))
                    offset_z = node.tcp_offset_z * (math.cos(rx_rad) * math.cos(ry_rad))
                    
                    target_tcp = target_gripper.copy()
                    target_tcp[0] -= offset_x
                    target_tcp[1] -= offset_y
                    target_tcp[2] -= offset_z
                    
                    # 속도 노름 계산
                    v_norm = float(np.linalg.norm(velocity))
                    
                    # 로봇 이동
                    if node.use_servol_rt:
                        # Servol 실시간 제어 (1kHz, 부드러움)
                        node.send_servol_command(target_tcp, v_norm)
                    else:
                        # amovel 비동기 제어 (10Hz)
                        amovel(target_tcp, vel=v_norm, acc=node.a_max)
                    
                    # 오차 계산 (로그용 - XYZ만)
                    error = np.array(node.target_pos[:3]) - np.array(current_tcp[:3])
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
