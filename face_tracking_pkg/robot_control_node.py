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
from sensor_msgs.msg import JointState
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
        self.declare_parameter('velocity', 250.0)  # mm/s (추적 개선: 150→250)
        self.declare_parameter('acceleration', 400.0)  # mm/s² (추적 개선: 300→400)
        self.declare_parameter('k_p', 0.4)  # 비례 게인 (오버슈트 방지: 0.5→0.4)
        self.declare_parameter('dead_zone', 30.0)  # mm (떨림 방지: 20→30)
        self.declare_parameter('tcp_offset_z', 228.6)  # mm (RG2 그리퍼 TCP offset)
        self.declare_parameter('use_servol_rt', False)  # amovel 사용 (servol_rt는 DSR 드라이버 호환 문제)
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
        self.control_period = 0.02  # 50Hz (떨림 방지: 100Hz→50Hz)
        
        # Velocity Low-pass Filter (떨림 방지)
        self.velocity_filter_alpha = 0.5  # 0=이전값만, 1=새값만 (0.5=부드러움)
        self.prev_velocity = np.array([0.0, 0.0, 0.0])
        
        # PD 제어용 이전 오차 (D항)
        self.prev_error = np.array([0.0, 0.0, 0.0])
        self.k_d = 0.1  # 미분 게인 (떨림 억제)
        
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
        
        # 조준 거리 텍스트 퍼블리셔
        self.aim_distance_pub = self.create_publisher(
            Marker, '/robot_control/aim_distance', 10)
        
        # 얼굴 위치 저장 (조준선용)
        self.filtered_face_pos = None
        self.filtered_face_time = None  # 마지막 수신 시간
        
        # J6 제어용: 얼굴 이미지 X 좌표 (화면 중앙 유지)
        self.face_image_x = None  # 얼굴 중심 X 좌표 (pixel)
        self.face_image_y = None  # 얼굴 중심 Y 좌표 (pixel)
        self.image_center_x = 320.0  # 이미지 중앙 (640x480 기준)
        self.image_center_y = 240.0  # 이미지 중앙 Y
        
        # 시작 자세 저장 (누적 방지)
        self.start_rx = None
        self.start_rz = None
        
        # 조인트 상태 저장 (토픽 구독으로)
        self.current_joints_deg = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]  # degrees
        self.joint_state_sub = self.create_subscription(
            JointState, '/dsr01/joint_states', self.joint_state_callback, 10)
        
        # 얼굴 이미지 좌표 구독 (J6 제어용)
        from std_msgs.msg import Float32MultiArray
        self.face_sub = self.create_subscription(
            Float32MultiArray, '/face_detection/faces', self.face_image_callback, 10)
        
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
    
    def face_image_callback(self, msg):
        """얼굴 이미지 좌표 수신 (자세 제어용)"""
        if len(msg.data) >= 4:
            # [center_x, center_y, width, height]
            self.face_image_x = msg.data[0]
            self.face_image_y = msg.data[1]
    
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
            self.prev_error = error.copy()  # 오차 저장
            return None
        
        # 5. PD 제어: 속도 벡터 계산
        # P항: 오차에 비례
        p_term = error * self.k_p
        
        # D항: 오차 변화율에 비례 (떨림 억제)
        error_derivative = (error - self.prev_error) / self.control_period
        d_term = error_derivative * self.k_d
        self.prev_error = error.copy()
        
        # PD 합산
        velocity = p_term - d_term  # D항은 뺌 (급격한 변화 억제)
        
        # 6. 속도 크기 제한
        velocity_norm = np.linalg.norm(velocity)
        
        # 최소 속도 임계값 (떨림 방지)
        min_velocity = 5.0  # mm/s
        if velocity_norm < min_velocity and distance < self.dead_zone * 2:
            return None
        
        if velocity_norm > self.v_max:
            velocity = velocity * (self.v_max / velocity_norm)
        
        # 7. Low-pass filter (부드러운 모션)
        velocity = self.velocity_filter_alpha * velocity + (1 - self.velocity_filter_alpha) * self.prev_velocity
        self.prev_velocity = velocity.copy()
        
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
        
        # 거리 텍스트 마커 (TCP → Filtered 거리)
        # 중간 지점에 표시
        mid_point = (tcp + face) / 2
        
        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "aim_distance"
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = mid_point[0]
        text_marker.pose.position.y = mid_point[1]
        text_marker.pose.position.z = mid_point[2] + 0.05  # 약간 위로
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.04  # 텍스트 크기
        text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 1.0, 1.0  # 흰색
        text_marker.text = f"{distance:.2f}m"  # 미터 단위
        text_marker.lifetime.sec = 0
        text_marker.lifetime.nanosec = 500000000  # 0.5초
        self.aim_distance_pub.publish(text_marker)
    
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
                            # 방향 추종용: 시작 Rx, Rz 저장 (누적 방지)
                            node.start_rx = current_tcp[3]
                            node.start_rz = current_tcp[5]
                            node.get_logger().info(f"✅ 추적 준비 완료! TCP: ({current_tcp[0]:.0f}, {current_tcp[1]:.0f}, {current_tcp[2]:.0f})mm")
                            node.get_logger().info(f"   시작 자세: Rx={node.start_rx:.1f}°, Rz={node.start_rz:.1f}°")
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
                
                # ================================================================
                # 완전한 6DOF 추종 제어 (J1 우선 + 위치 + 자세)
                # ================================================================
                # 0단계: J1 우선 회전 (여유 공간 확보)
                # 1단계: 위치 제어 (X, Y, Z)
                # 2단계: 자세 제어 (Rx, Rz)
                # ================================================================
                
                if node.target_pos is None or node.filtered_face_pos is None:
                    continue
                
                # 현재 얼굴 위치와 TCP 위치
                face_pos = np.array(node.filtered_face_pos)  # mm (실제 얼굴)
                target_pos = np.array(node.target_pos[:3])  # mm (안전거리 적용된 목표)
                tcp_pos = np.array(current_tcp[:3])  # mm
                
                # ========================================
                # 0. J1 우선 제어 (베이스 회전 - 여유 공간 확보)
                # ========================================
                # 목표: 얼굴이 로봇 정면에 오도록 J1 회전
                # 현재 J1 읽기
                current_j1 = 0.0
                current_joints = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]  # 기본값
                try:
                    joint_result = get_current_posj()
                    # DSR API는 튜플 (joints, sol) 형태로 반환할 수 있음
                    if joint_result is not None:
                        if isinstance(joint_result, (list, tuple)):
                            if len(joint_result) >= 6:
                                # [j1, j2, j3, j4, j5, j6] 형태
                                current_joints = list(joint_result[:6])
                                current_j1 = float(current_joints[0])
                            elif len(joint_result) >= 1 and isinstance(joint_result[0], (list, tuple)):
                                # ((j1, j2, j3, j4, j5, j6), sol) 형태
                                current_joints = list(joint_result[0])
                                current_j1 = float(current_joints[0])
                except Exception as e:
                    node.get_logger().warn(f"⚠️ Joint 읽기 실패: {e}", throttle_duration_sec=2.0)
                
                # 얼굴 방향 각도 계산 (XY 평면에서)
                # atan2(Y, X) → 얼굴이 있는 방향
                face_angle_rad = math.atan2(face_pos[1], face_pos[0])
                face_angle_deg = math.degrees(face_angle_rad)
                
                # J1 목표 = 얼굴 방향 (로봇 정면이 얼굴을 향하도록)
                # 단, 급격한 회전 방지를 위해 점진적으로
                j1_error = face_angle_deg - current_j1
                
                # J1 제어 게인 (크면 빠르게 회전)
                j1_gain = 0.5  # 50% 반영 (더 적극적으로)
                j1_delta = j1_error * j1_gain
                j1_delta = max(-15.0, min(15.0, j1_delta))  # ±15°/cycle 제한 (증가)
                
                # J1 안전 범위 제한 (±150°)
                target_j1 = current_j1 + j1_delta
                target_j1 = max(-150.0, min(150.0, target_j1))
                
                # J1이 충분히 정렬되었는지 확인
                j1_aligned = abs(j1_error) < 10.0  # 10° 이내면 정렬됨 (더 엄격)
                
                # ========================================
                # 1. 위치 제어 (X, Y, Z)
                # ========================================
                # PD 제어로 속도 계산
                velocity = node.track_face(node.target_pos, current_tcp)
                
                if velocity is None:
                    # Dead zone 안이거나 안전 영역 밖
                    # 하지만 J1은 계속 정렬
                    if not j1_aligned:
                        # J1만 움직이기 (amovej 비동기)
                        target_joints = current_joints.copy()
                        target_joints[0] = target_j1
                        amovej(target_joints, vel=30.0, acc=60.0)
                        node.get_logger().info(
                            f"🔄 J1 정렬 | J1: {current_j1:.1f}° → {target_j1:.1f}° (오차: {j1_error:.1f}°)"
                        )
                    continue
                
                # 위치 업데이트 (J1 정렬 정도에 따라 속도 조절)
                # J1이 정렬 안 됐으면 위치 이동 속도 감소
                speed_factor = 1.0 if j1_aligned else 0.5
                
                target_gripper = current_tcp.copy()
                target_gripper[0] += velocity[0] * node.control_period * speed_factor
                target_gripper[1] += velocity[1] * node.control_period * speed_factor
                target_gripper[2] += velocity[2] * node.control_period * speed_factor
                
                # ========================================
                # 2. 자세 제어 (Rx, Ry, Rz)
                # ========================================
                # TCP → 얼굴 방향 벡터
                direction = face_pos - tcp_pos
                distance = np.linalg.norm(direction)
                
                if distance < 100.0:  # 10cm 미만은 무시
                    continue
                
                direction_norm = direction / distance
                
                # --- Rz (J6): 좌우 회전 (이미지 X 기반) ---
                rz_delta = 0.0
                if node.face_image_x is not None:
                    face_x_error = node.face_image_x - node.image_center_x  # pixel
                    rz_gain = 0.05  # deg/pixel
                    rz_delta = face_x_error * rz_gain
                    rz_delta = max(-25.0, min(25.0, rz_delta))
                
                # start_rz가 None이면 현재값으로 초기화
                if node.start_rz is None:
                    node.start_rz = current_tcp[5]
                if node.start_rx is None:
                    node.start_rx = current_tcp[3]
                
                base_rz = node.start_rz
                target_gripper[5] = base_rz + rz_delta
                
                # --- Rx (J4): 상하 기울임 (Pitch 기반) ---
                horizontal_dist = math.sqrt(direction[0]**2 + direction[1]**2)
                if horizontal_dist > 10.0:  # 너무 가까우면 무시
                    pitch_rad = math.atan2(-direction[2], horizontal_dist)
                    pitch_deg = math.degrees(pitch_rad)
                    rx_delta = pitch_deg * 0.3  # 30% 게인
                    rx_delta = max(-25.0, min(25.0, rx_delta))
                else:
                    rx_delta = 0.0
                
                base_rx = node.start_rx
                target_gripper[3] = base_rx + rx_delta
                
                # --- Ry (J5): 고정 (IK가 자동 계산) ---
                target_gripper[4] = current_tcp[4]
                
                # ========================================
                # 3. 안전 영역 클램핑
                # ========================================
                # XY 반경 체크
                r_xy = math.sqrt(target_gripper[0]**2 + target_gripper[1]**2)
                if r_xy < node.safe_r_min:
                    scale = node.safe_r_min / r_xy if r_xy > 0 else 1.0
                    target_gripper[0] *= scale
                    target_gripper[1] *= scale
                elif r_xy > node.safe_r_max:
                    scale = node.safe_r_max / r_xy
                    target_gripper[0] *= scale
                    target_gripper[1] *= scale
                
                # Z 높이 클램핑
                target_gripper[2] = max(node.safe_z_min, min(800.0, target_gripper[2]))
                
                # ========================================
                # 4. 손목 TCP 계산 (그리퍼 → 손목)
                # ========================================
                rx_rad = math.radians(target_gripper[3])
                ry_rad = math.radians(target_gripper[4])
                offset_x = node.tcp_offset_z * (-math.sin(ry_rad))
                offset_y = node.tcp_offset_z * (math.sin(rx_rad) * math.cos(ry_rad))
                offset_z = node.tcp_offset_z * (math.cos(rx_rad) * math.cos(ry_rad))
                
                target_tcp = target_gripper.copy()
                target_tcp[0] -= offset_x
                target_tcp[1] -= offset_y
                target_tcp[2] -= offset_z
                
                # ========================================
                # 5. 로봇 이동 명령 (amovel + 필요시 J1 보정)
                # ========================================
                v_norm = float(np.linalg.norm(velocity))
                v_norm = max(30.0, min(node.v_max, v_norm))  # 30~v_max mm/s
                
                # J1 정렬이 안 됐으면 속도 감소
                if not j1_aligned:
                    v_norm *= 0.5
                
                amovel(target_tcp, vel=v_norm, acc=node.a_max)
                
                # ========================================
                # 6. 로그 출력
                # ========================================
                pos_error = np.linalg.norm(target_pos - tcp_pos)
                j1_status = "✅" if j1_aligned else "🔄"
                node.get_logger().info(
                    f"🎯 6DOF | Err: {pos_error:.0f}mm | "
                    f"J1: {current_j1:.1f}°→{target_j1:.1f}°{j1_status} | "
                    f"FaceAngle: {face_angle_deg:.1f}° | "
                    f"Vel: {v_norm:.0f}mm/s"
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
