#!/usr/bin/env python3
"""
Face Tracking Node - 2D 좌표를 3D 로봇 좌표계로 변환

TF2를 사용하여 카메라 좌표계 → 로봇 베이스 좌표계 변환

Subscribed Topics:
  /face_detection/faces - 얼굴 2D 좌표 (from face_detection_node)
  /camera/camera/aligned_depth_to_color/image_raw - 깊이 이미지
  /camera/camera/color/camera_info - 카메라 내부 파라미터

Published Topics:
  /face_tracking/marker - RViz 마커 (카메라 프레임, 초록색)
  /face_tracking/marker_robot - RViz 마커 (로봇 베이스 프레임, 빨간색) ← 로봇 목표 위치!
  /face_tracking/line - 카메라→얼굴 연결선 (노란색)
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from face_tracking_pkg.face_tracking_ekf import FaceTrackingEKF


class FaceTrackingNode(Node):
    def __init__(self):
        super().__init__('face_tracking_node')
        
        self.bridge = CvBridge()
        
        # 파라미터 선언
        self.declare_parameter('target_offset_mm', 650.0)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('robot_frame', 'base_link')
        
        self.target_offset_mm = self.get_parameter('target_offset_mm').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        
        # 데이터 저장
        self.faces_data = []
        self.depth_frame = None
        self.intrinsics = None
        
        # TF2 버퍼 및 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 구독자
        self.faces_sub = self.create_subscription(
            Float32MultiArray, '/face_detection/faces', self.faces_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        # 발행자
        self.marker_pub = self.create_publisher(Marker, '/face_tracking/marker', 10)
        self.marker_robot_pub = self.create_publisher(Marker, '/face_tracking/marker_robot', 10)
        self.marker_ekf_pub = self.create_publisher(Marker, '/face_tracking/marker_ekf', 10)  # EKF 필터링된 카메라 프레임
        # 텍스트 전용 토픽
        self.text_pub = self.create_publisher(Marker, '/face_tracking/text', 10)
        self.text_ekf_pub = self.create_publisher(Marker, '/face_tracking/text_ekf', 10)
        self.text_robot_pub = self.create_publisher(Marker, '/face_tracking/text_robot', 10)
        self.line_pub = self.create_publisher(Marker, '/face_tracking/line', 10)
        
        # EKF 초기화 (카메라 프레임 좌표 필터링용)
        self.camera_ekf = FaceTrackingEKF(dt=0.033, dim=3)
        
        # 얼굴 감지 상태 플래그
        self.face_detected = False
        
        # 타이머: 트래킹 루프 (30Hz) - 병목 해결
        self.timer = self.create_timer(0.033, self.tracking_loop)
        
        # 성능 측정
        self.loop_count = 0
        self.last_fps_time = self.get_clock().now()
        self.tracking_fps = 0.0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔄 Face Tracking Node (TF2) 시작! [30Hz]")
        self.get_logger().info("  🟢 초록 마커: 카메라 프레임")
        self.get_logger().info("  🔴 빨간 마커: 로봇 베이스 프레임 (목표)")
        self.get_logger().info("=" * 60)
    
    def faces_callback(self, msg):
        self.faces_data = list(msg.data)
    
    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"깊이 이미지 변환 실패: {e}")
    
    def camera_info_callback(self, msg):
        self.intrinsics = {
            'fx': msg.k[0], 'fy': msg.k[4],
            'ppx': msg.k[2], 'ppy': msg.k[5]
        }
    
    def get_3d_position(self, center_x, center_y):
        """2D 픽셀 좌표 → 3D 카메라 좌표 (mm)"""
        if self.depth_frame is None or self.intrinsics is None:
            return None
        
        x, y = int(center_x), int(center_y)
        h, w = self.depth_frame.shape
        
        if x < 10 or x >= w - 10 or y < 10 or y >= h - 10:
            return None
        
        # 개선: 9x9 영역 사용 (3x3는 너무 작아서 노이즈에 민감)
        depth_region = self.depth_frame[y-4:y+5, x-4:x+5]
        valid_depths = depth_region[depth_region > 0]
        
        if len(valid_depths) == 0:
            return None
        
        # 개선: Trimmed Mean (상하위 20% 제거 후 평균)
        # median보다 빠르면서도 이상치에 강건
        if len(valid_depths) >= 5:
            valid_depths_sorted = np.sort(valid_depths)
            trim_count = max(1, len(valid_depths) // 5)  # 20% 제거
            trimmed = valid_depths_sorted[trim_count:-trim_count]
            depth_mm = float(np.mean(trimmed))
        else:
            # 샘플 적으면 그냥 mean
            depth_mm = float(np.mean(valid_depths))
        
        camera_x = (center_x - self.intrinsics['ppx']) * depth_mm / self.intrinsics['fx']
        camera_y = (center_y - self.intrinsics['ppy']) * depth_mm / self.intrinsics['fy']
        camera_z = depth_mm
        
        return np.array([camera_x, camera_y, camera_z])
    
    def camera_to_robot_tf2(self, camera_pos_mm):
        """TF2를 사용해 카메라 좌표 → 로봇 베이스 좌표 변환"""
        try:
            point_camera = PointStamped()
            point_camera.header.frame_id = self.camera_frame
            point_camera.header.stamp = self.get_clock().now().to_msg()
            point_camera.point.x = camera_pos_mm[0] / 1000.0
            point_camera.point.y = camera_pos_mm[1] / 1000.0
            point_camera.point.z = camera_pos_mm[2] / 1000.0
            
            # 최적화: 타임아웃 단축 (0.1초 → 0.01초)
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame, self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.01)
            )
            
            point_base = tf2_geometry_msgs.do_transform_point(point_camera, transform)
            
            return np.array([
                point_base.point.x * 1000.0,
                point_base.point.y * 1000.0,
                point_base.point.z * 1000.0
            ])
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF2 변환 실패: {e}")
            return None
    
    def delete_all_markers(self):
        """모든 마커와 텍스트를 삭제"""
        marker_ids = [
            (self.marker_pub, "face_camera"),
            (self.text_pub, "face_camera_text"),
            (self.marker_ekf_pub, "face_camera_ekf"),
            (self.text_ekf_pub, "face_camera_ekf_text"),
            (self.marker_robot_pub, "face_robot_target"),
            (self.text_robot_pub, "face_robot_text"),
            (self.line_pub, "face_line")
        ]
        
        for pub, ns in marker_ids:
            marker = Marker()
            marker.header.frame_id = self.robot_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = 0
            marker.action = Marker.DELETE
            pub.publish(marker)
    
    def publish_camera_marker(self, camera_pos):
        """카메라 프레임 마커 (초록색 큐브) - base_link 프레임으로 변환하여 표시"""
        # TF 변환으로 base_link 좌표 얻기
        robot_pos = self.camera_to_robot_tf2(camera_pos)
        if robot_pos is None:
            return
        
        marker = Marker()
        marker.header.frame_id = self.robot_frame  # base_link 사용
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_camera"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = robot_pos[0] / 1000.0
        marker.pose.position.y = robot_pos[1] / 1000.0
        marker.pose.position.z = robot_pos[2] / 1000.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.08
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 0.5
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000  # 0.5초
        self.marker_pub.publish(marker)
        
        # 텍스트 마커 (별도 토픽) - base_link
        text = Marker()
        text.header.frame_id = self.robot_frame
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "face_camera_text"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = robot_pos[0] / 1000.0
        text.pose.position.y = robot_pos[1] / 1000.0
        text.pose.position.z = robot_pos[2] / 1000.0 + 0.08
        text.pose.orientation.w = 1.0
        text.scale.z = 0.04
        text.color.r, text.color.g, text.color.b, text.color.a = 0.0, 1.0, 0.0, 1.0
        text.text = "Raw"
        text.lifetime.sec = 0
        text.lifetime.nanosec = 500000000  # 0.5초
        self.text_pub.publish(text)
    
    def publish_camera_ekf_marker(self, filtered_pos):
        """카메라 프레임 EKF 필터링 마커 (청록색 큐브) - base_link 프레임으로 변환"""
        # TF 변환으로 base_link 좌표 얻기
        robot_pos = self.camera_to_robot_tf2(filtered_pos)
        if robot_pos is None:
            return
        
        marker = Marker()
        marker.header.frame_id = self.robot_frame  # base_link 사용
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_camera_ekf"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = robot_pos[0] / 1000.0
        marker.pose.position.y = robot_pos[1] / 1000.0
        marker.pose.position.z = robot_pos[2] / 1000.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.10
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.8, 0.8, 0.5
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000  # 0.5초
        self.marker_ekf_pub.publish(marker)
        
        # 텍스트 마커 (별도 토픽) - base_link
        text = Marker()
        text.header.frame_id = self.robot_frame
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "face_camera_ekf_text"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = robot_pos[0] / 1000.0
        text.pose.position.y = robot_pos[1] / 1000.0
        text.pose.position.z = robot_pos[2] / 1000.0 + 0.10
        text.pose.orientation.w = 1.0
        text.scale.z = 0.04
        text.color.r, text.color.g, text.color.b, text.color.a = 0.0, 1.0, 1.0, 1.0
        text.text = "Filtered"
        text.lifetime.sec = 0
        text.lifetime.nanosec = 500000000  # 0.5초
        self.text_ekf_pub.publish(text)
    
    def publish_robot_marker(self, robot_pos):
        """로봇 베이스 프레임 마커 (빨간색 큐브)"""
        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_robot_target"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = robot_pos[0] / 1000.0
        marker.pose.position.y = robot_pos[1] / 1000.0
        marker.pose.position.z = robot_pos[2] / 1000.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.10
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 0.5
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000  # 0.5초
        self.marker_robot_pub.publish(marker)
        
        # 텍스트 마커 (별도 토픽)
        text = Marker()
        text.header.frame_id = self.robot_frame
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "face_robot_text"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = robot_pos[0] / 1000.0
        text.pose.position.y = robot_pos[1] / 1000.0
        text.pose.position.z = robot_pos[2] / 1000.0 + 0.10
        text.pose.orientation.w = 1.0
        text.scale.z = 0.05
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 0.0, 0.0, 1.0
        text.text = "Target"
        text.lifetime.sec = 0
        text.lifetime.nanosec = 500000000  # 0.5초
        self.text_robot_pub.publish(text)
    
    def publish_line_marker(self, camera_pos):
        """
        카메라→얼굴 투영 라인 (노란색) 
        공학적으로 정확한 투영: RGB 렌즈 중심 → 얼굴 위치
        base_link 프레임에서 TF 변환을 사용하여 정확히 마커 중심을 통과
        """
        try:
            # RGB 렌즈 위치를 camera_link 프레임에서 base_link로 변환
            # D435i: RGB 렌즈는 camera_link 중심에서 약 Y=-15mm 위치
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,  # base_link
                "camera_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # RGB 렌즈 위치 (camera_link 기준: Y=-15mm)
            # camera_link: X=앞, Y=왼쪽, Z=위
            from geometry_msgs.msg import PointStamped
            rgb_point = PointStamped()
            rgb_point.header.frame_id = "camera_link"
            rgb_point.header.stamp = self.get_clock().now().to_msg()
            rgb_point.point.x = 0.0
            rgb_point.point.y = -0.015  # RGB 렌즈 오프셋 (camera_link에서 오른쪽)
            rgb_point.point.z = 0.0
            
            # TF 변환
            from tf2_geometry_msgs import do_transform_point
            rgb_transformed = do_transform_point(rgb_point, transform)
            
            rgb_origin_robot = [
                rgb_transformed.point.x * 1000.0,
                rgb_transformed.point.y * 1000.0,
                rgb_transformed.point.z * 1000.0
            ]
        except Exception as e:
            self.get_logger().warn(f"RGB lens TF failed: {e}", throttle_duration_sec=5.0)
            return
        
        # 얼굴 위치도 TF 변환
        face_pos_robot = self.camera_to_robot_tf2(camera_pos)
        if face_pos_robot is None:
            return
        
        marker = Marker()
        marker.header.frame_id = self.robot_frame  # base_link
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_line"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # p1: RGB 렌즈 중심 (base_link 좌표)
        p1 = Point()
        p1.x = rgb_origin_robot[0] / 1000.0
        p1.y = rgb_origin_robot[1] / 1000.0
        p1.z = rgb_origin_robot[2] / 1000.0
        
        # p2: 얼굴 위치 (base_link 좌표)
        p2 = Point()
        p2.x = face_pos_robot[0] / 1000.0
        p2.y = face_pos_robot[1] / 1000.0
        p2.z = face_pos_robot[2] / 1000.0
        
        marker.points = [p1, p2]
        marker.scale.x = 0.01
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0
        marker.lifetime.nanosec = 500000000
        self.line_pub.publish(marker)
    
    def tracking_loop(self):
        """메인 트래킹 루프 (30Hz)"""
        # FPS 측정
        self.loop_count += 1
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_fps_time).nanoseconds / 1e9
        
        # 성공/실패 카운터 초기화
        if not hasattr(self, 'success_count'):
            self.success_count = 0
            self.fail_count = 0
        
        if time_diff >= 1.0:
            self.tracking_fps = self.loop_count / time_diff
            success_rate = (self.success_count / self.loop_count * 100) if self.loop_count > 0 else 0
            self.get_logger().info(
                f"📊 Loop: {self.tracking_fps:.1f}Hz | "
                f"3D Success: {self.success_count}/{self.loop_count} ({success_rate:.1f}%)"
            )
            self.loop_count = 0
            self.success_count = 0
            self.fail_count = 0
            self.last_fps_time = current_time
        
        # 얼굴 감지 실패 시 모든 마커 삭제
        if len(self.faces_data) < 4:
            if self.face_detected:  # 이전에 감지되었다면 삭제 마커 발행
                self.delete_all_markers()
                self.face_detected = False
            return
        
        self.face_detected = True
        
        center_x, center_y = self.faces_data[0], self.faces_data[1]
        
        camera_pos = self.get_3d_position(center_x, center_y)
        if camera_pos is None:
            self.fail_count += 1
            return
        
        self.success_count += 1
        
        # EKF 업데이트 (측정값으로 보정)
        camera_pos_array = np.array(camera_pos)
        
        if not self.camera_ekf.initialized:
            self.camera_ekf.initialize(camera_pos_array.tolist())
        else:
            # Predict & Update
            self.camera_ekf.predict()
            self.camera_ekf.update(camera_pos_array.tolist())
        
        # Raw 초록색 마커 (원본)
        self.publish_camera_marker(camera_pos)
        
        # EKF 청록색 마커 (측정값 있을 때만)
        filtered_camera_pos = self.camera_ekf.get_position()
        self.publish_camera_ekf_marker(filtered_camera_pos)
        
        # Line 마커도 EKF 필터된 위치 사용 (청록색 마커와 싱크)
        self.publish_line_marker(filtered_camera_pos)
        
        # 목표 위치 = 필터된 얼굴 위치에서 안전거리만큼 떨어진 위치
        depth = abs(filtered_camera_pos[2])  # Z축 깊이 (mm)
        if depth < 100.0:  # 10cm 미만은 무시
            return
        
        # 안전거리: 590mm (얼굴에서 59cm 떨어진 위치가 목표)
        safety_distance = 590.0
        distance = np.linalg.norm(filtered_camera_pos)
        direction = filtered_camera_pos / distance
        
        # 목표 = 얼굴에서 안전거리만큼 카메라 방향으로
        target_camera_pos = filtered_camera_pos - direction * safety_distance
        
        robot_pos_xyz = self.camera_to_robot_tf2(target_camera_pos)
        if robot_pos_xyz is None:
            return
        
        # ============================================
        # 안전구역 클램핑 (Target이 안전 범위 내에 있도록)
        # ============================================
        # 로봇 도달 가능 범위 (m0609: 900mm reach)
        safe_r_min = 300.0   # 최소 반경 (mm) - 로봇 몸통 충돌 방지
        safe_r_max = 850.0   # 최대 반경 (mm) - 도달 한계
        safe_z_min = 150.0   # 최소 높이 (mm) - 테이블 충돌 방지
        safe_z_max = 800.0   # 최대 높이 (mm)
        
        # XY 평면 반경 계산
        r_xy = np.sqrt(robot_pos_xyz[0]**2 + robot_pos_xyz[1]**2)
        
        # 반경 클램핑
        if r_xy < safe_r_min:
            scale = safe_r_min / r_xy if r_xy > 0 else 1.0
            robot_pos_xyz[0] *= scale
            robot_pos_xyz[1] *= scale
            self.get_logger().warn(f"⚠️ Target 반경 클램핑: {r_xy:.0f} → {safe_r_min:.0f}mm", throttle_duration_sec=2.0)
        elif r_xy > safe_r_max:
            scale = safe_r_max / r_xy
            robot_pos_xyz[0] *= scale
            robot_pos_xyz[1] *= scale
            self.get_logger().warn(f"⚠️ Target 반경 클램핑: {r_xy:.0f} → {safe_r_max:.0f}mm", throttle_duration_sec=2.0)
        
        # Z 높이 클램핑
        if robot_pos_xyz[2] < safe_z_min:
            self.get_logger().warn(f"⚠️ Target Z 클램핑: {robot_pos_xyz[2]:.0f} → {safe_z_min:.0f}mm", throttle_duration_sec=2.0)
            robot_pos_xyz[2] = safe_z_min
        elif robot_pos_xyz[2] > safe_z_max:
            self.get_logger().warn(f"⚠️ Target Z 클램핑: {robot_pos_xyz[2]:.0f} → {safe_z_max:.0f}mm", throttle_duration_sec=2.0)
            robot_pos_xyz[2] = safe_z_max
        
        # 디버그: 목표 위치 로그
        self.get_logger().info(
            f"📍 Target: [{robot_pos_xyz[0]:.0f}, {robot_pos_xyz[1]:.0f}, {robot_pos_xyz[2]:.0f}]mm | "
            f"Depth: {depth:.0f}mm, Safety: {safety_distance:.0f}mm",
            throttle_duration_sec=1.0
        )
        
        # TCP 회전 계산 (얼굴을 향하도록) - EKF 필터링된 위치 사용!
        # Z축 기준 회전 (팔 끝이 얼굴을 가리키도록)
        face_robot_pos = self.camera_to_robot_tf2(filtered_camera_pos)
        if face_robot_pos is not None:
            # 목표에서 얼굴로의 방향
            vec_to_face = np.array(face_robot_pos) - np.array(robot_pos_xyz)
            distance_to_face = np.linalg.norm(vec_to_face)
            if distance_to_face > 10.0:
                # Z축 회전 (Yaw) 계산
                rz = math.degrees(math.atan2(vec_to_face[1], vec_to_face[0]))
                # Y축 회전 (Pitch) 계산
                ry = -math.degrees(math.atan2(vec_to_face[2], math.sqrt(vec_to_face[0]**2 + vec_to_face[1]**2)))
                robot_pos = robot_pos_xyz + [0.0, ry, rz]
            else:
                robot_pos = robot_pos_xyz + [0.0, 0.0, 0.0]
        else:
            robot_pos = robot_pos_xyz + [0.0, 0.0, 0.0]
        
        self.publish_robot_marker(robot_pos[:3])  # XYZ만 마커로


def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
