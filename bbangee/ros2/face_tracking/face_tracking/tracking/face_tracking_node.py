#!/usr/bin/env python3
"""
Face Tracking Node - 2D→3D 변환 및 EKF 필터링

2D 얼굴 좌표를 3D 로봇 좌표계로 변환하고 EKF 필터링 적용

Subscribed Topics:
    /face_detection/faces - 얼굴 2D 좌표 [cx, cy, w, h]
    /camera/camera/aligned_depth_to_color/image_raw - 깊이 이미지
    /camera/camera/color/camera_info - 카메라 내부 파라미터

Published Topics:
    /face_tracking/marker - 카메라 프레임 마커 (초록)
    /face_tracking/marker_robot - 로봇 프레임 마커 (빨강) ← 로봇 목표!
    /face_tracking/marker_ekf - EKF 필터링 마커 (청록)
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

from .ekf_filter import EKFFilter
from ..utils.constants import SAFETY_LIMITS, SAFETY_DISTANCE


class FaceTrackingNode(Node):
    """얼굴 추적 노드 - 3D 변환 및 EKF 필터링"""
    
    def __init__(self):
        super().__init__('face_tracking_node')
        
        self.bridge = CvBridge()
        
        # 파라미터
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
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # EKF 필터 (카메라 프레임)
        self.ekf = EKFFilter(dt=0.033, dim=3)
        
        # 상태
        self.face_detected = False
        
        # 성능 측정
        self.loop_count = 0
        self.success_count = 0
        self.last_fps_time = self.get_clock().now()
        
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
        self.marker_ekf_pub = self.create_publisher(Marker, '/face_tracking/marker_ekf', 10)
        self.line_pub = self.create_publisher(Marker, '/face_tracking/line', 10)
        # 텍스트 마커 퍼블리셔 (원본 코드 복원)
        self.text_pub = self.create_publisher(Marker, '/face_tracking/text', 10)
        self.text_ekf_pub = self.create_publisher(Marker, '/face_tracking/text_ekf', 10)
        self.text_robot_pub = self.create_publisher(Marker, '/face_tracking/text_robot', 10)
        
        # 타이머 (100Hz)
        self.timer = self.create_timer(0.01, self.tracking_loop)
        
        self._print_startup_info()
    
    def _print_startup_info(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔄 Face Tracking Node 시작! [100Hz]")
        self.get_logger().info("  🟢 초록 마커: Raw 위치")
        self.get_logger().info("  🔵 청록 마커: EKF 필터링")
        self.get_logger().info("  🔴 빨간 마커: 로봇 목표")
        self.get_logger().info("=" * 60)
    
    # ========================================
    # 콜백
    # ========================================
    def faces_callback(self, msg):
        self.faces_data = list(msg.data)
    
    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"깊이 변환 실패: {e}")
    
    def camera_info_callback(self, msg):
        self.intrinsics = {
            'fx': msg.k[0], 'fy': msg.k[4],
            'ppx': msg.k[2], 'ppy': msg.k[5]
        }
    
    # ========================================
    # 3D 변환
    # ========================================
    def get_3d_position(self, center_x: float, center_y: float) -> np.ndarray:
        """
        2D 픽셀 → 3D 카메라 좌표 (mm)
        
        핀홀 카메라 모델:
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
        """
        if self.depth_frame is None or self.intrinsics is None:
            return None
        
        x, y = int(center_x), int(center_y)
        h, w = self.depth_frame.shape
        
        if x < 10 or x >= w - 10 or y < 10 or y >= h - 10:
            return None
        
        # 9x9 영역 Trimmed Mean (이상치 제거)
        depth_region = self.depth_frame[y-4:y+5, x-4:x+5]
        valid_depths = depth_region[depth_region > 0]
        
        if len(valid_depths) == 0:
            return None
        
        if len(valid_depths) >= 5:
            sorted_depths = np.sort(valid_depths)
            trim = max(1, len(valid_depths) // 5)
            depth_mm = float(np.mean(sorted_depths[trim:-trim]))
        else:
            depth_mm = float(np.mean(valid_depths))
        
        # 3D 변환
        camera_x = (center_x - self.intrinsics['ppx']) * depth_mm / self.intrinsics['fx']
        camera_y = (center_y - self.intrinsics['ppy']) * depth_mm / self.intrinsics['fy']
        camera_z = depth_mm
        
        return np.array([camera_x, camera_y, camera_z])
    
    def camera_to_robot(self, camera_pos_mm: np.ndarray) -> np.ndarray:
        """TF2로 카메라 → 로봇 좌표 변환"""
        try:
            point_camera = PointStamped()
            point_camera.header.frame_id = self.camera_frame
            point_camera.header.stamp = self.get_clock().now().to_msg()
            point_camera.point.x = camera_pos_mm[0] / 1000.0
            point_camera.point.y = camera_pos_mm[1] / 1000.0
            point_camera.point.z = camera_pos_mm[2] / 1000.0
            
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
        except Exception as e:
            return None
    
    # ========================================
    # 안전 영역 클램핑
    # ========================================
    def clamp_to_safety(self, pos: np.ndarray) -> np.ndarray:
        """안전 영역 내로 클램핑"""
        result = pos.copy()
        
        # XY 평면 반경
        r_xy = np.sqrt(pos[0]**2 + pos[1]**2)
        
        if r_xy < SAFETY_LIMITS['r_min']:
            scale = SAFETY_LIMITS['r_min'] / r_xy if r_xy > 0 else 1.0
            result[0] *= scale
            result[1] *= scale
        elif r_xy > SAFETY_LIMITS['r_max']:
            scale = SAFETY_LIMITS['r_max'] / r_xy
            result[0] *= scale
            result[1] *= scale
        
        # Z 높이
        result[2] = np.clip(result[2], SAFETY_LIMITS['z_min'], SAFETY_LIMITS['z_max'])
        
        return result
    
    # ========================================
    # 마커 발행
    # ========================================
    def publish_marker(self, pos_mm: np.ndarray, ns: str, color: tuple, publisher, text_label: str = None, text_publisher=None):
        """마커 발행 헬퍼 (텍스트 마커 포함)"""
        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = pos_mm[0] / 1000.0
        marker.pose.position.y = pos_mm[1] / 1000.0
        marker.pose.position.z = pos_mm[2] / 1000.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.08
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 0.5
        marker.lifetime.nanosec = 500000000
        publisher.publish(marker)
        
        # 텍스트 마커 발행 (원본 코드 복원)
        if text_label and text_publisher:
            text = Marker()
            text.header.frame_id = self.robot_frame
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = ns + "_text"
            text.id = 0
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pos_mm[0] / 1000.0
            text.pose.position.y = pos_mm[1] / 1000.0
            text.pose.position.z = pos_mm[2] / 1000.0 + 0.08  # 마커 위에 표시
            text.pose.orientation.w = 1.0
            text.scale.z = 0.04
            text.color.r, text.color.g, text.color.b, text.color.a = *color, 1.0
            text.text = text_label
            text.lifetime.nanosec = 500000000
            text_publisher.publish(text)
    
    def publish_line(self, camera_pos_mm: np.ndarray):
        """
        카메라 → 얼굴 투영 라인 (노란색)
        
        RGB 렌즈 중심 → 얼굴 위치까지의 라인
        원본 코드 복원: camera_link 프레임에서 RGB 렌즈 오프셋 적용
        """
        try:
            # RGB 렌즈 위치를 camera_link에서 base_link로 변환
            # D435i: RGB 렌즈는 camera_link 중심에서 Y=-15mm 위치
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame, "camera_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            rgb_point = PointStamped()
            rgb_point.header.frame_id = "camera_link"
            rgb_point.header.stamp = self.get_clock().now().to_msg()
            rgb_point.point.x = 0.0
            rgb_point.point.y = -0.015  # RGB 렌즈 오프셋 (camera_link에서 오른쪽)
            rgb_point.point.z = 0.0
            
            rgb_transformed = tf2_geometry_msgs.do_transform_point(rgb_point, transform)
            
            rgb_origin_robot = np.array([
                rgb_transformed.point.x * 1000.0,
                rgb_transformed.point.y * 1000.0,
                rgb_transformed.point.z * 1000.0
            ])
        except Exception as e:
            self.get_logger().warn(f"RGB lens TF failed: {e}", throttle_duration_sec=5.0)
            return
        
        # 얼굴 위치도 TF 변환
        face_pos_robot = self.camera_to_robot(camera_pos_mm)
        if face_pos_robot is None:
            return
        
        marker = Marker()
        marker.header.frame_id = self.robot_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        p1, p2 = Point(), Point()
        p1.x = rgb_origin_robot[0] / 1000.0
        p1.y = rgb_origin_robot[1] / 1000.0
        p1.z = rgb_origin_robot[2] / 1000.0
        p2.x = face_pos_robot[0] / 1000.0
        p2.y = face_pos_robot[1] / 1000.0
        p2.z = face_pos_robot[2] / 1000.0
        
        marker.points = [p1, p2]
        marker.scale.x = 0.01
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0
        marker.lifetime.nanosec = 500000000
        self.line_pub.publish(marker)
    
    def delete_markers(self):
        """모든 마커 삭제 (텍스트 포함)"""
        # 큐브 마커 삭제
        for pub, ns in [(self.marker_pub, "face_raw"), 
                        (self.marker_ekf_pub, "face_ekf"),
                        (self.marker_robot_pub, "face_target")]:
            marker = Marker()
            marker.header.frame_id = self.robot_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.action = Marker.DELETE
            pub.publish(marker)
        
        # 텍스트 마커 삭제 (원본 코드 복원)
        for pub, ns in [(self.text_pub, "face_raw_text"),
                        (self.text_ekf_pub, "face_ekf_text"),
                        (self.text_robot_pub, "face_target_text")]:
            marker = Marker()
            marker.header.frame_id = self.robot_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.action = Marker.DELETE
            pub.publish(marker)
    
    # ========================================
    # 메인 루프
    # ========================================
    def tracking_loop(self):
        """메인 추적 루프 (100Hz)"""
        # FPS 측정
        self.loop_count += 1
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_fps_time).nanoseconds / 1e9
        
        if time_diff >= 1.0:
            success_rate = (self.success_count / self.loop_count * 100) if self.loop_count > 0 else 0
            self.get_logger().info(
                f"📊 {self.loop_count/time_diff:.1f}Hz | 3D: {success_rate:.1f}%",
                throttle_duration_sec=2.0)
            self.loop_count = 0
            self.success_count = 0
            self.last_fps_time = current_time
        
        # 얼굴 데이터 확인
        if len(self.faces_data) < 4:
            if self.face_detected:
                self.delete_markers()
                self.face_detected = False
            return
        
        self.face_detected = True
        center_x, center_y = self.faces_data[0], self.faces_data[1]
        
        # 3D 위치 계산
        camera_pos = self.get_3d_position(center_x, center_y)
        if camera_pos is None:
            return
        
        self.success_count += 1
        
        # EKF 필터링
        if not self.ekf.initialized:
            self.ekf.initialize(camera_pos.tolist())
        else:
            self.ekf.predict()
            self.ekf.update(camera_pos.tolist())
        
        filtered_pos = self.ekf.get_position()
        
        # 로봇 좌표 변환
        robot_raw = self.camera_to_robot(camera_pos)
        robot_filtered = self.camera_to_robot(filtered_pos)
        
        if robot_raw is None or robot_filtered is None:
            return
        
        # 목표 위치 계산 (안전거리 적용)
        depth = abs(filtered_pos[2])
        if depth < 100:
            return
        
        distance = np.linalg.norm(filtered_pos)
        direction = filtered_pos / distance
        target_camera = filtered_pos - direction * SAFETY_DISTANCE
        
        robot_target = self.camera_to_robot(target_camera)
        if robot_target is None:
            return
        
        robot_target[2] += 50.0  # Z 오프셋
        robot_target = self.clamp_to_safety(robot_target)
        
        # 마커 발행 (텍스트 포함 - 원본 코드 복원)
        self.publish_marker(robot_raw, "face_raw", (0.0, 1.0, 0.0), self.marker_pub, "Raw", self.text_pub)
        self.publish_marker(robot_filtered, "face_ekf", (0.0, 0.8, 0.8), self.marker_ekf_pub, "Filtered", self.text_ekf_pub)
        self.publish_marker(robot_target, "face_target", (1.0, 0.0, 0.0), self.marker_robot_pub, "Target", self.text_robot_pub)
        
        # Line 마커 발행 (원본 코드 복원 - RGB 렌즈 오프셋 적용)
        self.publish_line(filtered_pos)
        
        # 디버그 로그
        self.get_logger().info(
            f"📍 Target: [{robot_target[0]:.0f}, {robot_target[1]:.0f}, {robot_target[2]:.0f}]mm",
            throttle_duration_sec=1.0)


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
