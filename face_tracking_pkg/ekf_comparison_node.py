#!/usr/bin/env python3
"""
EKF Performance Comparison Node
Raw vs Filtered 신호 비교 시각화

Subscribed Topics:
  /face_tracking/marker_robot - 로봇 좌표계 얼굴 위치
  
Published Topics:
  /ekf_comparison/raw - Raw 신호 마커
  /ekf_comparison/filtered - 필터링된 신호 마커
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from face_tracking_pkg.face_tracking_ekf import FaceTrackingEKF
from collections import deque


class EKFComparisonNode(Node):
    """EKF 성능 비교 노드"""
    
    def __init__(self):
        super().__init__('ekf_comparison_node')
        
        # EKF 초기화
        self.ekf = FaceTrackingEKF(dt=0.033)
        
        # 구독
        self.marker_sub = self.create_subscription(
            Marker, '/face_tracking/marker_robot', self.marker_callback, 10)
        
        # 발행
        self.raw_pub = self.create_publisher(Marker, '/ekf_comparison/raw', 10)
        self.filtered_pub = self.create_publisher(Marker, '/ekf_comparison/filtered', 10)
        
        # 통계
        self.noise_history = deque(maxlen=100)
        self.sample_count = 0
        
        # 타이머 (1Hz 통계 출력)
        self.timer = self.create_timer(1.0, self.print_statistics)
        
        self.get_logger().info("🔬 EKF Comparison Node Started")
        self.get_logger().info("  RViz에서 /ekf_comparison/raw, /ekf_comparison/filtered 추가")
        
    def marker_callback(self, msg):
        """마커 콜백"""
        # Raw 위치
        raw_x = msg.pose.position.x * 1000.0
        raw_y = msg.pose.position.y * 1000.0
        raw_z = msg.pose.position.z * 1000.0
        raw_pos = [raw_x, raw_y, raw_z]
        
        # 이상치 필터링
        if not (200 < raw_x < 1000 and -400 < raw_y < 600 and 200 < raw_z < 800):
            return
        
        # EKF 처리
        if not self.ekf.initialized:
            self.ekf.initialize(raw_pos)
            filtered_pos = raw_pos
        else:
            self.ekf.predict()
            self.ekf.update(raw_pos)
            filtered_pos = self.ekf.get_position()
        
        # 노이즈 계산
        noise = np.linalg.norm(np.array(raw_pos) - np.array(filtered_pos))
        self.noise_history.append(noise)
        self.sample_count += 1
        
        # Raw 마커 발행 (빨간색)
        raw_marker = Marker()
        raw_marker.header = msg.header
        raw_marker.ns = "raw"
        raw_marker.id = 0
        raw_marker.type = Marker.SPHERE
        raw_marker.action = Marker.ADD
        raw_marker.pose.position.x = raw_x / 1000.0
        raw_marker.pose.position.y = raw_y / 1000.0
        raw_marker.pose.position.z = raw_z / 1000.0
        raw_marker.scale.x = 0.05
        raw_marker.scale.y = 0.05
        raw_marker.scale.z = 0.05
        raw_marker.color.r = 1.0
        raw_marker.color.g = 0.0
        raw_marker.color.b = 0.0
        raw_marker.color.a = 0.6
        self.raw_pub.publish(raw_marker)
        
        # Filtered 마커 발행 (초록색)
        filtered_marker = Marker()
        filtered_marker.header = msg.header
        filtered_marker.ns = "filtered"
        filtered_marker.id = 1
        filtered_marker.type = Marker.SPHERE
        filtered_marker.action = Marker.ADD
        filtered_marker.pose.position.x = filtered_pos[0] / 1000.0
        filtered_marker.pose.position.y = filtered_pos[1] / 1000.0
        filtered_marker.pose.position.z = filtered_pos[2] / 1000.0
        filtered_marker.scale.x = 0.06
        filtered_marker.scale.y = 0.06
        filtered_marker.scale.z = 0.06
        filtered_marker.color.r = 0.0
        filtered_marker.color.g = 1.0
        filtered_marker.color.b = 0.0
        filtered_marker.color.a = 0.8
        self.filtered_pub.publish(filtered_marker)
        
        # 궤적 라인 (옵션)
        if not hasattr(self, 'raw_trail'):
            self.raw_trail = deque(maxlen=30)
            self.filtered_trail = deque(maxlen=30)
        
        self.raw_trail.append(Point(x=raw_x/1000.0, y=raw_y/1000.0, z=raw_z/1000.0))
        self.filtered_trail.append(Point(x=filtered_pos[0]/1000.0, 
                                        y=filtered_pos[1]/1000.0, 
                                        z=filtered_pos[2]/1000.0))
        
    def print_statistics(self):
        """통계 출력"""
        if len(self.noise_history) == 0:
            return
        
        avg_noise = np.mean(self.noise_history)
        max_noise = np.max(self.noise_history)
        std_noise = np.std(self.noise_history)
        
        # 노이즈 감소율 계산
        noise_reduction = (avg_noise / max_noise) * 100 if max_noise > 0 else 0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"📊 EKF Performance (Samples: {self.sample_count})")
        self.get_logger().info(f"  평균 노이즈: {avg_noise:.2f} mm")
        self.get_logger().info(f"  최대 노이즈: {max_noise:.2f} mm")
        self.get_logger().info(f"  표준편차: {std_noise:.2f} mm")
        self.get_logger().info(f"  노이즈 감소: {(1-avg_noise/10.0)*100:.1f}% (vs Raw ~10mm)")
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = EKFComparisonNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
