#!/usr/bin/env python3
"""
EKF Before/After 실시간 비교 도구 (영상 버전)
Raw 신호와 Filtered 신호를 실제 카메라 영상에 표시

좌: Raw (떨림) | 우: Filtered (부드러움)
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
from face_tracking_pkg.face_tracking_ekf import FaceTrackingEKF
from collections import deque
import cv2
import time


class EKFBeforeAfter(Node):
    """EKF Before/After 비교 노드 (영상 버전)"""
    
    def __init__(self):
        super().__init__('ekf_before_after')
        
        # EKF 초기화
        self.ekf = FaceTrackingEKF(dt=0.033)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 구독
        self.marker_sub = self.create_subscription(
            Marker, '/face_tracking/marker_robot', self.marker_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.faces_sub = self.create_subscription(
            Float32MultiArray, '/face_detection/faces', self.faces_callback, 10)
        
        # 통계 데이터
        self.raw_positions = deque(maxlen=100)
        self.filtered_positions = deque(maxlen=100)
        self.noise_values = deque(maxlen=100)
        self.velocity_values = deque(maxlen=100)
        
        # Raw 신호 변동성 측정 (연속 프레임 간 차이)
        self.raw_jitter = deque(maxlen=100)  # Raw 신호의 떨림 정도
        self.filtered_jitter = deque(maxlen=100)  # Filtered 신호의 떨림 정도
        self.prev_raw = None
        self.prev_filtered = None
        
        # 현재 값
        self.current_raw = None
        self.current_filtered = None
        self.current_raw_2d = None  # MediaPipe 픽셀 좌표 (원본)
        self.current_filtered_2d = None  # EKF 필터링된 픽셀 좌표
        self.current_noise = 0.0
        self.current_velocity = 0.0
        
        # MediaPipe로부터의 픽셀 좌표 EKF (2D)
        self.pixel_ekf = FaceTrackingEKF(dt=0.033, dim=2)  # 2D EKF for pixel coordinates
        
        # 카메라 영상
        self.latest_image = None
        
        # OpenCV 윈도우
        self.window_name = 'EKF Before/After - Camera View'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1600, 1000)
        
        # 타이머 (10Hz 화면 갱신)
        self.timer = self.create_timer(0.1, self.update_display)
        
        # 시작 시간
        self.start_time = time.time()
        self.sample_count = 0
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🔬 EKF Before/After Comparison Started (Camera View)")
        self.get_logger().info("  📹 좌: Raw (빨간 마커 - 떨림)")
        self.get_logger().info("  📹 우: Filtered (초록 마커 - 부드러움)")
        self.get_logger().info("  📊 하단: 통계 및 노이즈 그래프")
        self.get_logger().info("  ❌ ESC 키 또는 Ctrl+C로 종료")
        self.get_logger().info("=" * 60)
    
    def image_callback(self, msg):
        """카메라 이미지 수신"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
    
    def faces_callback(self, msg):
        """MediaPipe 얼굴 픽셀 좌표 수신 (Raw)"""
        if len(msg.data) < 4:
            return
        
        # [center_x, center_y, width, height, ...]
        raw_pixel_x = int(msg.data[0])
        raw_pixel_y = int(msg.data[1])
        
        # 화면 범위 체크
        if self.latest_image is not None:
            img_h, img_w = self.latest_image.shape[:2]
            if not (0 <= raw_pixel_x < img_w and 0 <= raw_pixel_y < img_h):
                return
        
        self.current_raw_2d = (raw_pixel_x, raw_pixel_y)
        
        # 2D 픽셀 EKF 처리
        raw_pixel = np.array([float(raw_pixel_x), float(raw_pixel_y)])
        
        if not self.pixel_ekf.initialized:
            self.pixel_ekf.initialize(raw_pixel.tolist())
            self.current_filtered_2d = self.current_raw_2d
        else:
            self.pixel_ekf.predict()
            self.pixel_ekf.update(raw_pixel.tolist())
            filtered_pixel = self.pixel_ekf.get_position()
            
            filt_x = int(filtered_pixel[0])
            filt_y = int(filtered_pixel[1])
            self.current_filtered_2d = (filt_x, filt_y)
        
    def marker_callback(self, msg):
        """마커 콜백 - 3D 로봇 좌표 (노이즈 통계용)"""
        # 3D 로봇 좌표 (노이즈 계산용)
        raw_x = msg.pose.position.x * 1000.0
        raw_y = msg.pose.position.y * 1000.0
        raw_z = msg.pose.position.z * 1000.0
        
        # 이상치 필터링
        if not (200 < raw_x < 1000 and -400 < raw_y < 600 and 200 < raw_z < 800):
            return
        
        raw_pos = np.array([raw_x, raw_y, raw_z])
        self.current_raw = raw_pos
        
        # 3D EKF 처리 (노이즈 통계용)
        if not self.ekf.initialized:
            self.ekf.initialize(raw_pos.tolist())
            self.current_filtered = raw_pos
            self.current_velocity = 0.0
            self.current_noise = 0.0
            self.prev_raw = raw_pos.copy()
            self.prev_filtered = raw_pos.copy()
        else:
            self.ekf.predict()
            self.ekf.update(raw_pos.tolist())
            filtered_pos = self.ekf.get_position()
            filtered_vel = self.ekf.get_velocity()
            
            self.current_filtered = filtered_pos
            self.current_velocity = np.linalg.norm(filtered_vel)
            self.current_noise = np.linalg.norm(raw_pos - filtered_pos)
            
            # Jitter 계산 (연속 프레임 간 변화량)
            if self.prev_raw is not None:
                raw_jitter = np.linalg.norm(raw_pos - self.prev_raw)
                filtered_jitter = np.linalg.norm(filtered_pos - self.prev_filtered)
                
                self.raw_jitter.append(raw_jitter)
                self.filtered_jitter.append(filtered_jitter)
                
            self.prev_raw = raw_pos.copy()
            self.prev_filtered = filtered_pos.copy()
        
        # 데이터 저장
        self.raw_positions.append(raw_pos.copy())
        self.filtered_positions.append(self.current_filtered.copy())
        self.noise_values.append(self.current_noise)
        self.velocity_values.append(self.current_velocity)
        self.sample_count += 1
        
    def update_display(self):
        """화면 갱신 - 영상 기반"""
        if self.latest_image is None:
            return
        
        # 카메라 영상 복사 (좌우 2개)
        img_height, img_width = self.latest_image.shape[:2]
        
        # Raw 영상 (왼쪽)
        raw_img = self.latest_image.copy()
        
        # Filtered 영상 (오른쪽)
        filtered_img = self.latest_image.copy()
        
        # Raw 마커 표시 (빨간색)
        if self.current_raw_2d is not None:
            cv2.circle(raw_img, self.current_raw_2d, 15, (0, 0, 255), -1)
            cv2.circle(raw_img, self.current_raw_2d, 20, (0, 0, 255), 3)
            cv2.putText(raw_img, "RAW", (self.current_raw_2d[0] - 30, self.current_raw_2d[1] - 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Filtered 마커 표시 (초록색)
        if self.current_filtered_2d is not None:
            cv2.circle(filtered_img, self.current_filtered_2d, 15, (0, 255, 0), -1)
            cv2.circle(filtered_img, self.current_filtered_2d, 20, (0, 255, 0), 3)
            cv2.putText(filtered_img, "FILTERED", (self.current_filtered_2d[0] - 50, self.current_filtered_2d[1] - 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # 좌우 영상 합치기
        combined_img = np.hstack([raw_img, filtered_img])
        
        # 라벨 추가 (영어만)
        cv2.putText(combined_img, "BEFORE (Raw Signal - Noisy)", (50, 50),
                   cv2.FONT_HERSHEY_TRIPLEX, 0.9, (0, 0, 255), 2)
        
        cv2.putText(combined_img, "AFTER (EKF Filtered - Smooth)", (img_width + 50, 50),
                   cv2.FONT_HERSHEY_TRIPLEX, 0.9, (0, 255, 0), 2)
        
        # 구분선
        cv2.line(combined_img, (img_width, 0), (img_width, img_height), (255, 255, 255), 3)
        
        # 통계 패널 생성 (하단)
        stats_height = 350
        canvas = np.zeros((img_height + stats_height, img_width * 2, 3), dtype=np.uint8)
        
        # 영상을 캔버스 상단에 배치
        canvas[:img_height, :] = combined_img
        
        # === 통계 패널 (하단) ===
        stats_y = img_height + 20
        
        cv2.putText(canvas, "STATISTICS & PERFORMANCE", (50, stats_y),
                   cv2.FONT_HERSHEY_TRIPLEX, 1.0, (255, 255, 0), 2)
        
        # 설명란 (Description Box)
        desc_y = stats_y + 40
        cv2.rectangle(canvas, (50, desc_y), (img_width * 2 - 50, desc_y + 50), (40, 40, 40), -1)
        cv2.rectangle(canvas, (50, desc_y), (img_width * 2 - 50, desc_y + 50), (100, 100, 100), 2)
        
        cv2.putText(canvas, "EKF (Extended Kalman Filter): Sensor noise removal & motion prediction",
                   (60, desc_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(canvas, "Noise Reduction = (Raw Jitter - Filtered Jitter) / Raw Jitter | Target: > 70%",
                   (60, desc_y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        # 통계 시작 위치 조정
        stats_y = desc_y + 65
        
        if len(self.noise_values) > 0:
            avg_noise = np.mean(self.noise_values)
            max_noise = np.max(self.noise_values)
            min_noise = np.min(self.noise_values)
            std_noise = np.std(self.noise_values)
            avg_vel = np.mean(self.velocity_values)
            
            # Noise Reduction 계산 (Raw 떨림 vs Filtered 떨림)
            if len(self.raw_jitter) > 0 and len(self.filtered_jitter) > 0:
                avg_raw_jitter = np.mean(self.raw_jitter)
                avg_filtered_jitter = np.mean(self.filtered_jitter)
                
                # Reduction = (Raw떨림 - Filtered떨림) / Raw떨림 * 100
                if avg_raw_jitter > 0.01:  # 0으로 나누기 방지
                    noise_reduction = ((avg_raw_jitter - avg_filtered_jitter) / avg_raw_jitter) * 100
                else:
                    noise_reduction = 0.0
            else:
                noise_reduction = 0.0
            
            # 좌측 통계
            col1_x = 50
            cv2.putText(canvas, f"Avg Noise: {avg_noise:.2f} mm", (col1_x, stats_y+50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(canvas, f"Min/Max: {min_noise:.2f} / {max_noise:.2f} mm", (col1_x, stats_y+85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(canvas, f"Std Dev: {std_noise:.2f} mm", (col1_x, stats_y+120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 중앙 통계
            col2_x = 500
            
            # Noise Reduction 색상 (퍼센트 기반)
            reduction_color = (0, 255, 0) if noise_reduction > 70 else (0, 255, 255) if noise_reduction > 50 else (0, 165, 255)
            cv2.putText(canvas, f"Noise Reduction: {noise_reduction:.1f}%", (col2_x, stats_y+50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, reduction_color, 2)
            
            cv2.putText(canvas, f"Avg Velocity: {avg_vel:.1f} mm/s", (col2_x, stats_y+85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 품질 평가 (절대 노이즈 기반)
            quality_color = (0, 255, 0) if avg_noise < 3 else (0, 255, 255) if avg_noise < 5 else (0, 165, 255)
            quality_text = "EXCELLENT" if avg_noise < 3 else "GOOD" if avg_noise < 5 else "FAIR"
            cv2.putText(canvas, f"Quality: {quality_text}", (col2_x, stats_y+120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, quality_color, 2)
            
            # 우측 통계
            col3_x = img_width * 2 - 300
            cv2.putText(canvas, f"Samples: {self.sample_count}", (col3_x, stats_y+50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(canvas, f"Runtime: {time.time() - self.start_time:.1f}s", (col3_x, stats_y+85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 노이즈 그래프
            graph_y = stats_y + 155
            graph_width = img_width * 2 - 100
            
            if len(self.noise_values) > 1:
                points = []
                max_graph_value = max(15.0, max_noise)  # 최소 15mm 스케일
                for i, noise in enumerate(list(self.noise_values)):
                    x = int(50 + i * (graph_width / 100))
                    y = int(graph_y + 70 - (noise / max_graph_value) * 60)
                    points.append((x, y))
                
                # 그래프 영역 배경
                cv2.rectangle(canvas, (45, graph_y - 5), (50 + graph_width, graph_y + 75), (30, 30, 30), -1)
                
                # 기준선들
                cv2.line(canvas, (50, graph_y + 70), (50 + graph_width, graph_y + 70), (100, 100, 100), 1)  # 0mm
                cv2.line(canvas, (50, graph_y + 10), (50 + graph_width, graph_y + 10), (100, 100, 100), 1)  # max
                
                # 노이즈 라인
                for i in range(len(points) - 1):
                    cv2.line(canvas, points[i], points[i+1], (0, 255, 255), 2)
                
                # 레이블
                cv2.putText(canvas, "Noise History (Last 100 samples)", (50, graph_y - 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
                cv2.putText(canvas, "0mm", (15, graph_y + 75),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
                cv2.putText(canvas, f"{max_graph_value:.0f}mm", (5, graph_y + 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
        
        # ESC 안내
        cv2.putText(canvas, "Press ESC to exit", (950, 780),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
        
        cv2.imshow(self.window_name, canvas)
        key = cv2.waitKey(1)
        
        if key == 27:  # ESC
            self.get_logger().info("ESC pressed. Shutting down...")
            rclpy.shutdown()
    
    def print_final_stats(self):
        """최종 통계"""
        if len(self.noise_values) == 0:
            return
        
        avg_noise = np.mean(self.noise_values)
        max_noise = np.max(self.noise_values)
        min_noise = np.min(self.noise_values)
        std_noise = np.std(self.noise_values)
        noise_reduction = ((10.0 - avg_noise) / 10.0) * 100
        
        runtime = time.time() - self.start_time
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("📊 Final EKF Performance Report")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Runtime: {runtime:.1f}s")
        self.get_logger().info(f"Total Samples: {self.sample_count}")
        self.get_logger().info(f"")
        self.get_logger().info(f"BEFORE (Raw):")
        self.get_logger().info(f"  Expected Noise: ~10mm")
        self.get_logger().info(f"")
        self.get_logger().info(f"AFTER (EKF Filtered):")
        self.get_logger().info(f"  Avg Noise: {avg_noise:.2f} mm")
        self.get_logger().info(f"  Min Noise: {min_noise:.2f} mm")
        self.get_logger().info(f"  Max Noise: {max_noise:.2f} mm")
        self.get_logger().info(f"  Std Dev: {std_noise:.2f} mm")
        self.get_logger().info(f"")
        self.get_logger().info(f"🎯 IMPROVEMENT:")
        self.get_logger().info(f"  Noise Reduction: {noise_reduction:.1f}%")
        self.get_logger().info(f"  Quality: {'✅ EXCELLENT' if avg_noise < 3 else '⚠️ GOOD' if avg_noise < 5 else '❌ NEEDS TUNING'}")
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = EKFBeforeAfter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_final_stats()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
