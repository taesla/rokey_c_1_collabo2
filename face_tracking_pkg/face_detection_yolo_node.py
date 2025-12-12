#!/usr/bin/env python3
"""
Face Detection Node - YOLOv8 Face Detection

YOLOv8n-face 모델을 사용하여 얼굴 감지 후 좌표 퍼블리시
- GPU 가속 지원
- 100+ FPS 성능 (GPU)
- 높은 안정성 (깜빡임 없음)

Subscribed Topics:
  /camera/camera/color/image_raw - RealSense 컬러 이미지

Published Topics:
  /face_detection/image - 얼굴 표시된 이미지 (RViz 확인용)
  /face_detection/faces - 얼굴 좌표 리스트 [center_x, center_y, w, h, ...]
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import os


class FaceDetectionYoloNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        
        self.bridge = CvBridge()
        self.current_frame = None
        
        # 파라미터 선언
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('show_window', False)
        self.declare_parameter('use_gpu', True)
        
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.show_window = self.get_parameter('show_window').value
        use_gpu = self.get_parameter('use_gpu').value
        
        # 모델 경로 찾기
        if not model_path or not os.path.exists(model_path):
            # 기본 경로에서 찾기
            package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = os.path.join(package_dir, 'models', 'yolov8n-face.pt')
            
            if not os.path.exists(model_path):
                # src 폴더에서 찾기
                model_path = '/home/rokey/ros2_ws/src/face_tracking_pkg/models/yolov8n-face.pt'
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"❌ 모델 파일을 찾을 수 없습니다: {model_path}")
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        # YOLO 모델 로드
        self.get_logger().info(f"🔄 YOLO 모델 로딩: {model_path}")
        self.model = YOLO(model_path)
        
        # GPU 사용 설정
        if use_gpu:
            try:
                import torch
                if torch.cuda.is_available():
                    self.device = 'cuda'
                    self.get_logger().info(f"🚀 GPU 사용: {torch.cuda.get_device_name(0)}")
                else:
                    self.device = 'cpu'
                    self.get_logger().warn("⚠️ GPU 사용 불가, CPU 모드로 실행")
            except:
                self.device = 'cpu'
                self.get_logger().warn("⚠️ PyTorch CUDA 확인 실패, CPU 모드")
        else:
            self.device = 'cpu'
            self.get_logger().info("💻 CPU 모드로 실행")
        
        # 성능 측정용
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = self.get_clock().now()
        
        # 신뢰도 히스토리 (안정성 향상)
        self.confidence_history = []
        self.history_size = 5
        
        # 이전 검출 결과 (깜빡임 방지)
        self.last_detection = None
        self.no_detection_count = 0
        self.max_no_detection = 3  # 3프레임까지 이전 결과 유지
        
        # RealSense 이미지 토픽 구독
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # 퍼블리셔: 얼굴 표시된 이미지
        self.image_pub = self.create_publisher(
            Image,
            '/face_detection/image',
            10
        )
        
        # 퍼블리셔: 얼굴 좌표
        self.faces_pub = self.create_publisher(
            Float32MultiArray,
            '/face_detection/faces',
            10
        )
        
        # 타이머로 처리 (30Hz)
        self.timer = self.create_timer(0.033, self.process_loop)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("🎥 Face Detection Node (YOLOv8) 시작!")
        self.get_logger().info(f"  Model: yolov8n-face")
        self.get_logger().info(f"  Device: {self.device}")
        self.get_logger().info(f"  Confidence: {self.confidence_threshold}")
        self.get_logger().info("  Published Topics:")
        self.get_logger().info("    /face_detection/image")
        self.get_logger().info("    /face_detection/faces")
        self.get_logger().info("=" * 50)
    
    def image_callback(self, msg):
        """ROS2 이미지 메시지를 OpenCV 이미지로 변환"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
    
    def process_loop(self):
        """얼굴 감지 및 퍼블리시 (YOLO)"""
        if self.current_frame is None:
            return
        
        frame = self.current_frame.copy()
        h, w, _ = frame.shape
        
        # YOLO 추론
        results = self.model.predict(
            frame, 
            conf=self.confidence_threshold,
            device=self.device,
            verbose=False
        )
        
        # 얼굴 좌표 메시지 생성
        faces_msg = Float32MultiArray()
        faces_data = []
        
        num_faces = 0
        best_detection = None
        max_area = 0
        best_conf = 0
        
        # 결과 처리
        if results and len(results) > 0:
            boxes = results[0].boxes
            
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    # Bounding box 좌표 (xyxy 형식)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0].cpu().numpy())
                    
                    area = (x2 - x1) * (y2 - y1)
                    
                    # 가장 큰 (가장 가까운) 얼굴 선택
                    if area > max_area:
                        max_area = area
                        best_conf = conf
                        best_detection = {
                            'x1': int(x1), 'y1': int(y1),
                            'x2': int(x2), 'y2': int(y2),
                            'conf': conf
                        }
        
        # 깜빡임 방지: 검출 실패 시 이전 결과 유지
        if best_detection is None:
            self.no_detection_count += 1
            if self.last_detection is not None and self.no_detection_count <= self.max_no_detection:
                best_detection = self.last_detection
                best_conf = best_detection['conf'] * 0.9  # 신뢰도 감소
        else:
            self.no_detection_count = 0
            self.last_detection = best_detection
        
        # 가장 가까운 얼굴 처리
        if best_detection:
            num_faces = 1
            x1 = best_detection['x1']
            y1 = best_detection['y1']
            x2 = best_detection['x2']
            y2 = best_detection['y2']
            
            width = x2 - x1
            height = y2 - y1
            center_x = x1 + width // 2
            center_y = y1 + height // 2
            
            # Bounding box 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 중심점 표시
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # 신뢰도 시간적 필터링
            raw_confidence = best_conf
            self.confidence_history.append(raw_confidence)
            if len(self.confidence_history) > self.history_size:
                self.confidence_history.pop(0)
            
            smoothed_confidence = sum(self.confidence_history) / len(self.confidence_history)
            
            # 화면 표시
            cv2.putText(frame, f"Raw: {raw_confidence:.2f}", 
                       (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(frame, f"Smooth: {smoothed_confidence:.2f}", 
                       (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 좌표 데이터 추가 [center_x, center_y, width, height]
            faces_data.extend([float(center_x), float(center_y), 
                             float(width), float(height)])
        
        # FPS 계산
        self.frame_count += 1
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_fps_time).nanoseconds / 1e9
        
        if time_diff >= 1.0:
            self.fps = self.frame_count / time_diff
            self.frame_count = 0
            self.last_fps_time = current_time
        
        # 정보 표시
        cv2.putText(frame, f"Faces: {num_faces}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(frame, f"YOLOv8 ({self.device.upper()})", (10, 110), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # 이미지 퍼블리시
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"이미지 발행 실패: {e}")
        
        # 좌표 퍼블리시
        faces_msg.data = faces_data
        self.faces_pub.publish(faces_msg)
        
        # OpenCV 창 표시 (옵션)
        if self.show_window:
            cv2.imshow("Face Detection (YOLOv8)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.get_logger().info("종료합니다.")
                cv2.destroyAllWindows()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionYoloNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
