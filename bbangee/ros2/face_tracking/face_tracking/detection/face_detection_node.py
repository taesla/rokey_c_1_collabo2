#!/usr/bin/env python3
"""
Face Detection Node - ROS2 인터페이스

YoloDetector를 사용한 얼굴 감지 ROS2 노드
감지 결과를 토픽으로 발행

Subscribed Topics:
    /camera/camera/color/image_raw - RealSense 컬러 이미지

Published Topics:
    /face_detection/image - 얼굴 표시된 이미지
    /face_detection/faces - 얼굴 좌표 [center_x, center_y, w, h]
"""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from .yolo_detector import YoloDetector, Detection


class FaceDetectionNode(Node):
    """얼굴 감지 ROS2 노드"""
    
    def __init__(self):
        super().__init__('face_detection_node')
        
        self.bridge = CvBridge()
        self.current_frame = None
        
        # 파라미터 선언
        self._declare_parameters()
        
        # 파라미터 로드
        params = self._load_parameters()
        
        # Detector 초기화 (로거 전달)
        self.detector = YoloDetector(
            model_path=params['model_path'],
            confidence_threshold=params['confidence_threshold'],
            use_gpu=params['use_gpu'],
            use_tensorrt=params['use_tensorrt'],
            use_fp16=params['use_fp16'],
            input_size=params['input_size'],
            use_preprocessing=params['use_preprocessing'],
            use_roi_tracking=params['use_roi_tracking'],
            logger=self.get_logger()
        )
        
        self.show_window = params['show_window']
        
        # FPS 측정
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = self.get_clock().now()
        
        # 신뢰도 히스토리 (안정화)
        self.confidence_history = []
        self.history_size = 5
        
        # 구독자
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # 발행자
        self.image_pub = self.create_publisher(Image, '/face_detection/image', 10)
        self.faces_pub = self.create_publisher(Float32MultiArray, '/face_detection/faces', 10)
        
        # 타이머 (30Hz)
        self.timer = self.create_timer(0.033, self.process_loop)
        
        self._print_startup_info()
    
    def _declare_parameters(self):
        """파라미터 선언"""
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('show_window', False)
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('use_tensorrt', True)
        self.declare_parameter('use_fp16', True)
        self.declare_parameter('input_size', 1280)
        self.declare_parameter('use_preprocessing', True)
        self.declare_parameter('use_roi_tracking', True)
    
    def _load_parameters(self) -> dict:
        """파라미터 로드"""
        return {
            'model_path': self.get_parameter('model_path').value,
            'confidence_threshold': self.get_parameter('confidence_threshold').value,
            'show_window': self.get_parameter('show_window').value,
            'use_gpu': self.get_parameter('use_gpu').value,
            'use_tensorrt': self.get_parameter('use_tensorrt').value,
            'use_fp16': self.get_parameter('use_fp16').value,
            'input_size': self.get_parameter('input_size').value,
            'use_preprocessing': self.get_parameter('use_preprocessing').value,
            'use_roi_tracking': self.get_parameter('use_roi_tracking').value,
        }
    
    def _print_startup_info(self):
        """시작 정보 출력"""
        stats = self.detector.get_stats()
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎥 Face Detection Node 시작!")
        self.get_logger().info(f"  Device: {stats['device'].upper()}")
        self.get_logger().info(f"  TensorRT: {'✅' if stats['tensorrt'] else '❌'}")
        self.get_logger().info(f"  FP16: {'✅' if stats['fp16'] else '❌'}")
        self.get_logger().info(f"  ROI Tracking: {'✅' if stats['roi_tracking'] else '❌'}")
        self.get_logger().info("  Topics:")
        self.get_logger().info("    Sub: /camera/camera/color/image_raw")
        self.get_logger().info("    Pub: /face_detection/faces")
        self.get_logger().info("    Pub: /face_detection/image")
        self.get_logger().info("=" * 60)
    
    def image_callback(self, msg):
        """이미지 수신 콜백"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
    
    def process_loop(self):
        """메인 처리 루프 (30Hz)"""
        if self.current_frame is None:
            return
        
        frame = self.current_frame.copy()
        
        # 감지 수행
        detection = self.detector.detect(frame)
        
        # 결과 처리
        faces_msg = Float32MultiArray()
        num_faces = 0
        
        # ROI 영역 표시 (디버그용 - 보라색)
        roi = self.detector._get_roi(frame.shape)
        if roi is not None:
            cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (255, 0, 255), 1)
        
        if detection:
            num_faces = 1
            cx, cy = detection.center
            
            # 바운딩 박스 그리기
            color = (0, 255, 0) if self.detector.no_detection_count == 0 else (0, 255, 255)
            cv2.rectangle(frame, (detection.x1, detection.y1), 
                         (detection.x2, detection.y2), color, 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            
            # 신뢰도 평활화
            self.confidence_history.append(detection.confidence)
            if len(self.confidence_history) > self.history_size:
                self.confidence_history.pop(0)
            smoothed_conf = sum(self.confidence_history) / len(self.confidence_history)
            
            cv2.putText(frame, f"Conf: {smoothed_conf:.2f}",
                       (detection.x1, detection.y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 메시지 데이터
            faces_msg.data = [float(cx), float(cy), 
                             float(detection.width), float(detection.height)]
        
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
        cv2.putText(frame, f"Inference: {self.detector.avg_inference_time:.1f}ms", (10, 110),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        # 최적화 상태
        stats = self.detector.get_stats()
        opt_list = []
        if stats['tensorrt']:
            opt_list.append("TRT")
        if stats['fp16']:
            opt_list.append("FP16")
        if stats['roi_tracking']:
            opt_list.append("ROI")
        cv2.putText(frame, f"YOLOv8 [{', '.join(opt_list)}]", (10, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # 발행
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"이미지 발행 실패: {e}")
        
        self.faces_pub.publish(faces_msg)
        
        # OpenCV 창
        if self.show_window:
            cv2.imshow("Face Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.get_logger().info("종료합니다.")
                cv2.destroyAllWindows()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionNode()
    
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
