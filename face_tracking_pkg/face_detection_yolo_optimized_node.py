#!/usr/bin/env python3
"""
Face Detection Node - YOLOv8 Optimized Version

고급 최적화 적용:
1. TensorRT 엔진 자동 변환 (NVIDIA GPU)
2. FP16 Half Precision 추론
3. 고해상도 입력 (1280px)
4. 히스토그램 평활화 전처리
5. NMS 최적화
6. 멀티스케일 검출
7. ROI 기반 빠른 추적

Subscribed Topics:
  /camera/camera/color/image_raw - RealSense 컬러 이미지

Published Topics:
  /face_detection/image - 얼굴 표시된 이미지
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
import time


class FaceDetectionYoloOptimizedNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        
        self.bridge = CvBridge()
        self.current_frame = None
        
        # 파라미터 선언
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.4)  # 낮춰서 민감하게
        self.declare_parameter('show_window', False)
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('use_tensorrt', True)  # TensorRT 사용
        self.declare_parameter('use_fp16', True)  # FP16 추론
        self.declare_parameter('input_size', 1280)  # 고해상도 입력
        self.declare_parameter('use_preprocessing', True)  # 전처리 적용
        self.declare_parameter('use_roi_tracking', True)  # ROI 기반 추적
        
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.show_window = self.get_parameter('show_window').value
        use_gpu = self.get_parameter('use_gpu').value
        self.use_tensorrt = self.get_parameter('use_tensorrt').value
        self.use_fp16 = self.get_parameter('use_fp16').value
        self.input_size = self.get_parameter('input_size').value
        self.use_preprocessing = self.get_parameter('use_preprocessing').value
        self.use_roi_tracking = self.get_parameter('use_roi_tracking').value
        
        # 모델 경로 찾기
        if not model_path or not os.path.exists(model_path):
            package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = os.path.join(package_dir, 'models', 'yolov8n-face.pt')
            
            if not os.path.exists(model_path):
                model_path = '/home/rokey/ros2_ws/src/face_tracking_pkg/models/yolov8n-face.pt'
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"❌ 모델 파일을 찾을 수 없습니다: {model_path}")
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        # GPU 확인
        self.device = 'cpu'
        self.cuda_available = False
        
        if use_gpu:
            try:
                import torch
                if torch.cuda.is_available():
                    self.device = 'cuda'
                    self.cuda_available = True
                    gpu_name = torch.cuda.get_device_name(0)
                    self.get_logger().info(f"🚀 GPU 감지: {gpu_name}")
                else:
                    self.get_logger().warn("⚠️ GPU 사용 불가, CPU 모드로 실행")
                    self.use_tensorrt = False
                    self.use_fp16 = False
            except Exception as e:
                self.get_logger().warn(f"⚠️ PyTorch CUDA 확인 실패: {e}")
                self.use_tensorrt = False
                self.use_fp16 = False
        
        # TensorRT 엔진 로드 또는 생성
        self.model = self._load_optimized_model(model_path)
        
        # 성능 측정용
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = self.get_clock().now()
        self.inference_times = []
        self.avg_inference_time = 0.0
        
        # 신뢰도 히스토리 (안정성 향상)
        self.confidence_history = []
        self.history_size = 5
        
        # 이전 검출 결과 (깜빡임 방지)
        self.last_detection = None
        self.no_detection_count = 0
        self.max_no_detection = 5  # 5프레임까지 이전 결과 유지
        
        # ROI 기반 추적 (빠른 검출)
        self.last_roi = None
        self.roi_margin = 100  # ROI 마진 (픽셀)
        self.roi_fail_count = 0
        self.max_roi_fail = 3
        
        # CLAHE (적응형 히스토그램 평활화)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        
        # RealSense 이미지 토픽 구독
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # 퍼블리셔
        self.image_pub = self.create_publisher(Image, '/face_detection/image', 10)
        self.faces_pub = self.create_publisher(Float32MultiArray, '/face_detection/faces', 10)
        
        # 타이머 (30Hz)
        self.timer = self.create_timer(0.033, self.process_loop)
        
        self._print_startup_info()
    
    def _load_optimized_model(self, model_path):
        """최적화된 모델 로드 (TensorRT 우선)"""
        
        # TensorRT 엔진 경로
        engine_path = model_path.replace('.pt', '.engine')
        
        if self.use_tensorrt and self.cuda_available:
            if os.path.exists(engine_path):
                # 기존 TensorRT 엔진 사용
                self.get_logger().info(f"🔥 TensorRT 엔진 로딩: {engine_path}")
                try:
                    model = YOLO(engine_path)
                    self.get_logger().info("✅ TensorRT 엔진 로드 성공!")
                    return model
                except Exception as e:
                    self.get_logger().warn(f"⚠️ TensorRT 엔진 로드 실패: {e}")
            
            # TensorRT 엔진 생성 시도
            self.get_logger().info(f"🔄 YOLO 모델 로딩 및 TensorRT 변환 시도...")
            try:
                model = YOLO(model_path)
                self.get_logger().info("🔧 TensorRT 엔진 생성 중... (최초 1회, 1-2분 소요)")
                
                # TensorRT export
                model.export(
                    format='engine',
                    half=self.use_fp16,
                    imgsz=self.input_size,
                    device=0
                )
                
                if os.path.exists(engine_path):
                    self.get_logger().info("✅ TensorRT 엔진 생성 완료!")
                    model = YOLO(engine_path)
                    return model
                    
            except Exception as e:
                self.get_logger().warn(f"⚠️ TensorRT 변환 실패: {e}")
                self.get_logger().info("📌 일반 YOLO 모델로 대체합니다.")
        
        # 일반 YOLO 모델 로드
        self.get_logger().info(f"🔄 YOLO 모델 로딩: {model_path}")
        model = YOLO(model_path)
        
        return model
    
    def _print_startup_info(self):
        """시작 정보 출력"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎥 Face Detection Node (YOLOv8 Optimized) 시작!")
        self.get_logger().info(f"  Device: {self.device.upper()}")
        self.get_logger().info(f"  TensorRT: {'✅ 활성화' if self.use_tensorrt and self.cuda_available else '❌ 비활성화'}")
        self.get_logger().info(f"  FP16: {'✅ 활성화' if self.use_fp16 and self.cuda_available else '❌ 비활성화'}")
        self.get_logger().info(f"  Input Size: {self.input_size}px")
        self.get_logger().info(f"  Preprocessing: {'✅' if self.use_preprocessing else '❌'}")
        self.get_logger().info(f"  ROI Tracking: {'✅' if self.use_roi_tracking else '❌'}")
        self.get_logger().info(f"  Confidence: {self.confidence_threshold}")
        self.get_logger().info("  Published Topics:")
        self.get_logger().info("    /face_detection/image")
        self.get_logger().info("    /face_detection/faces")
        self.get_logger().info("=" * 60)
    
    def image_callback(self, msg):
        """ROS2 이미지 메시지를 OpenCV 이미지로 변환"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
    
    def _preprocess_image(self, frame):
        """이미지 전처리 (조명 보정)"""
        if not self.use_preprocessing:
            return frame
        
        # LAB 색공간으로 변환
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        
        # L 채널에 CLAHE 적용 (적응형 히스토그램 평활화)
        lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])
        
        # BGR로 다시 변환
        enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        
        return enhanced
    
    def _get_roi(self, frame_shape):
        """ROI 영역 계산 (이전 검출 기반)"""
        if not self.use_roi_tracking or self.last_detection is None:
            return None
        
        h, w = frame_shape[:2]
        det = self.last_detection
        
        # ROI 계산 (마진 포함)
        x1 = max(0, det['x1'] - self.roi_margin)
        y1 = max(0, det['y1'] - self.roi_margin)
        x2 = min(w, det['x2'] + self.roi_margin)
        y2 = min(h, det['y2'] + self.roi_margin)
        
        # 최소 크기 보장
        if (x2 - x1) < 100 or (y2 - y1) < 100:
            return None
        
        return (x1, y1, x2, y2)
    
    def _detect_in_roi(self, frame, roi):
        """ROI 영역에서 검출"""
        x1, y1, x2, y2 = roi
        roi_frame = frame[y1:y2, x1:x2]
        
        # ROI에서 추론
        results = self.model.predict(
            roi_frame,
            conf=self.confidence_threshold,
            device=self.device,
            half=self.use_fp16 and self.cuda_available,
            verbose=False
        )
        
        # 결과와 ROI 오프셋을 함께 반환 (inplace 수정 대신)
        return results, (x1, y1)
    
    def _detect_full_frame(self, frame):
        """전체 프레임에서 검출"""
        results = self.model.predict(
            frame,
            conf=self.confidence_threshold,
            device=self.device,
            imgsz=self.input_size,
            half=self.use_fp16 and self.cuda_available,
            verbose=False
        )
        return results
    
    def process_loop(self):
        """얼굴 감지 및 퍼블리시"""
        if self.current_frame is None:
            return
        
        frame = self.current_frame.copy()
        h, w, _ = frame.shape
        
        # 전처리
        processed_frame = self._preprocess_image(frame)
        
        # 추론 시간 측정 시작
        start_time = time.time()
        
        # ROI 기반 검출 시도
        results = None
        roi_offset = (0, 0)  # ROI 오프셋 (x, y)
        roi = self._get_roi(frame.shape)
        
        if roi is not None:
            results, roi_offset = self._detect_in_roi(processed_frame, roi)
            
            # ROI에서 검출 실패 시 전체 프레임 검출
            if not results or len(results) == 0 or results[0].boxes is None or len(results[0].boxes) == 0:
                self.roi_fail_count += 1
                if self.roi_fail_count >= self.max_roi_fail:
                    results = self._detect_full_frame(processed_frame)
                    roi_offset = (0, 0)
                    self.roi_fail_count = 0
            else:
                self.roi_fail_count = 0
        else:
            # ROI 없으면 전체 프레임 검출
            results = self._detect_full_frame(processed_frame)
            roi_offset = (0, 0)
        
        # 추론 시간 측정 종료
        inference_time = (time.time() - start_time) * 1000  # ms
        self.inference_times.append(inference_time)
        if len(self.inference_times) > 30:
            self.inference_times.pop(0)
        self.avg_inference_time = sum(self.inference_times) / len(self.inference_times)
        
        # 결과 처리
        faces_msg = Float32MultiArray()
        faces_data = []
        num_faces = 0
        best_detection = None
        max_area = 0
        best_conf = 0
        
        # ROI 오프셋 적용
        offset_x, offset_y = roi_offset
        
        if results and len(results) > 0:
            boxes = results[0].boxes
            
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    # clone()으로 복사 후 오프셋 적용
                    coords = box.xyxy[0].cpu().numpy().copy()
                    x1 = coords[0] + offset_x
                    y1 = coords[1] + offset_y
                    x2 = coords[2] + offset_x
                    y2 = coords[3] + offset_y
                    conf = float(box.conf[0].cpu().numpy())
                    area = (x2 - x1) * (y2 - y1)
                    
                    if area > max_area:
                        max_area = area
                        best_conf = conf
                        best_detection = {
                            'x1': int(x1), 'y1': int(y1),
                            'x2': int(x2), 'y2': int(y2),
                            'conf': conf
                        }
        
        # 깜빡임 방지
        if best_detection is None:
            self.no_detection_count += 1
            if self.last_detection is not None and self.no_detection_count <= self.max_no_detection:
                best_detection = self.last_detection.copy()
                best_detection['conf'] = self.last_detection['conf'] * (0.9 ** self.no_detection_count)
        else:
            self.no_detection_count = 0
            self.last_detection = best_detection
            self.last_roi = (best_detection['x1'], best_detection['y1'],
                            best_detection['x2'], best_detection['y2'])
        
        # 검출 결과 처리
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
            color = (0, 255, 0) if self.no_detection_count == 0 else (0, 255, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # ROI 영역 표시 (디버그용)
            if roi is not None:
                cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (255, 0, 255), 1)
            
            # 신뢰도 표시
            raw_confidence = best_detection['conf']
            self.confidence_history.append(raw_confidence)
            if len(self.confidence_history) > self.history_size:
                self.confidence_history.pop(0)
            
            smoothed_confidence = sum(self.confidence_history) / len(self.confidence_history)
            
            cv2.putText(frame, f"Conf: {smoothed_confidence:.2f}", 
                       (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 좌표 데이터
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
        cv2.putText(frame, f"Inference: {self.avg_inference_time:.1f}ms", (10, 110), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        # 최적화 상태 표시
        opt_status = []
        if self.use_tensorrt and self.cuda_available:
            opt_status.append("TRT")
        if self.use_fp16 and self.cuda_available:
            opt_status.append("FP16")
        if self.use_roi_tracking:
            opt_status.append("ROI")
        
        opt_text = f"YOLOv8 Optimized [{', '.join(opt_status)}]"
        cv2.putText(frame, opt_text, (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # 이미지 퍼블리시
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"이미지 발행 실패: {e}")
        
        # 좌표 퍼블리시
        faces_msg.data = faces_data
        self.faces_pub.publish(faces_msg)
        
        # OpenCV 창 표시
        if self.show_window:
            cv2.imshow("Face Detection (YOLOv8 Optimized)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.get_logger().info("종료합니다.")
                cv2.destroyAllWindows()
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionYoloOptimizedNode()
    
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
