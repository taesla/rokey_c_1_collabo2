"""
OBB 기반 Object Detection Node
- OBB 모델로 회전 각도 추출
- 각도 정보를 서비스 응답에 포함
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition
from pick_and_place_text.realsense import ImgNode
from pick_and_place_text.yolo_obb import YoloModel


PACKAGE_NAME = 'pick_and_place_text'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class ObjectDetectionNode(Node):
    def __init__(self, model_name='yolo_obb'):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = YoloModel()  # OBB 모델 사용
        self.bridge = CvBridge()
        
        # 현재 탐지된 각도 저장 (robot_move에서 사용)
        self.current_angle = 0.0
        
        # Detection 결과 이미지 퍼블리셔 (RViz에서 볼 수 있음)
        self.detection_pub = self.create_publisher(Image, '/detection/image', 10)
        
        # 각도 퍼블리셔
        self.angle_pub = self.create_publisher(Float32, '/detection/angle', 10)
        
        # 실시간 Detection 타이머 (0.1초마다)
        self.create_timer(0.1, self.publish_detection_image)
        
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )
        self.get_logger().info("ObjectDetectionNode (OBB) initialized.")
        self.get_logger().info("Detection image topic: /detection/image")
        self.get_logger().info("Angle topic: /detection/angle")

    def publish_detection_image(self):
        """실시간으로 OBB Detection 결과를 퍼블리시합니다."""
        rclpy.spin_once(self.img_node, timeout_sec=0.01)
        frame = self.img_node.get_color_frame()
        if frame is None:
            return
        
        # YOLO OBB 추론
        results = self.model.model(frame, verbose=False, conf=0.5)
        
        # OBB 결과 그리기
        annotated_frame = frame.copy()
        if results[0].obb is not None and len(results[0].obb) > 0:
            for i in range(len(results[0].obb)):
                obb_points = results[0].obb.xyxyxyxy[i].cpu().numpy()
                conf = float(results[0].obb.conf[i])
                cls = int(results[0].obb.cls[i])
                
                # 각도 계산
                angle = self.model.calculate_angle(obb_points)
                
                # OBB 그리기
                pts = np.array(obb_points, dtype=np.int32)
                cv2.polylines(annotated_frame, [pts], True, (0, 255, 0), 2)
                
                # 중심점
                cx = int(np.mean([p[0] for p in obb_points]))
                cy = int(np.mean([p[1] for p in obb_points]))
                
                # 클래스명 찾기
                class_names = {0: "hammer", 1: "pliers", 2: "screw", 3: "wrench"}
                class_name = class_names.get(cls, "unknown")
                
                cv2.putText(annotated_frame, f"{class_name} {conf:.2f} | {angle:.1f}deg", 
                           (cx - 60, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 255), -1)
        
        # ROS 메시지로 변환 및 퍼블리시
        msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.detection_pub.publish(msg)

    def handle_get_depth(self, request, response):
        """클라이언트 요청을 처리해 3D 좌표를 반환합니다."""
        self.get_logger().info(f"Received request: {request}")
        coords, angle = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        
        # 각도 정보 퍼블리시
        angle_msg = Float32()
        angle_msg.data = float(angle) if angle is not None else 0.0
        self.angle_pub.publish(angle_msg)
        self.current_angle = angle if angle is not None else 0.0
        
        self.get_logger().info(f"Response: coords={coords}, angle={angle:.1f}deg")
        return response

    def _compute_position(self, target):
        """OBB로 객체의 카메라 좌표와 각도를 계산합니다."""
        rclpy.spin_once(self.img_node)

        box, score, angle, annotated_frame = self.model.get_best_detection_with_viz(self.img_node, target)
        
        if box is None or score is None:
            self.get_logger().warn("No detection found.")
            return (0.0, 0.0, 0.0), 0.0
        
        self.get_logger().info(f"Detection: box={box}, score={score}, angle={angle:.1f}deg")
        
        # OBB 중심점 사용
        cx, cy = int(box[0]), int(box[1])
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return (0.0, 0.0, 0.0), angle

        coords = self._pixel_to_camera_coords(cx, cy, cz)
        return coords, angle

    def _get_depth(self, x, y):
        """픽셀 좌표의 depth 값을 안전하게 읽어옵니다."""
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도합니다."""
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        """픽셀 좌표와 intrinsics를 이용해 카메라 좌표계로 변환합니다."""
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
