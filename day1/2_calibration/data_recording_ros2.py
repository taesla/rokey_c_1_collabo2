import os
import cv2
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"


class DataRecordingNode(Node):
    def __init__(self):
        super().__init__('data_recording_node')
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        self.current_frame = None
        self.current_pose = None  # 로봇 현재 위치
        
        # RealSense 이미지 토픽 구독
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # 로봇 위치 토픽 구독
        self.pose_sub = self.create_subscription(
            Float64MultiArray,
            f'/{ROBOT_ID}/msg/current_posx',
            self.pose_callback,
            10
        )
        
        # 데이터 저장 경로 설정
        self.source_path = "./data"
        os.makedirs(self.source_path, exist_ok=True)
        
        # 저장 데이터
        self.write_data = {"poses": [], "file_name": []}
        
        # 타이머로 OpenCV 창 업데이트 (30Hz)
        self.timer = self.create_timer(0.033, self.display_loop)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("데이터 레코딩 시작!")
        self.get_logger().info("  [q] 이미지 저장")
        self.get_logger().info("  [ESC] 종료")
        self.get_logger().info("=" * 50)
    
    def pose_callback(self, msg):
        """로봇 위치 토픽 콜백"""
        self.current_pose = list(msg.data)[:6]  # x, y, z, rx, ry, rz
    
    def image_callback(self, msg):
        """ROS2 이미지 메시지를 OpenCV 이미지로 변환"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
    
    def display_loop(self):
        """OpenCV 창 업데이트 및 키 입력 처리"""
        if self.current_frame is None:
            return
        
        # 이미지 표시
        cv2.imshow("RealSense Camera (ROS2)", self.current_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == 27:  # ESC
            self.get_logger().info("종료합니다.")
            cv2.destroyAllWindows()
            rclpy.shutdown()
            
        elif key == ord("q"):
            self.save_data()
    
    def save_data(self):
        """현재 프레임과 로봇 위치 저장"""
        if self.current_frame is None:
            self.get_logger().warn("프레임이 없습니다!")
            return
        
        # 로봇 위치 가져오기 (토픽에서 수신한 값 사용)
        if self.current_pose is not None:
            pos = self.current_pose
            file_name = f"{pos[0]:.2f}_{pos[1]:.2f}_{pos[2]:.2f}.jpg"
            self.get_logger().info(f"로봇 위치: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, {pos[3]:.2f}, {pos[4]:.2f}, {pos[5]:.2f}]")
        else:
            # 로봇 위치를 못 가져오면 타임스탬프 사용
            import time
            timestamp = int(time.time() * 1000)
            file_name = f"img_{timestamp}.jpg"
            pos = [0, 0, 0, 0, 0, 0]
            self.get_logger().warn("로봇 위치 수신 안됨 - 타임스탬프 사용")
        
        # 이미지 저장
        file_path = f"{self.source_path}/{file_name}"
        cv2.imwrite(file_path, self.current_frame)
        
        # 데이터 추가
        self.write_data["file_name"].append(file_name)
        self.write_data["poses"].append(pos)
        
        # JSON 저장
        with open(f"{self.source_path}/calibrate_data.json", "w") as json_file:
            json.dump(self.write_data, json_file, indent=4)
        
        self.get_logger().info(f"✓ 저장 완료 [{len(self.write_data['poses'])}장]: {file_path}")


def main(args=None):
    rclpy.init(args=args)
    
    node = DataRecordingNode()
    
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
