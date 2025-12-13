"""
Camera to Robot TF Publisher
- link_6 (gripper) → camera_link static TF 발행
- T_gripper2camera.npy 캘리브레이션 값 사용
"""
import os
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory


class CameraTFPublisher(Node):
    def __init__(self):
        super().__init__('camera_tf_publisher')
        
        # Static TF broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 캘리브레이션 파일 로드
        package_path = get_package_share_directory("pick_and_place_text")
        calib_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
        
        if os.path.exists(calib_path):
            T = np.load(calib_path)
            self.get_logger().info(f"Loaded calibration from: {calib_path}")
        else:
            self.get_logger().warn("Calibration file not found, using default values")
            T = np.eye(4)
        
        # Translation (mm → m)
        tx = T[0, 3] / 1000.0
        ty = T[1, 3] / 1000.0
        tz = T[2, 3] / 1000.0
        
        # Rotation (matrix → quaternion)
        r = Rotation.from_matrix(T[:3, :3])
        quat = r.as_quat()  # [x, y, z, w]
        
        self.get_logger().info(f"Translation (m): [{tx:.4f}, {ty:.4f}, {tz:.4f}]")
        self.get_logger().info(f"Quaternion: [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]")
        
        # Static TF 발행: link_6 → camera_link
        self.publish_static_tf(
            parent_frame="link_6",
            child_frame="camera_link",
            translation=(tx, ty, tz),
            rotation=quat
        )
        
        self.get_logger().info("Static TF published: link_6 → camera_link")

    def publish_static_tf(self, parent_frame, child_frame, translation, rotation):
        """Static TF 발행"""
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CameraTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
