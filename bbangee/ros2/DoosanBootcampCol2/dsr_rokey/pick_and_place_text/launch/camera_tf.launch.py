#!/usr/bin/env python3
"""
Camera TF Launch File
- Intel RealSense D435i 카메라 실행
- link_6 -> camera_link TF 발행 (Hand-Eye Calibration 적용)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # RealSense 패키지 경로
    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch'
    )

    # Hand-Eye Calibration 결과 (T_gripper2camera 역변환 적용)
    # Translation (m)
    tf_x = 0.034320
    tf_y = 0.062701
    tf_z = 0.198739
    # Quaternion (x, y, z, w)
    tf_qx = 0.508241
    tf_qy = 0.499358
    tf_qz = 0.498430
    tf_qw = -0.493863

    return LaunchDescription([
        # RealSense D435i 카메라 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_launch_dir, 'rs_launch.py')
            ),
            launch_arguments={
                'depth_module.depth_profile': '640x480x30',
                'rgb_camera.color_profile': '640x480x30',
                'initial_reset': 'true',
                'align_depth.enable': 'true',
                'enable_rgbd': 'true',
                'pointcloud.enable': 'true',
            }.items()
        ),

        # link_6 -> camera_link TF (Hand-Eye Calibration)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            arguments=[
                '--x', str(tf_x),
                '--y', str(tf_y),
                '--z', str(tf_z),
                '--qx', str(tf_qx),
                '--qy', str(tf_qy),
                '--qz', str(tf_qz),
                '--qw', str(tf_qw),
                '--frame-id', 'link_6',
                '--child-frame-id', 'camera_link',
            ],
            output='screen'
        ),
    ])
