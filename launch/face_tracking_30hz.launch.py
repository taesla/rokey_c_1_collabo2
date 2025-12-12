#!/usr/bin/env python3
"""
Face Tracking 30Hz Launch File
- Intel RealSense D435i (30Hz)
- MediaPipe Face Detection (30Hz)
- Face Tracking Node (30Hz)
- Robot Control Node
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

    # Launch Arguments
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='Show OpenCV window for face detection'
    )

    return LaunchDescription([
        show_window_arg,

        # ========================================
        # 1. RealSense D435i Camera (30Hz)
        # ========================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_launch_dir, 'rs_launch.py')
            ),
            launch_arguments={
                # 30Hz 설정
                'depth_module.depth_profile': '640x480x30',
                'rgb_camera.color_profile': '640x480x30',
                
                # 성능 최적화
                'initial_reset': 'true',
                'align_depth.enable': 'true',
                'enable_rgbd': 'true',
                'pointcloud.enable': 'false',  # 포인트클라우드 비활성화 (성능)
                
                # 추가 최적화 옵션
                'enable_sync': 'true',
                'unite_imu_method': '0',  # IMU 비활성화
                'enable_accel': 'false',
                'enable_gyro': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
            }.items()
        ),

        # ========================================
        # 2. Face Detection Node (MediaPipe)
        # ========================================
        Node(
            package='face_tracking_pkg',
            executable='face_detection_node',
            name='face_detection_node',
            output='screen',
            parameters=[{
                'model_selection': 1,  # 1=Long-range (5m), 0=Short-range (2m)
                'min_detection_confidence': 0.5,
                'show_window': LaunchConfiguration('show_window'),
            }],
            # 성능 최적화: 단일 스레드
            prefix='taskset -c 0',
        ),

        # ========================================
        # 3. Face Tracking Node (TF2 Transform)
        # ========================================
        Node(
            package='face_tracking_pkg',
            executable='face_tracking_node',
            name='face_tracking_node',
            output='screen',
            parameters=[{
                'target_offset_mm': 650.0,  # 얼굴에서 떨어진 거리
                'camera_frame': 'camera_color_optical_frame',
                'robot_frame': 'base_link',
            }],
            # 성능 최적화: 단일 스레드
            prefix='taskset -c 1',
        ),

        # ========================================
        # 4. Robot Control Node
        # ========================================
        Node(
            package='face_tracking_pkg',
            executable='robot_control_node',
            name='robot_control_node',
            output='screen',
            parameters=[{
                'velocity': 45,
                'acceleration': 45,
                'j1_gain': 0.12,
                'j5_gain': 0.08,
                'patrol_step': 10.0,
            }],
        ),
    ])
