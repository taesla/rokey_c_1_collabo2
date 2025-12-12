#!/usr/bin/env python3
"""
Face Tracking with YOLO Launch 파일

YOLO 기반 얼굴 검출 + 좌표 변환 + 로봇 제어

사용법:
  ros2 launch face_tracking_pkg face_tracking_yolo.launch.py
  ros2 launch face_tracking_pkg face_tracking_yolo.launch.py use_gpu:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch Arguments
    velocity_arg = DeclareLaunchArgument(
        'velocity', default_value='60',
        description='Robot movement velocity (mm/s)'
    )
    acceleration_arg = DeclareLaunchArgument(
        'acceleration', default_value='120',
        description='Robot movement acceleration (mm/s²)'
    )
    show_window_arg = DeclareLaunchArgument(
        'show_window', default_value='true',
        description='Show OpenCV detection window'
    )
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu', default_value='true',
        description='Use GPU for YOLO inference'
    )
    confidence_arg = DeclareLaunchArgument(
        'confidence', default_value='0.5',
        description='YOLO detection confidence threshold'
    )
    
    # Node 1: Face Detection (YOLO)
    face_detection_node = Node(
        package='face_tracking_pkg',
        executable='face_detection_yolo_node',
        name='face_detection',
        parameters=[{
            'confidence_threshold': LaunchConfiguration('confidence'),
            'use_gpu': LaunchConfiguration('use_gpu'),
            'show_window': LaunchConfiguration('show_window')
        }],
        output='screen'
    )
    
    # Node 2: Face Tracking (좌표 변환)
    face_tracking_node = Node(
        package='face_tracking_pkg',
        executable='face_tracking_node',
        name='face_tracking',
        parameters=[{
            'target_offset_mm': 650.0,
            'camera_frame': 'camera_color_optical_frame',
            'robot_frame': 'base_link'
        }],
        output='screen'
    )
    
    # Node 3: Robot Control
    robot_control_node = Node(
        package='face_tracking_pkg',
        executable='robot_control_node',
        name='robot_control',
        parameters=[{
            'velocity': LaunchConfiguration('velocity'),
            'acceleration': LaunchConfiguration('acceleration'),
            'k_p': 0.2,
            'dead_zone': 50.0,
            'tcp_offset_z': 228.6,
            'use_ekf': False
        }],
        output='screen'
    )
    
    return LaunchDescription([
        velocity_arg,
        acceleration_arg,
        show_window_arg,
        use_gpu_arg,
        confidence_arg,
        face_detection_node,
        face_tracking_node,
        robot_control_node,
    ])
