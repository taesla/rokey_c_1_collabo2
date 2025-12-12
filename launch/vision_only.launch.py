#!/usr/bin/env python3
"""
Face Detection + Tracking Launch (로봇 제어 없이)

비전 노드만 실행 (로봇 없이 테스트용):
1. face_detection_node - 얼굴 감지
2. face_tracking_node - 좌표 변환 (TF2)

사용법:
  ros2 launch face_tracking_pkg vision_only.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch Arguments
    show_window_arg = DeclareLaunchArgument(
        'show_window', default_value='true',
        description='Show OpenCV detection window'
    )
    
    # Node 1: Face Detection
    face_detection_node = Node(
        package='face_tracking_pkg',
        executable='face_detection_node',
        name='face_detection',
        parameters=[{
            'scale_factor': 1.3,
            'min_neighbors': 1,
            'min_size': 30,
            'show_window': LaunchConfiguration('show_window')
        }],
        output='screen'
    )
    
    # Node 2: Face Tracking (TF2 좌표 변환)
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
    
    return LaunchDescription([
        show_window_arg,
        face_detection_node,
        face_tracking_node
    ])
