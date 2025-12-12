#!/usr/bin/env python3
"""
Face Tracking 전체 시스템 Launch 파일

3개 노드를 한번에 실행:
1. face_detection_node - 얼굴 감지
2. face_tracking_node - 좌표 변환 (TF2)
3. robot_control_node - 로봇 제어

사용법:
  ros2 launch face_tracking_pkg face_tracking.launch.py
  ros2 launch face_tracking_pkg face_tracking.launch.py velocity:=40
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch Arguments
    velocity_arg = DeclareLaunchArgument(
        'velocity', default_value='35',
        description='Robot movement velocity'
    )
    acceleration_arg = DeclareLaunchArgument(
        'acceleration', default_value='35',
        description='Robot movement acceleration'
    )
    show_window_arg = DeclareLaunchArgument(
        'show_window', default_value='true',
        description='Show OpenCV detection window'
    )
    target_offset_arg = DeclareLaunchArgument(
        'target_offset_mm', default_value='650.0',
        description='Target offset from face (mm)'
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
            'target_offset_mm': LaunchConfiguration('target_offset_mm'),
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
            'robot_id': 'dsr01',
            'robot_model': 'm0609',
            'velocity': LaunchConfiguration('velocity'),
            'acceleration': LaunchConfiguration('acceleration'),
            'j1_gain': 0.12,
            'j5_gain': 0.08,
            'patrol_step': 15.0,
            'detection_timeout': 2.0,
            'max_fail_count': 3
        }],
        output='screen',
        prefix='xterm -e'  # 별도 터미널에서 실행 (키보드 입력용)
    )
    
    return LaunchDescription([
        velocity_arg,
        acceleration_arg,
        show_window_arg,
        target_offset_arg,
        face_detection_node,
        face_tracking_node,
        robot_control_node
    ])
