#!/usr/bin/env python3
"""
Joint-Space Face Tracking Launch File

조인트 공간 제어 방식 얼굴 추적 시스템

구성:
  1. face_detection_yolo_optimized_node - TensorRT YOLO 얼굴 감지
  2. face_tracking_node - 3D 좌표 변환 + EKF
  3. joint_tracking_node - 조인트 직접 제어 (NEW!)
  
실행:
  ros2 launch face_tracking_pkg joint_tracking.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    """런치 설정 생성"""
    
    # 1. 얼굴 감지 노드 (TensorRT YOLO)
    face_detection_node = Node(
        package='face_tracking_pkg',
        executable='face_detection_yolo_optimized_node',
        name='face_detection_yolo_optimized_node',
        output='screen',
        parameters=[{
            'model_path': '/home/rokey/ros2_ws/src/face_tracking_pkg/models/yolov8n-face.pt',
            'engine_path': '/home/rokey/ros2_ws/src/face_tracking_pkg/models/yolov8n-face.engine',
            'confidence_threshold': 0.5,
            'use_tensorrt': True,
            'target_fps': 30.0,
        }]
    )
    
    # 2. 3D 좌표 변환 + EKF 노드
    face_tracking_node = Node(
        package='face_tracking_pkg',
        executable='face_tracking_node',
        name='face_tracking_node',
        output='screen',
        parameters=[{
            'use_ekf': True,
            'ekf_dt': 0.033,  # 30Hz
            'tracking_loop_rate': 100.0,  # 100Hz
        }]
    )
    
    # 3. 조인트 추적 노드 (1초 후 시작)
    joint_tracking_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='face_tracking_pkg',
                executable='joint_tracking_node',
                name='joint_tracking_node',
                output='screen',
                parameters=[{
                    'robot_id': 'dsr01',
                    'robot_model': 'm0609',
                    # 조인트 속도 제한 (°/s)
                    'j1_vel_limit': 100.0,  # 베이스 회전
                    'j4_vel_limit': 150.0,  # 손목 피치
                    'j6_vel_limit': 150.0,  # 손목 요
                    # 제어 게인
                    'j1_gain': 0.8,
                    'j4_gain': 0.6,
                    'j6_gain': 0.3,
                    # 떨림 방지
                    'dead_zone_deg': 2.0,
                    # TCP 오프셋
                    'tcp_offset_z': 228.6,
                }]
            )
        ]
    )
    
    return LaunchDescription([
        face_detection_node,
        face_tracking_node,
        joint_tracking_node,
    ])
