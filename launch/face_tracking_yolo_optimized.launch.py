"""
Face Tracking Launch File - YOLO Optimized Version

고급 최적화 버전:
- TensorRT 엔진 자동 변환
- FP16 Half Precision
- 고해상도 입력 (1280px)
- 히스토그램 평활화 전처리
- ROI 기반 빠른 추적

사용법:
  ros2 launch face_tracking_pkg face_tracking_yolo_optimized.launch.py
  ros2 launch face_tracking_pkg face_tracking_yolo_optimized.launch.py use_tensorrt:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu', default_value='true',
        description='Use GPU for inference'
    )
    
    use_tensorrt_arg = DeclareLaunchArgument(
        'use_tensorrt', default_value='true',
        description='Use TensorRT for optimized inference'
    )
    
    use_fp16_arg = DeclareLaunchArgument(
        'use_fp16', default_value='true',
        description='Use FP16 half precision'
    )
    
    input_size_arg = DeclareLaunchArgument(
        'input_size', default_value='1280',
        description='Input image size for YOLO (640, 1280)'
    )
    
    use_preprocessing_arg = DeclareLaunchArgument(
        'use_preprocessing', default_value='true',
        description='Apply image preprocessing (CLAHE)'
    )
    
    use_roi_tracking_arg = DeclareLaunchArgument(
        'use_roi_tracking', default_value='true',
        description='Use ROI-based fast tracking'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence', default_value='0.4',
        description='Detection confidence threshold'
    )
    
    velocity_arg = DeclareLaunchArgument(
        'velocity', default_value='100.0',
        description='Robot max velocity (mm/s)'
    )
    
    acceleration_arg = DeclareLaunchArgument(
        'acceleration', default_value='100.0',
        description='Robot max acceleration (mm/s²)'
    )
    
    show_window_arg = DeclareLaunchArgument(
        'show_window', default_value='false',
        description='Show OpenCV window'
    )
    
    # YOLO Optimized Face Detection Node
    face_detection_node = Node(
        package='face_tracking_pkg',
        executable='face_detection_yolo_optimized_node',
        name='face_detection_node',
        output='screen',
        parameters=[{
            'use_gpu': LaunchConfiguration('use_gpu'),
            'use_tensorrt': LaunchConfiguration('use_tensorrt'),
            'use_fp16': LaunchConfiguration('use_fp16'),
            'input_size': LaunchConfiguration('input_size'),
            'use_preprocessing': LaunchConfiguration('use_preprocessing'),
            'use_roi_tracking': LaunchConfiguration('use_roi_tracking'),
            'confidence_threshold': LaunchConfiguration('confidence'),
            'show_window': LaunchConfiguration('show_window'),
        }]
    )
    
    # Face Tracking Node (3D coordinate transform + EKF)
    face_tracking_node = Node(
        package='face_tracking_pkg',
        executable='face_tracking_node',
        name='face_tracking_node',
        output='screen',
        parameters=[{
            'use_ekf': True,
            'process_noise': 1.0,
            'measurement_noise': 10.0,
            'smoothing_alpha': 0.3,
        }]
    )
    
    # Robot Control Node
    robot_control_node = Node(
        package='face_tracking_pkg',
        executable='robot_control_node',
        name='robot_control_node',
        output='screen',
        parameters=[{
            'max_velocity': LaunchConfiguration('velocity'),
            'max_acceleration': LaunchConfiguration('acceleration'),
            'control_period': 0.1,
        }]
    )
    
    return LaunchDescription([
        use_gpu_arg,
        use_tensorrt_arg,
        use_fp16_arg,
        input_size_arg,
        use_preprocessing_arg,
        use_roi_tracking_arg,
        confidence_arg,
        velocity_arg,
        acceleration_arg,
        show_window_arg,
        face_detection_node,
        face_tracking_node,
        robot_control_node,
    ])
