from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'face_tracking_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일 포함
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='Face detection and tracking for Doosan robot with RealSense camera',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'face_detection_node = face_tracking_pkg.face_detection_node:main',
            'face_tracking_node = face_tracking_pkg.face_tracking_node:main',
            'robot_control_node = face_tracking_pkg.robot_control_node:main',
            'robot_control_mpc_node = face_tracking_pkg.robot_control_mpc_node:main',
            'robot_control_mpc_unified = face_tracking_pkg.robot_control_mpc_unified:main',
            'camera_performance_test = face_tracking_pkg.camera_performance_test:main',
            'ekf_comparison_node = face_tracking_pkg.ekf_comparison_node:main',
            'ekf_before_after = face_tracking_pkg.ekf_before_after:main',
        ],
    },
)
