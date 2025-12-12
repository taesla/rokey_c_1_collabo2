#!/usr/bin/env python3
"""
RealSense Camera Performance Test
- RGB 및 Depth 스트림 Hz 측정
- 프레임 타이밍 분석
- 동기화 상태 확인
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time
from collections import deque


class CameraPerformanceTest(Node):
    def __init__(self):
        super().__init__('camera_performance_test')
        
        # RGB 스트림 모니터링
        self.rgb_times = deque(maxlen=100)
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )
        
        # Depth 스트림 모니터링
        self.depth_times = deque(maxlen=100)
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        # 통계 출력 타이머
        self.timer = self.create_timer(2.0, self.print_stats)
        
        self.get_logger().info('🎥 Camera Performance Test Started')
        self.get_logger().info('📊 Monitoring RGB and Depth streams...')
        
    def rgb_callback(self, msg):
        current_time = time.time()
        self.rgb_times.append(current_time)
        
    def depth_callback(self, msg):
        current_time = time.time()
        self.depth_times.append(current_time)
        
    def print_stats(self):
        self.get_logger().info('=' * 60)
        
        # RGB 통계
        if len(self.rgb_times) >= 2:
            rgb_intervals = [
                self.rgb_times[i] - self.rgb_times[i-1] 
                for i in range(1, len(self.rgb_times))
            ]
            rgb_fps = 1.0 / (sum(rgb_intervals) / len(rgb_intervals))
            rgb_std = (sum((x - sum(rgb_intervals)/len(rgb_intervals))**2 
                          for x in rgb_intervals) / len(rgb_intervals)) ** 0.5
            
            self.get_logger().info(
                f'📷 RGB Stream:   {rgb_fps:.2f} Hz (±{rgb_std*1000:.2f}ms)'
            )
        else:
            self.get_logger().warn('⚠️  RGB Stream: No data')
            
        # Depth 통계
        if len(self.depth_times) >= 2:
            depth_intervals = [
                self.depth_times[i] - self.depth_times[i-1] 
                for i in range(1, len(self.depth_times))
            ]
            depth_fps = 1.0 / (sum(depth_intervals) / len(depth_intervals))
            depth_std = (sum((x - sum(depth_intervals)/len(depth_intervals))**2 
                            for x in depth_intervals) / len(depth_intervals)) ** 0.5
            
            self.get_logger().info(
                f'📏 Depth Stream: {depth_fps:.2f} Hz (±{depth_std*1000:.2f}ms)'
            )
        else:
            self.get_logger().warn('⚠️  Depth Stream: No data')
            
        # 동기화 상태
        if len(self.rgb_times) >= 1 and len(self.depth_times) >= 1:
            sync_diff = abs(self.rgb_times[-1] - self.depth_times[-1])
            if sync_diff < 0.01:  # 10ms 이내
                self.get_logger().info(f'✅ Sync: Good ({sync_diff*1000:.1f}ms)')
            else:
                self.get_logger().warn(f'⚠️  Sync: Poor ({sync_diff*1000:.1f}ms)')


def main(args=None):
    rclpy.init(args=args)
    node = CameraPerformanceTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
