#!/usr/bin/env python3
"""
Unified Robot Control MPC Node - Full 6-DOF with EKF Integration

단일 노드에서 전체 파이프라인 처리:
  MediaPipe (Raw) → EKF (Filtering) → MPC (Trajectory) → Robot

States:
  - IDLE: 홈 위치, 얼굴 감지 대기
  - TRACKING: 얼굴 추적, MPC 제어

Sequence:
  1. 얼굴 감지 → 즉시 TRACKING 모드
  2. 3-5초 미감지 → IDLE 모드 (홈 복귀)

Author: Rokey AI Lab
Date: 2025-12-10
"""
import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import DR_init
from face_tracking_pkg.face_tracking_ekf import FaceTrackingEKF

try:
    import cvxpy as cp
    CVXPY_AVAILABLE = True
except ImportError:
    CVXPY_AVAILABLE = False
    print("⚠️  cvxpy not installed!")


class MPCController:
    """Model Predictive Controller for 6-DOF robot"""
    
    def __init__(self, dt=0.033, horizon=10):
        self.dt = dt
        self.horizon = horizon
        self.n_joints = 6
        self.n_states = 12  # [q1..q6, q1_dot..q6_dot]
        self.n_controls = 6  # [q1_ddot..q6_ddot]
        
        # Joint limits
        self.q_min = np.array([-3.14, -1.3, -2.0, -3.14, -2.0, -3.14])
        self.q_max = np.array([3.14, 1.3, 2.0, 3.14, 2.0, 3.14])
        self.q_dot_max = np.array([3.67, 3.32, 3.67, 6.98, 6.98, 10.47])
        self.q_dot_min = -self.q_dot_max
        self.q_ddot_max = np.array([0.734, 0.664, 0.734, 1.396, 1.396, 2.094])
        self.q_ddot_min = -self.q_ddot_max
        
        # Cost weights
        self.Q_pos = 100.0
        self.Q_vel = 1.0
        self.R = 0.1
        self.Q_terminal = 200.0
        
        # System dynamics
        self.A, self.B = self._build_dynamics_matrices()
    
    def _build_dynamics_matrices(self):
        """Discrete-time dynamics: x[k+1] = A*x[k] + B*u[k]"""
        dt = self.dt
        n = self.n_joints
        
        A = np.eye(2*n)
        A[:n, n:] = np.eye(n) * dt
        
        B = np.zeros((2*n, n))
        B[:n, :] = 0.5 * dt**2 * np.eye(n)
        B[n:, :] = dt * np.eye(n)
        
        return A, B
    
    def solve(self, x_current, q_target):
        """Solve MPC optimization"""
        if not CVXPY_AVAILABLE:
            return np.zeros(self.n_controls), False
        
        N = self.horizon
        n_x = self.n_states
        n_u = self.n_controls
        
        x = cp.Variable((n_x, N+1))
        u = cp.Variable((n_u, N))
        
        cost = 0
        constraints = [x[:, 0] == x_current]
        x_target = np.concatenate([q_target, np.zeros(self.n_joints)])
        
        for k in range(N):
            constraints.append(x[:, k+1] == self.A @ x[:, k] + self.B @ u[:, k])
            constraints.append(x[:self.n_joints, k+1] >= self.q_min)
            constraints.append(x[:self.n_joints, k+1] <= self.q_max)
            constraints.append(x[self.n_joints:, k+1] >= self.q_dot_min)
            constraints.append(x[self.n_joints:, k+1] <= self.q_dot_max)
            constraints.append(u[:, k] >= self.q_ddot_min)
            constraints.append(u[:, k] <= self.q_ddot_max)
            
            position_error = x[:self.n_joints, k] - q_target
            velocity = x[self.n_joints:, k]
            cost += self.Q_pos * cp.sum_squares(position_error)
            cost += self.Q_vel * cp.sum_squares(velocity)
            cost += self.R * cp.sum_squares(u[:, k])
        
        # Terminal cost
        terminal_error = x[:self.n_joints, N] - q_target
        cost += self.Q_terminal * cp.sum_squares(terminal_error)
        
        problem = cp.Problem(cp.Minimize(cost), constraints)
        
        try:
            problem.solve(solver=cp.OSQP, verbose=False, warm_start=True)
            
            if problem.status == cp.OPTIMAL or problem.status == cp.OPTIMAL_INACCURATE:
                return u.value[:, 0], True
            else:
                return np.zeros(n_u), False
        except Exception as e:
            return np.zeros(n_u), False


class RobotControlMPCUnified(Node):
    """통합 MPC 제어 노드 - EKF + MPC + 상태머신"""
    
    def __init__(self, movej_func, get_current_posj_func, ikin_func):
        super().__init__('robot_control_mpc_unified')
        
        # DSR 함수들 저장
        self.movej = movej_func
        self.get_current_posj = get_current_posj_func
        self.ikin = ikin_func
        
        # Parameters
        self.declare_parameter('detection_timeout', 4.0)  # 3-5초
        self.declare_parameter('ekf_process_noise', 0.1)
        self.declare_parameter('ekf_measurement_noise', 5.0)
        
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.ekf_q = self.get_parameter('ekf_process_noise').value
        self.ekf_r = self.get_parameter('ekf_measurement_noise').value
        
        # Home position
        self.home_joints = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.start_joints = [3.06, 2.84, 92.13, 86.07, -1.43, 8.33]
        
        # State machine
        self.state = "IDLE"  # IDLE or TRACKING
        self.last_detection_time = None
        
        # EKF initialization
        self.ekf = FaceTrackingEKF(dt=0.033)
        self.ekf.kf.Q = np.eye(9) * self.ekf_q
        self.ekf.kf.Q[6:9, 6:9] *= 2.0
        self.ekf.kf.R = np.eye(3) * self.ekf_r
        
        # MPC initialization
        self.mpc = MPCController(dt=0.033, horizon=10)
        self.dt = 0.033  # 30Hz
        
        # Current state
        self.current_q = None
        self.current_q_dot = np.zeros(6)
        self.target_pos_3d = None
        self.solution_space = 0  # IK solution space (0~7)
        
        # Subscriptions (Robot frame marker from face_tracking_node)
        self.marker_sub = self.create_subscription(
            Marker, '/face_tracking/marker_robot', self.marker_callback, 10)
        
        # Publishers (filtered marker for visualization)
        self.ekf_marker_pub = self.create_publisher(
            Marker, '/face_tracking/marker_ekf_filtered', 10)
        
        # Control timer (30Hz)
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        # Log timer (1Hz)
        self.log_timer = self.create_timer(1.0, self.log_status)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 Unified Robot Control MPC Node")
        self.get_logger().info(f"  EKF: Q={self.ekf_q}, R={self.ekf_r}")
        self.get_logger().info(f"  Detection Timeout: {self.detection_timeout}s")
        self.get_logger().info(f"  Control: 30Hz MPC")
        self.get_logger().info("  Keys: 's'=Start, 'h'=Home, 'q'=Quit")
        self.get_logger().info("=" * 60)
    
    def marker_callback(self, msg):
        """Robot frame marker 수신 (base_link) → EKF 필터링"""
        # Convert to mm
        raw_x = msg.pose.position.x * 1000.0
        raw_y = msg.pose.position.y * 1000.0
        raw_z = msg.pose.position.z * 1000.0
        
        # Basic sanity check (넓은 범위)
        if not (100 < raw_x < 2000 and -800 < raw_y < 800 and 100 < raw_z < 1200):
            self.get_logger().warn(f"⚠️  Out of range: [{raw_x:.0f}, {raw_y:.0f}, {raw_z:.0f}]mm", throttle_duration_sec=2.0)
            return
        
        # Outlier rejection
        if self.target_pos_3d is not None:
            dx = abs(raw_x - self.target_pos_3d[0])
            dy = abs(raw_y - self.target_pos_3d[1])
            dz = abs(raw_z - self.target_pos_3d[2])
            if dx > 200 or dy > 200 or dz > 200:
                return
        
        raw_pos = [raw_x, raw_y, raw_z]
        
        # EKF filtering
        if not self.ekf.initialized:
            self.ekf.initialize(raw_pos)
            self.target_pos_3d = np.array(raw_pos)
            self.get_logger().info(f"🔬 EKF initialized: [{raw_x:.0f}, {raw_y:.0f}, {raw_z:.0f}]mm")
        else:
            self.ekf.predict()
            self.ekf.update(raw_pos)
            filtered_pos = self.ekf.get_position()
            self.target_pos_3d = filtered_pos
        
        # Update detection time
        self.last_detection_time = time.time()
        
        # State transition: IDLE → TRACKING
        if self.state == "IDLE":
            self.get_logger().info("🎯 Face detected! → TRACKING")
            self.state = "TRACKING"
        
        # Publish EKF marker (blue)
        self.publish_ekf_marker(self.target_pos_3d)
    
    def publish_ekf_marker(self, filtered_pos):
        """Publish filtered marker for visualization"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "face_ekf_unified"
        marker.id = 20
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = filtered_pos[0] / 1000.0
        marker.pose.position.y = filtered_pos[1] / 1000.0
        marker.pose.position.z = filtered_pos[2] / 1000.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.ekf_marker_pub.publish(marker)
    
    def control_loop(self):
        """Main control loop - 30Hz"""
        # Get current joint state
        current_posj = self.get_current_posj()
        if current_posj is None:
            return
        self.current_q = np.deg2rad(current_posj)  # Convert to radians
        
        if self.state == "IDLE":
            # Check if in home position, if not move there
            home_rad = np.deg2rad(self.home_joints)
            error = np.linalg.norm(self.current_q - home_rad)
            if error > 0.05:  # ~3 degrees
                self.movej(self.home_joints, vel=30, acc=30)
        
        elif self.state == "TRACKING":
            # Check timeout
            if self.last_detection_time is not None:
                elapsed = time.time() - self.last_detection_time
                if elapsed > self.detection_timeout:
                    self.get_logger().info(f"⏱️  Face lost for {elapsed:.1f}s → IDLE (Home)")
                    self.state = "IDLE"
                    return
            
            # MPC control
            if self.target_pos_3d is not None:
                x, y, z = self.target_pos_3d
                
                # IK: Convert target position to joint angles
                target_posx = [x, y, z, 0.0, 180.0, 0.0]  # mm, deg
                
                try:
                    # Try multiple solution spaces to find best pose
                    best_ik = None
                    min_distance = float('inf')
                    
                    for sol_space in range(8):  # 0~7
                        try:
                            ik_result = self.ikin(target_posx, sol_space)
                            if ik_result is not None:
                                ik_q = np.deg2rad(ik_result[:6])
                                # Choose solution closest to current pose
                                distance = np.linalg.norm(ik_q - self.current_q)
                                if distance < min_distance:
                                    min_distance = distance
                                    best_ik = ik_q
                                    self.solution_space = sol_space
                        except:
                            continue
                    
                    if best_ik is not None:
                        target_q = best_ik
                    else:
                        raise Exception("No valid IK solution")
                        
                except Exception as e:
                    # Fallback: 2-DOF approximation
                    r = math.sqrt(x**2 + y**2)
                    j1_deg = math.degrees(math.atan2(y, x))
                    j5_deg = math.degrees(math.atan2(z - 500, r - 300))
                    target_q = self.current_q.copy()
                    target_q[0] = math.radians(j1_deg)
                    target_q[4] = math.radians(j5_deg)
                
                # MPC solve
                x_current = np.concatenate([self.current_q, self.current_q_dot])
                u_opt, success = self.mpc.solve(x_current, target_q)
                
                if success:
                    # Integrate to get next joint positions
                    next_q = self.current_q + self.current_q_dot * self.dt + 0.5 * u_opt * self.dt**2
                    next_q_dot = self.current_q_dot + u_opt * self.dt
                    
                    # Update velocity estimate
                    self.current_q_dot = next_q_dot
                    
                    # Send command
                    next_q_deg = np.rad2deg(next_q)
                    self.movej(next_q_deg.tolist(), vel=30, acc=30)
    
    def log_status(self):
        """Status logging - 1Hz"""
        if self.state == "IDLE":
            self.get_logger().info("💤 IDLE - Waiting for face detection...")
        elif self.state == "TRACKING":
            if self.target_pos_3d is not None:
                x, y, z = self.target_pos_3d
                if self.current_q is not None:
                    joints = np.rad2deg(self.current_q)
                    elapsed = time.time() - self.last_detection_time if self.last_detection_time else 0
                    self.get_logger().info(
                        f"🎯 TRACKING | Target: [{x:.0f}, {y:.0f}, {z:.0f}]mm | "
                        f"J1: {joints[0]:.1f}° J2: {joints[1]:.1f}° J3: {joints[2]:.1f}° "
                        f"J4: {joints[3]:.1f}° J5: {joints[4]:.1f}° J6: {joints[5]:.1f}° | "
                        f"Lost: {elapsed:.1f}s"
                    )


def main(args=None):
    rclpy.init(args=args)
    
    # Initialize DSR robot
    print("🔧 Initializing Doosan robot...")
    
    # DSR 설정
    DR_init.__dsr__id = 'dsr01'
    DR_init.__dsr__model = 'm0609'
    
    # DSR 노드 생성
    dsr_node = rclpy.create_node('robot_control_dsr', namespace='dsr01')
    DR_init.__dsr__node = dsr_node
    
    # DSR 함수 import
    try:
        from DSR_ROBOT2 import movej, get_current_posj, ikin
        print("✅ DSR 모듈 import 성공")
    except ImportError as e:
        print(f"❌ DSR 모듈 import 실패: {e}")
        return
    
    print("✅ Robot initialized")
    
    # Create control node (pass DSR functions)
    node = RobotControlMPCUnified(movej, get_current_posj, ikin)
    
    # Executor로 두 노드 함께 실행
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(dsr_node)
    
    print("\n" + "=" * 60)
    print("🎮 Controls:")
    print("  's' - Start (move to start position & tracking)")
    print("  'h' - Go to home position")
    print("  'q' - Quit")
    print("=" * 60)
    print(">>> ", end='', flush=True)
    
    try:
        while rclpy.ok():
            # ROS2 spin
            for _ in range(10):
                executor.spin_once(timeout_sec=0.01)
            
            # Check keyboard input
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.readline().strip().lower()
                
                if key == 's':
                    node.get_logger().info("📦 시작 위치로 이동 중...")
                    node.state = "IDLE"
                    movej(node.start_joints, vel=30, acc=30)
                    node.get_logger().info("✅ 시작 위치 도착! 얼굴 감지 대기 중...")
                    node.state = "TRACKING"  # 디텍션 대기
                    node.last_detection_time = None  # 초기화
                
                elif key == 'h':
                    node.get_logger().info("🏠 홈 위치로 이동...")
                    node.state = "IDLE"
                    movej(node.home_joints, vel=50, acc=50)
                    node.get_logger().info("✅ 홈 도착!")
                
                elif key == 'q':
                    node.get_logger().info("👋 종료...")
                    break
                
                print(">>> ", end='', flush=True)
    
    finally:
        executor.shutdown()
        node.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
