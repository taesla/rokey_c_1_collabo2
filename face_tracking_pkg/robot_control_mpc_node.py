#!/usr/bin/env python3
"""
Robot Control MPC Node - Full 6-DOF Model Predictive Control

Phase 5-2: MPC 기반 얼굴 추적 (J1~J6 전체 제어)

Features:
  - 6-DOF State Space (12 states: positions + velocities)
  - EKF 필터링된 목표 위치
  - 관절 한계, 속도, 가속도 제약
  - 부드러운 궤적 생성 (jerk minimization)
  - Doosan API FK/IK 활용

Control Pipeline:
  EKF(3D target) → MPC(optimal trajectory) → FK/IK → J1~J6 commands → Robot

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

try:
    import cvxpy as cp
    CVXPY_AVAILABLE = True
except ImportError:
    CVXPY_AVAILABLE = False
    print("⚠️  cvxpy not installed. MPC will not work!")
    print("   Install: pip install cvxpy")


class MPCController:
    """Model Predictive Controller for 6-DOF robot"""
    
    def __init__(self, dt=0.033, horizon=10):
        """
        Args:
            dt: Control timestep (30Hz = 0.033s)
            horizon: Prediction horizon (N steps)
        """
        self.dt = dt
        self.horizon = horizon
        self.n_joints = 6
        
        # State: [q1, q2, ..., q6, q1_dot, ..., q6_dot]  (12-dimensional)
        self.n_states = 2 * self.n_joints
        # Control: [q1_ddot, ..., q6_ddot]  (6-dimensional)
        self.n_controls = self.n_joints
        
        # Joint limits (from joint_limits.yaml)
        self.q_min = np.array([-3.14, -1.3, -2.0, -3.14, -2.0, -3.14])
        self.q_max = np.array([3.14, 1.3, 2.0, 3.14, 2.0, 3.14])
        
        # Velocity limits (rad/s)
        self.q_dot_max = np.array([3.67, 3.32, 3.67, 6.98, 6.98, 10.47])
        self.q_dot_min = -self.q_dot_max
        
        # Acceleration limits (rad/s^2)
        self.q_ddot_max = np.array([0.734, 0.664, 0.734, 1.396, 1.396, 2.094])
        self.q_ddot_min = -self.q_ddot_max
        
        # Cost function weights
        self.Q_pos = 100.0      # Position tracking weight
        self.Q_vel = 1.0        # Velocity regularization
        self.R = 0.1            # Control effort weight
        self.Q_terminal = 200.0 # Terminal cost weight
        
        # System dynamics matrices (discrete-time)
        self.A, self.B = self._build_dynamics_matrices()
        
        print("✅ MPC Controller initialized")
        print(f"   Horizon: {horizon} steps ({horizon * dt:.2f}s)")
        print(f"   Update rate: {1/dt:.1f}Hz")
    
    def _build_dynamics_matrices(self):
        """Build discrete-time linear dynamics: x[k+1] = A*x[k] + B*u[k]"""
        # State: [q, q_dot]
        # Control: u = q_ddot
        # Dynamics: q_dot[k+1] = q_dot[k] + u[k]*dt
        #           q[k+1] = q[k] + q_dot[k]*dt + 0.5*u[k]*dt^2
        
        dt = self.dt
        n = self.n_joints
        
        # A matrix (12x12)
        A = np.eye(2*n)
        A[:n, n:] = np.eye(n) * dt  # q[k+1] = q[k] + q_dot[k]*dt
        
        # B matrix (12x6)
        B = np.zeros((2*n, n))
        B[:n, :] = 0.5 * dt**2 * np.eye(n)  # q[k+1] += 0.5*u[k]*dt^2
        B[n:, :] = dt * np.eye(n)            # q_dot[k+1] = q_dot[k] + u[k]*dt
        
        return A, B
    
    def solve(self, x_current, q_target):
        """
        Solve MPC optimization problem
        
        Args:
            x_current: Current state [q1..q6, q1_dot..q6_dot] (12,)
            q_target: Target joint angles [q1..q6] (6,)
        
        Returns:
            u_opt: Optimal control (first step) [q1_ddot..q6_ddot] (6,)
            success: True if solved successfully
        """
        if not CVXPY_AVAILABLE:
            return np.zeros(self.n_controls), False
        
        N = self.horizon
        n_x = self.n_states
        n_u = self.n_controls
        
        # Decision variables
        x = cp.Variable((n_x, N+1))  # States over horizon
        u = cp.Variable((n_u, N))    # Controls over horizon
        
        # Cost function
        cost = 0
        constraints = []
        
        # Initial condition
        constraints.append(x[:, 0] == x_current)
        
        # Target state (zero velocity)
        x_target = np.concatenate([q_target, np.zeros(self.n_joints)])
        
        for k in range(N):
            # Dynamics constraints
            constraints.append(x[:, k+1] == self.A @ x[:, k] + self.B @ u[:, k])
            
            # State constraints
            constraints.append(x[:self.n_joints, k+1] >= self.q_min)
            constraints.append(x[:self.n_joints, k+1] <= self.q_max)
            constraints.append(x[self.n_joints:, k+1] >= self.q_dot_min)
            constraints.append(x[self.n_joints:, k+1] <= self.q_dot_max)
            
            # Control constraints
            constraints.append(u[:, k] >= self.q_ddot_min)
            constraints.append(u[:, k] <= self.q_ddot_max)
            
            # Stage cost
            position_error = x[:self.n_joints, k] - q_target
            velocity_cost = x[self.n_joints:, k]
            control_cost = u[:, k]
            
            cost += self.Q_pos * cp.sum_squares(position_error)
            cost += self.Q_vel * cp.sum_squares(velocity_cost)
            cost += self.R * cp.sum_squares(control_cost)
        
        # Terminal cost
        terminal_error = x[:, N] - x_target
        cost += self.Q_terminal * cp.sum_squares(terminal_error)
        
        # Solve
        problem = cp.Problem(cp.Minimize(cost), constraints)
        
        try:
            problem.solve(solver=cp.OSQP, verbose=False, warm_start=True)
            
            if problem.status == cp.OPTIMAL or problem.status == cp.OPTIMAL_INACCURATE:
                u_opt = u[:, 0].value  # First control input
                return u_opt, True
            else:
                print(f"⚠️  MPC solver failed: {problem.status}")
                return np.zeros(n_u), False
                
        except Exception as e:
            print(f"❌ MPC error: {e}")
            return np.zeros(n_u), False


class RobotControlMPCNode(Node):
    """MPC 기반 로봇 제어 노드"""
    
    def __init__(self):
        super().__init__('robot_control_mpc_node')
        
        # Parameters
        self.declare_parameter('robot_id', 'dsr01')
        self.declare_parameter('robot_model', 'm0609')
        self.declare_parameter('mpc_horizon', 10)
        self.declare_parameter('control_rate', 30.0)
        
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_model = self.get_parameter('robot_model').value
        self.mpc_horizon = self.get_parameter('mpc_horizon').value
        self.control_rate = self.get_parameter('control_rate').value
        self.dt = 1.0 / self.control_rate
        
        # DSR 함수들은 main()에서 import 후 할당됨
        self.movej = None
        self.get_current_posj = None
        self.set_robot_mode = None
        self.fkin = None
        self.ikin = None
        self.ROBOT_MODE_MANUAL = None
        
        # MPC Controller
        self.mpc = MPCController(dt=self.dt, horizon=self.mpc_horizon)
        
        # State
        self.state = "IDLE"  # IDLE, TRACKING
        self.current_q = np.zeros(6)       # Current joint angles
        self.current_q_dot = np.zeros(6)   # Current joint velocities
        self.target_pos_3d = None          # Target 3D position (x, y, z)
        
        # Subscriber - EKF 필터링된 로봇 프레임 마커 사용
        self.marker_sub = self.create_subscription(
            Marker, '/face_tracking/marker_ekf_filtered', self.marker_callback, 10)
        
        # Control timer (30Hz)
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 Robot Control MPC Node (Full 6-DOF)")
        self.get_logger().info(f"  Robot: {self.robot_id} / {self.robot_model}")
        self.get_logger().info(f"  MPC Horizon: {self.mpc_horizon} ({self.mpc_horizon * self.dt:.2f}s)")
        self.get_logger().info(f"  Control Rate: {self.control_rate}Hz")
        self.get_logger().info("  키: 's'=시작, 'h'=홈, 'q'=종료")
        self.get_logger().info("=" * 60)
    
    def marker_callback(self, msg):
        """EKF 필터링된 얼굴 위치 마커 수신 (로봇 base_link 프레임)"""
        # EKF 필터링된 위치 (meter) - robot_control_node에서 발행
        # 이미 로봇 좌표계이므로 TF 변환 불필요
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # mm 단위로 변환 (Doosan API는 mm 사용)
        self.target_pos_3d = np.array([x * 1000.0, y * 1000.0, z * 1000.0])
    
    def control_loop(self):
        """메인 제어 루프 (30Hz)"""
        if self.state != "TRACKING":
            return
        
        if self.target_pos_3d is None:
            self.get_logger().warn("⚠️  target_pos_3d is None", throttle_duration_sec=2.0)
            return
        
        # 1. Get current joint state
        if self.get_current_posj is None:
            self.get_logger().error("❌ get_current_posj is None!")
            return
            
        current_posj = self.get_current_posj()
        if current_posj is None:
            self.get_logger().warn("⚠️  current_posj is None", throttle_duration_sec=2.0)
            return
        
        self.current_q = np.deg2rad(current_posj)  # Convert to radians
        
        # 2. Compute target joint angles using IK
        # target_pos_3d는 이미 mm 단위 (marker_callback에서 변환됨)
        x, y, z = self.target_pos_3d
        
        # Doosan IK: posx = [x, y, z, rx, ry, rz] (mm, deg)
        # Orientation: 카메라가 얼굴을 향하도록 (나중에 추가)
        # 현재는 고정 orientation 사용
        target_posx = [x, y, z, 0.0, 180.0, 0.0]  # [mm, deg]
        
        try:
            # IK 계산 (solution_space는 두 번째 인자)
            ik_result = self.ikin(target_posx, 0)  # solution_space=0 (기본 해)
            
            if ik_result is not None and len(ik_result) >= 6:
                # Doosan IK는 degree 반환 → radian 변환
                target_q = np.deg2rad(ik_result[:6])
            else:
                # IK 실패 시 2-DOF 근사 사용
                self.get_logger().warn("IK failed, using 2-DOF approximation")
                target_q = self.current_q.copy()
                target_q[0] = np.arctan2(y, x)  # J1
                target_q[4] = np.arctan2(z, np.sqrt(x**2 + y**2))  # J5
                
        except Exception as e:
            self.get_logger().error(f"IK error: {e}, using 2-DOF approximation")
            target_q = self.current_q.copy()
            target_q[0] = np.arctan2(y, x)
            target_q[4] = np.arctan2(z, np.sqrt(x**2 + y**2))
        
        # 3. Run MPC
        x_current = np.concatenate([self.current_q, self.current_q_dot])
        u_opt, success = self.mpc.solve(x_current, target_q)
        
        if not success:
            self.get_logger().warn("⚠️  MPC solve failed", throttle_duration_sec=2.0)
            return
        
        # 4. Apply control (integrate to get next position)
        next_q = self.current_q + self.current_q_dot * self.dt + 0.5 * u_opt * self.dt**2
        next_q_deg = np.rad2deg(next_q)
        
        # 5. Send command to robot
        if self.movej is None:
            self.get_logger().error("❌ movej is None!")
            return
            
        self.movej(next_q_deg.tolist(), vel=30, acc=30)
        
        # Debug log (1초마다)
        if not hasattr(self, '_last_debug_time'):
            self._last_debug_time = time.time()
        
        if time.time() - self._last_debug_time > 1.0:
            self.get_logger().info(
                f"🎯 Target: [{x:.0f}, {y:.0f}, {z:.0f}]mm | "
                f"J1: {np.rad2deg(next_q[0]):.1f}° | "
                f"J5: {np.rad2deg(next_q[4]):.1f}°"
            )
            self._last_debug_time = time.time()
        
        # Update velocity estimate
        self.current_q_dot += u_opt * self.dt


def main(args=None):
    rclpy.init(args=args)
    
    if not CVXPY_AVAILABLE:
        print("❌ cvxpy is required for MPC!")
        print("   Install: pip install cvxpy")
        return
    
    # MPC 노드 먼저 생성
    node = RobotControlMPCNode()
    
    # DSR 초기화
    DR_init.__dsr__id = node.robot_id
    DR_init.__dsr__model = node.robot_model
    
    # DSR 노드 생성 (별도)
    dsr_node = rclpy.create_node("robot_control_mpc_dsr", namespace=node.robot_id)
    DR_init.__dsr__node = dsr_node
    
    # DSR 함수 import
    try:
        from DSR_ROBOT2 import (
            set_robot_mode, get_current_posj, movej,
            set_velj, set_accj, fkin, ikin,
            ROBOT_MODE_MANUAL
        )
        node.movej = movej
        node.get_current_posj = get_current_posj
        node.set_robot_mode = set_robot_mode
        node.fkin = fkin
        node.ikin = ikin
        node.ROBOT_MODE_MANUAL = ROBOT_MODE_MANUAL
        print("✅ DSR 모듈 import 성공")
        
    except ImportError as e:
        print(f"❌ DSR 모듈 import 실패: {e}")
        return
    
    print("\n>>> 키 입력:")
    print("  's' - TRACKING 모드 시작")
    print("  'h' - 홈 위치로 이동")
    print("  'q' - 종료")
    print(">>> ")
    
    # Executor로 두 노드 함께 실행
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(dsr_node)
    
    # 홈 위치
    home_joints = [0, 0, 90, 0, 90, 0]
    
    try:
        while rclpy.ok():
            # ROS2 노드 spin
            for _ in range(10):
                executor.spin_once(timeout_sec=0.01)
            
            # 키보드 입력 처리
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.readline().strip().lower()
                
                if key == 's':
                    node.get_logger().info("🚀 TRACKING 모드 시작!")
                    node.state = "TRACKING"
                
                elif key == 'h':
                    node.get_logger().info("🏠 홈 위치로 이동...")
                    node.state = "IDLE"
                    movej(home_joints, vel=30, acc=30)
                    node.get_logger().info("✅ 홈 도착!")
                
                elif key == 'q':
                    print("종료합니다...")
                    break
    
    except KeyboardInterrupt:
        print("\n키보드 인터럽트")
    finally:
        executor.shutdown()
        node.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("정상 종료")


if __name__ == "__main__":
    main()
