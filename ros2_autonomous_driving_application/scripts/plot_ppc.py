#!/usr/bin/env python3
"""
Pure Pursuit Controller 실시간 시각화 스크립트

기능:
  - Waypoints (계획 경로) 시각화
  - Actual Path (실제 주행 경로) 시각화
  - Lookahead Point 실시간 표시
  - Robot 현재 위치/방향 표시
  - 속도 명령 그래프
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
from tf_transformations import euler_from_quaternion
import json
from datetime import datetime
import os

class PPCPlotter(Node):
    def __init__(self):
        super().__init__('ppc_plotter')
        
        # Subscribers
        self.create_subscription(Path, '/waypoints_path', self.waypoints_callback, 10)
        self.create_subscription(Odometry, '/odometry/ekf_single', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_ppc', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/ppc/enable', self.enable_callback, 10)
        
        # Data storage
        self.waypoints = []
        self.actual_path = deque(maxlen=1000)  # 최대 1000개 저장
        self.current_pose = None
        self.current_yaw = 0.0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.is_enabled = False
        
        # 시간별 속도 기록 (최대 100개)
        self.time_stamps = deque(maxlen=100)
        self.linear_velocities = deque(maxlen=100)
        self.angular_velocities = deque(maxlen=100)
        self.start_time = self.get_clock().now()
        
        # 주행 시작 시간
        self.run_start_time = None
        
        # Plot 설정
        self.setup_plot()
        
    def waypoints_callback(self, msg: Path):
        """Waypoints 경로 수신"""
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) 
                         for pose in msg.poses]
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')
        
    def odom_callback(self, msg: Odometry):
        """현재 위치 수신"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.current_pose = (x, y)
        self.current_yaw = yaw
        
        # PPC 활성화 중일 때만 경로 기록
        if self.is_enabled:
            self.actual_path.append((x, y))
            
    def cmd_vel_callback(self, msg: Twist):
        """속도 명령 수신"""
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
        
        # 시간별 속도 기록
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        self.time_stamps.append(elapsed)
        self.linear_velocities.append(self.cmd_linear)
        self.angular_velocities.append(self.cmd_angular)
        
    def enable_callback(self, msg: Bool):
        """PPC 활성화 상태 수신"""
        if msg.data and not self.is_enabled:
            # 활성화: 경로 초기화
            self.actual_path.clear()
            self.run_start_time = self.get_clock().now()
            self.get_logger().info('PPC Enabled - Recording started')
        elif not msg.data and self.is_enabled:
            # 비활성화: 데이터 저장
            self.save_run_data()
            
        self.is_enabled = msg.data
        
    def setup_plot(self):
        """Matplotlib Figure 설정"""
        self.fig = plt.figure(figsize=(16, 10))
        
        # 2x2 레이아웃
        gs = self.fig.add_gridspec(2, 2, hspace=0.3, wspace=0.3)
        
        # 1. 경로 플롯 (큰 영역)
        self.ax_path = self.fig.add_subplot(gs[:, 0])
        self.ax_path.set_title('Pure Pursuit Trajectory', fontsize=14, fontweight='bold')
        self.ax_path.set_xlabel('X (m)')
        self.ax_path.set_ylabel('Y (m)')
        self.ax_path.grid(True, alpha=0.3)
        self.ax_path.axis('equal')
        
        # 2. 선속도 그래프
        self.ax_linear = self.fig.add_subplot(gs[0, 1])
        self.ax_linear.set_title('Linear Velocity', fontsize=12, fontweight='bold')
        self.ax_linear.set_xlabel('Time (s)')
        self.ax_linear.set_ylabel('v (m/s)')
        self.ax_linear.grid(True, alpha=0.3)
        
        # 3. 각속도 그래프
        self.ax_angular = self.fig.add_subplot(gs[1, 1])
        self.ax_angular.set_title('Angular Velocity', fontsize=12, fontweight='bold')
        self.ax_angular.set_xlabel('Time (s)')
        self.ax_angular.set_ylabel('ω (rad/s)')
        self.ax_angular.grid(True, alpha=0.3)
        
        # Plot 요소 초기화
        self.waypoints_line, = self.ax_path.plot([], [], 'r--', linewidth=2, 
                                                  label='Waypoints', marker='o', markersize=6)
        self.actual_line, = self.ax_path.plot([], [], 'b-', linewidth=2, 
                                               label='Actual Path', alpha=0.7)
        self.robot_marker, = self.ax_path.plot([], [], 'go', markersize=12, 
                                                label='Robot')
        self.robot_arrow = None
        
        self.linear_line, = self.ax_linear.plot([], [], 'b-', linewidth=2)
        self.angular_line, = self.ax_angular.plot([], [], 'r-', linewidth=2)
        
        self.ax_path.legend(loc='upper right')
        
        # 애니메이션 설정
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, 
                                blit=False, cache_frame_data=False)
        
    def update_plot(self, frame):
        """Plot 업데이트 (100ms마다 호출)"""
        # 1. Waypoints 그리기
        if self.waypoints:
            wp_x = [wp[0] for wp in self.waypoints]
            wp_y = [wp[1] for wp in self.waypoints]
            self.waypoints_line.set_data(wp_x, wp_y)
            
        # 2. Actual Path 그리기
        if self.actual_path:
            path_x = [p[0] for p in self.actual_path]
            path_y = [p[1] for p in self.actual_path]
            self.actual_line.set_data(path_x, path_y)
            
        # 3. Robot 위치/방향 그리기
        if self.current_pose:
            x, y = self.current_pose
            self.robot_marker.set_data([x], [y])
            
            # 로봇 방향 화살표 업데이트
            if self.robot_arrow:
                self.robot_arrow.remove()
            
            arrow_length = 0.5
            dx = arrow_length * np.cos(self.current_yaw)
            dy = arrow_length * np.sin(self.current_yaw)
            
            self.robot_arrow = self.ax_path.arrow(
                x, y, dx, dy,
                head_width=0.3, head_length=0.2,
                fc='green', ec='darkgreen', linewidth=2
            )
            
        # 4. Axis 범위 자동 조정
        if self.waypoints or self.actual_path:
            all_x = []
            all_y = []
            
            if self.waypoints:
                all_x.extend([wp[0] for wp in self.waypoints])
                all_y.extend([wp[1] for wp in self.waypoints])
                
            if self.actual_path:
                all_x.extend([p[0] for p in self.actual_path])
                all_y.extend([p[1] for p in self.actual_path])
                
            if all_x and all_y:
                margin = 2.0
                self.ax_path.set_xlim(min(all_x) - margin, max(all_x) + margin)
                self.ax_path.set_ylim(min(all_y) - margin, max(all_y) + margin)
                
        # 5. 속도 그래프 업데이트
        if self.time_stamps:
            times = list(self.time_stamps)
            linears = list(self.linear_velocities)
            angulars = list(self.angular_velocities)
            
            self.linear_line.set_data(times, linears)
            self.angular_line.set_data(times, angulars)
            
            # X축 범위 조정
            if times:
                time_margin = 5.0
                self.ax_linear.set_xlim(max(0, times[-1] - 30), times[-1] + time_margin)
                self.ax_angular.set_xlim(max(0, times[-1] - 30), times[-1] + time_margin)
                
            # Y축 범위 조정
            if linears:
                y_margin = 0.1
                self.ax_linear.set_ylim(min(linears) - y_margin, max(linears) + y_margin)
                
            if angulars:
                y_margin = 0.2
                max_ang = max(abs(min(angulars)), abs(max(angulars)))
                self.ax_angular.set_ylim(-max_ang - y_margin, max_ang + y_margin)
                
        # 상태 텍스트 표시
        status_text = f"PPC: {'ON' if self.is_enabled else 'OFF'} | "
        status_text += f"v={self.cmd_linear:.2f} m/s | "
        status_text += f"ω={self.cmd_angular:.2f} rad/s"
        
        self.fig.suptitle(status_text, fontsize=14, fontweight='bold',
                         color='green' if self.is_enabled else 'red')
        
        return [self.waypoints_line, self.actual_line, self.robot_marker, 
                self.linear_line, self.angular_line]
    
    def save_run_data(self):
        """주행 데이터 JSON으로 저장"""
        if not self.actual_path:
            self.get_logger().warn('No path data to save')
            return
            
        data_dir = os.path.expanduser('~/ppc_run_data')
        os.makedirs(data_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(data_dir, f'ppc_run_{timestamp}.json')
        
        data = {
            'timestamp': timestamp,
            'waypoints': self.waypoints,
            'actual_path': list(self.actual_path),
            'total_points': len(self.actual_path),
            'run_duration': (self.get_clock().now() - self.run_start_time).nanoseconds * 1e-9 
                           if self.run_start_time else 0.0
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
            
        self.get_logger().info(f'Run data saved: {filename}')
        
        # PNG로도 저장
        png_filename = os.path.join(data_dir, f'ppc_run_{timestamp}.png')
        self.fig.savefig(png_filename, dpi=150, bbox_inches='tight')
        self.get_logger().info(f'Plot saved: {png_filename}')


def main(args=None):
    rclpy.init(args=args)
    
    plotter = PPCPlotter()
    
    # ROS2 spin을 별도 스레드에서 실행
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(plotter,), daemon=True)
    spin_thread.start()
    
    # Matplotlib 메인 루프
    plt.show()
    
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()