#!/usr/bin/env python3
"""
Pure Pursuit Controller 실시간 시각화 스크립트 (구독 전용)

기능:
  - Waypoints (계획 경로) 시각화
  - Actual Path (실제 주행 경로) 시각화
  - Lookahead Point 실시간 표시 (제어 노드에서 수신)
  - Robot 현재 위치/방향 표시
  - Cross-Track Error, Heading Error 그래프 (제어 노드에서 수신)
  - State (move/rotate), Index 정보 표시
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String, Int32, Float64
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
from tf_transformations import euler_from_quaternion
import json
from datetime import datetime
import os
import math

class PlotPPC(Node):
    def __init__(self):
        super().__init__('ppc_plotter')
        
        # ========== Subscribers ==========
        # 기본 데이터
        self.create_subscription(Path, '/waypoints_path', self.waypoints_callback, 10)
        self.create_subscription(Odometry, '/odometry/ekf_single', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_ppc', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/ppc/enable', self.enable_callback, 10)
        
        # ✅ 제어 노드의 진실 구독
        self.create_subscription(PoseStamped, '/ppc/lookahead_point', self.lookahead_callback, 10)
        self.create_subscription(String, '/ppc/state', self.state_callback, 10)
        self.create_subscription(Int32, '/ppc/lookahead_idx', self.idx_callback, 10)
        self.create_subscription(Int32, '/ppc/waypoint_idx', self.waypoint_idx_callback, 10)  # ✅ 추가
        self.create_subscription(Float64, '/ppc/heading_error', self.heading_error_callback, 10)
        self.create_subscription(Float64, '/ppc/cte', self.cte_callback, 10)
        
        # ========== Data Storage ==========
        self.waypoints = []
        self.actual_path = deque()  # ✅ maxlen 제거 (전체 경로 저장)
        self.current_pose = None
        self.current_yaw = 0.0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.is_enabled = False
        
        # ✅ 제어 노드에서 받은 진실
        self.lookahead_point = None
        self.current_state = "move"
        self.current_idx = 0  # interpolated waypoints 인덱스
        self.next_waypoint_idx = 0  # ✅ 추가: 원본 waypoints 인덱스
        
        # 시간별 데이터
        self.time_stamps = deque()
        self.cte_values = deque()
        self.heading_errors = deque()
        self.start_time = None
        self.run_start_time = None
        
        # ========== TTR (Time to Recovery) 관련 ==========
        # 임계값 설정
        self.CTE_OUT = 0.20      # 이탈 임계값 (m)
        self.CTE_IN = 0.08       # 복귀 임계값 (m)
        self.HE_IN = 12.0        # 헤딩 오차 복귀 임계값 (degrees)
        self.T_HOLD = 0.5        # 최소 유지 시간 (s)
        self.TTR_TIMEOUT = 10.0  # 회복 타임아웃 (s)
        self.TTR_COOLDOWN = 0.5  # 연속 이벤트 방지 쿨다운 (s)
        
        # 상태 변수
        self.ttr_state = "normal"  # "normal", "off track", "recovering"
        self.t_out = None          # 이탈 시작 시간
        self.recovery_start = None # 복귀 조건 시작 시간
        self.last_ttr_event = None # 마지막 TTR 이벤트 시간 (쿨다운용)
        
        # TTR 기록
        self.ttr_events = []  # [(t_out, t_in, TTR, success), ...]
        
        # 이동평균 필터 (센서 노이즈 제거)
        self.cte_filter = deque(maxlen=3)     # 3샘플 이동평균
        self.he_filter = deque(maxlen=3)      # 3샘플 이동평균
        
        # Plot 설정
        self.setup_plot()
        
        self.get_logger().info('PlotPPC node initialized (Subscribe-only mode)')
    
    # ========== Callbacks ==========
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
        
        if self.is_enabled:
            self.actual_path.append((x, y))
            
            if self.start_time is None:
                self.start_time = self.get_clock().now()
    
    def cmd_vel_callback(self, msg: Twist):
        """속도 명령 수신"""
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
    
    def enable_callback(self, msg: Bool):
        """PPC 활성화 상태 수신"""
        if msg.data and not self.is_enabled:
            self.actual_path.clear()
            self.time_stamps.clear()
            self.cte_values.clear()
            self.heading_errors.clear()
            self.start_time = None
            self.run_start_time = self.get_clock().now()
            
            # ✅ TTR 상태 리셋
            self.ttr_state = "normal"
            self.t_out = None
            self.recovery_start = None
            self.last_ttr_event = None
            self.ttr_events.clear()
            self.cte_filter.clear()
            self.he_filter.clear()
            
            self.get_logger().info('PPC Enabled - Recording started')
        elif not msg.data and self.is_enabled:
            self.save_run_data()
        
        self.is_enabled = msg.data
    
    # ✅ 제어 노드 진실 수신
    def lookahead_callback(self, msg: PoseStamped):
        """Lookahead Point 수신 (제어 노드의 실제 목표점)"""
        self.lookahead_point = (msg.pose.position.x, msg.pose.position.y)
    
    def state_callback(self, msg: String):
        """State 수신 (move/rotate)"""
        self.current_state = msg.data
    
    def idx_callback(self, msg: Int32):
        """Lookahead Index 수신 (interpolated waypoints 기준)"""
        self.current_idx = msg.data

    def waypoint_idx_callback(self, msg: Int32):
        """✅ 추가: Waypoint Index 수신 (원본 waypoints 기준)"""
        self.next_waypoint_idx = msg.data
    
    def heading_error_callback(self, msg: Float64):
        """Heading Error 수신 (degrees)"""
        he_value = msg.data
        
        # 이동평균 필터 적용
        self.he_filter.append(he_value)
        he_filtered = sum(self.he_filter) / len(self.he_filter)
        
        self.heading_errors.append(he_filtered)
        
        # ✅ time_stamp도 함께 기록
        if self.is_enabled and self.start_time is not None:
            now = self.get_clock().now()
            elapsed = (now - self.start_time).nanoseconds * 1e-9
            self.time_stamps.append(elapsed)
            
            # ✅ TTR 계산 (MOVE 상태에서만)
            if self.current_state == "move":
                self._update_ttr(elapsed, he_filtered)

    def cte_callback(self, msg: Float64):
        """Cross-Track Error 수신 (meters)"""
        cte_value = msg.data
        
        # 이동평균 필터 적용
        self.cte_filter.append(cte_value)
        cte_filtered = sum(self.cte_filter) / len(self.cte_filter)
        
        self.cte_values.append(cte_filtered)
    
    # ========== Plot Setup ==========
    def setup_plot(self):
        """Matplotlib Figure 설정"""
        self.fig = plt.figure(figsize=(18, 8))
        gs = self.fig.add_gridspec(3, 2, width_ratios=[3, 1], 
                                  hspace=0.45, wspace=0.3)
        
        # 1. 경로 플롯 (왼쪽 전체)
        self.ax_path = self.fig.add_subplot(gs[:, 0])
        self.ax_path.set_title('Pure Pursuit Trajectory', fontsize=14, fontweight='bold')
        self.ax_path.set_xlabel('X (m)', fontsize=11)
        self.ax_path.set_ylabel('Y (m)', fontsize=11)
        self.ax_path.grid(True, alpha=0.3)
        # ✅ 초기값 설정 (데이터 수신 후 자동 조정)
        self.ax_path.set_xlim(0, 0)
        self.ax_path.set_ylim(0, 0)
        self.ax_path.set_aspect('equal', adjustable='box')
        
        # ✅ 2. Control Info (우상단)
        self.ax_info = self.fig.add_subplot(gs[0, 1])   
        self.ax_info.set_title('Control Info', fontsize=11, fontweight='bold', pad=10)
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.05, 0.5, 'Waiting...', fontsize=9,
                                            verticalalignment='center', 
                                            family='monospace',
                                            linespacing=1.3)
    
        # ✅ 3. Cross-Track Error (우 가운데)
        self.ax_cte = self.fig.add_subplot(gs[1, 1])
        self.ax_cte.set_title('Cross-Track Error', fontsize=11, fontweight='bold', pad=10)
        self.ax_cte.set_xlabel('Time (s)', fontsize=9)
        self.ax_cte.set_ylabel('CTE (m)', fontsize=9)
        self.ax_cte.grid(True, alpha=0.3)
        self.ax_cte.tick_params(labelsize=8)
    
        # ✅ 4. Heading Error (우하단)
        self.ax_heading = self.fig.add_subplot(gs[2, 1])
        self.ax_heading.set_title('Heading Error', fontsize=11, fontweight='bold', pad=10)
        self.ax_heading.set_xlabel('Time (s)', fontsize=9)
        self.ax_heading.set_ylabel('Error (deg)', fontsize=9)
        self.ax_heading.grid(True, alpha=0.3)
        self.ax_heading.tick_params(labelsize=8)
        
        # Plot 요소 초기화
        self.waypoints_line, = self.ax_path.plot([], [], 'r--', linewidth=2, 
                                                  label='Waypoints', marker='o', markersize=6)
        self.actual_line, = self.ax_path.plot([], [], 'b-', linewidth=2, 
                                               label='Actual Path', alpha=0.7)
        self.robot_marker, = self.ax_path.plot([], [], 'go', markersize=10, label='Robot')
        self.robot_arrow = None
        
        self.lookahead_marker, = self.ax_path.plot([], [], 'co', markersize=10, 
                                                    label='Lookahead', zorder=5)
        self.lookahead_line, = self.ax_path.plot([], [], 'c--', linewidth=2, alpha=0.5)
        
        self.cte_line, = self.ax_cte.plot([], [], 'r-', linewidth=2)
        self.heading_line, = self.ax_heading.plot([], [], 'b-', linewidth=2)
        
        self.ax_path.legend(loc='upper right', fontsize=8)
        
        # 애니메이션 설정
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, 
                                blit=False, cache_frame_data=False)
    
    def update_plot(self, frame):
        """Plot 업데이트 (100ms마다 호출)"""
        
        # 1. Waypoints
        if self.waypoints:
            wp_x = [wp[0] for wp in self.waypoints]
            wp_y = [wp[1] for wp in self.waypoints]
            self.waypoints_line.set_data(wp_x, wp_y)
        
        # 2. Actual Path
        if self.actual_path:
            actual_path_copy = list(self.actual_path)
            if actual_path_copy:
                path_x = [p[0] for p in actual_path_copy]
                path_y = [p[1] for p in actual_path_copy]
                self.actual_line.set_data(path_x, path_y)
    
        # ✅ 3. 자동 범위 조정 (Waypoints + Actual Path 기반)
        self._auto_adjust_limits()
        
        # 4. Robot
        if self.current_pose:
            x, y = self.current_pose
            yaw = self.current_yaw
            self.robot_marker.set_data([x], [y])
            
            if self.robot_arrow:
                self.robot_arrow.remove()
            
            arrow_length = 0.5
            dx = arrow_length * np.cos(yaw)
            dy = arrow_length * np.sin(yaw)
            self.robot_arrow = self.ax_path.arrow(x, y, dx, dy, 
                                                  head_width=0.3, head_length=0.2,
                                                  fc='green', ec='green', alpha=0.7)
        
        # ✅ 4. Lookahead Point (제어 노드의 진실)
        if self.lookahead_point and self.current_pose:
            lx, ly = self.lookahead_point
            rx, ry = self.current_pose
            
            self.lookahead_marker.set_data([lx], [ly])
            self.lookahead_line.set_data([rx, lx], [ry, ly])
            
            ld = math.hypot(lx - rx, ly - ry)
            
            next_wp_x, next_wp_y = self.waypoints[self.next_waypoint_idx] if self.next_waypoint_idx < len(self.waypoints) else (0, 0)

            # ✅ Info 패널 업데이트 (표 형태)
            info_lines = [
                '━' * 37,
                f'State:            {self.current_state.upper()}',
                f'Next Waypoint:    {self.next_waypoint_idx}/{len(self.waypoints)-1} ({next_wp_x:.2f}, {next_wp_y:.2f})',
                f'Current Yaw:      {math.degrees(self.current_yaw):.1f}°',
                '━' * 37,
                f'Lookahead Index:  {self.current_idx}',
                f'Lookahead Dist:   {ld:.2f} m',
                f'Target Point:     ({lx:.2f}, {ly:.2f})',
                '━' * 37,
            ]
            self.info_text.set_text('\n'.join(info_lines))
        else:
            self.lookahead_marker.set_data([], [])
            self.lookahead_line.set_data([], [])
            if not self.is_enabled:
                self.info_text.set_text('Waiting for PPC...')
        
        # 5. CTE
        cte_values_copy = list(self.cte_values)
        time_stamps_copy = list(self.time_stamps)
        
        if cte_values_copy and time_stamps_copy:
            self.cte_line.set_data(time_stamps_copy, cte_values_copy)
            if len(time_stamps_copy) > 0:
                t_max = time_stamps_copy[-1]
                t_min = max(0, t_max - 30)
                if t_max - t_min < 5:
                    t_max = t_min + 5
                self.ax_cte.set_xlim(t_min, t_max + 1)

                if cte_values_copy:
                    cte_max = max(max(cte_values_copy), 0.1)
                    self.ax_cte.set_ylim(0, cte_max * 1.2)
        
        # 6. Heading Error
        heading_errors_copy = list(self.heading_errors)

        if heading_errors_copy and time_stamps_copy:
            self.heading_line.set_data(time_stamps_copy, heading_errors_copy)
            if len(time_stamps_copy) > 0:
                t_max = time_stamps_copy[-1]
                t_min = max(0, t_max - 30)
                if t_max - t_min < 5:
                    t_max = t_min + 5
                self.ax_heading.set_xlim(t_min, t_max + 1)

                if heading_errors_copy:
                    h_max = max(abs(min(heading_errors_copy)), abs(max(heading_errors_copy)))
                    self.ax_heading.set_ylim(-h_max * 1.2, h_max * 1.2)
        
        # 7. Title
        status_text = f"PPC: {'ON' if self.is_enabled else 'OFF'} | State: {self.current_state.upper()}"
        if cte_values_copy:
            status_text += f" | CTE={cte_values_copy[-1]:.3f} m"
        if heading_errors_copy:
            status_text += f" | Heading Err={heading_errors_copy[-1]:.1f}°"

        self.fig.suptitle(status_text, fontsize=14, fontweight='bold',
                         color='green' if self.is_enabled else 'red')
        
        return [self.waypoints_line, self.actual_line, self.robot_marker,
                self.lookahead_marker, self.lookahead_line,
                self.cte_line, self.heading_line]
    
    def save_run_data(self):
        """주행 데이터 JSON으로 저장"""
        if not self.actual_path:
            self.get_logger().warn('No path data to save')
            return
        
        try:
            data_dir = os.path.expanduser('~/ppc_run_data')
            os.makedirs(data_dir, exist_ok=True)
            
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(data_dir, f'ppc_run_{timestamp}.json')
            
            # ✅ TTR 통계 계산
            ttr_stats = self._calculate_ttr_statistics()
            
            data = {
                'timestamp': timestamp,
                'waypoints': self.waypoints,
                'actual_path': list(self.actual_path),
                'cte_values': list(self.cte_values),
                'heading_errors': list(self.heading_errors),
                'avg_cte': np.mean(self.cte_values) if self.cte_values else 0.0,
                'max_cte': max(self.cte_values) if self.cte_values else 0.0,
                'total_points': len(self.actual_path),
                'run_duration': (self.get_clock().now() - self.run_start_time).nanoseconds * 1e-9 
                               if self.run_start_time else 0.0,
                # ✅ TTR 데이터 추가
                'ttr_events': self.ttr_events,
                'ttr_statistics': ttr_stats
            }
            
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            self.get_logger().info(f'Run data saved: {filename}')
            
            # ✅ TTR 통계 로그 출력
            if ttr_stats['total_events'] > 0:
                self.get_logger().info(
                    f"[TTR Summary] Total: {ttr_stats['total_events']}, "
                    f"Success: {ttr_stats['successful_recoveries']}, "
                    f"Failed: {ttr_stats['failed_recoveries']}, "
                    f"Avg TTR: {ttr_stats['avg_ttr']:.2f}s, "
                    f"Max TTR: {ttr_stats['max_ttr']:.2f}s"
                )
            
            
        except Exception as e:
            self.get_logger().error(f'Failed to save run data: {e}')
    
    def _auto_adjust_limits(self):
        """✅ 새 함수: 좌표 범위 자동 조정"""
        all_x = []
        all_y = []
        
        # Waypoints 좌표 수집
        if self.waypoints:
            all_x.extend([wp[0] for wp in self.waypoints])
            all_y.extend([wp[1] for wp in self.waypoints])
        
        # Actual Path 좌표 수집
        if self.actual_path:
            actual_path_copy = list(self.actual_path)
            all_x.extend([p[0] for p in actual_path_copy])
            all_y.extend([p[1] for p in actual_path_copy])
        
        # 좌표가 있으면 범위 계산
        if all_x and all_y:
            x_min, x_max = min(all_x), max(all_x)
            y_min, y_max = min(all_y), max(all_y)
            
            # 여유 공간 추가 (10%)
            x_margin = (x_max - x_min) * 0.1 or 1.0  # 최소 1m
            y_margin = (y_max - y_min) * 0.1 or 1.0
            
            x_min -= x_margin
            x_max += x_margin
            y_min -= y_margin
            y_max += y_margin
            
            # 현재 범위와 비교하여 변경이 필요한 경우에만 업데이트
            current_xlim = self.ax_path.get_xlim()
            current_ylim = self.ax_path.get_ylim()
            
            # ✅ 부드러운 범위 확장 (축소는 하지 않음)
            new_x_min = min(x_min, current_xlim[0])
            new_x_max = max(x_max, current_xlim[1])
            new_y_min = min(y_min, current_ylim[0])
            new_y_max = max(y_max, current_ylim[1])
            
            self.ax_path.set_xlim(new_x_min, new_x_max)
            self.ax_path.set_ylim(new_y_min, new_y_max)
    
    def _update_ttr(self, current_time, heading_error):
        """
        TTR (Time to Recovery) 계산 및 기록
        
        Args:
            current_time: 현재 경과 시간 (s)
            heading_error: 현재 헤딩 오차 (degrees, 필터링된 값)
        """
        # CTE는 이미 필터링되어 cte_values에 저장됨
        if not self.cte_values:
            return
        
        current_cte = self.cte_values[-1]
        
        # 쿨다운 체크
        if self.last_ttr_event is not None:
            if current_time - self.last_ttr_event < self.TTR_COOLDOWN:
                return
        
        # ========== 상태 머신 ==========
        
        if self.ttr_state == "normal":
            # 이탈 감지
            if current_cte > self.CTE_OUT:
                self.ttr_state = "off track"
                self.t_out = current_time
                self.recovery_start = None
                self.get_logger().info(
                    f'[TTR] off track detected at t={current_time:.2f}s, CTE={current_cte:.3f}m'
                )
        
        elif self.ttr_state == "off track":
            # 타임아웃 체크
            if current_time - self.t_out > self.TTR_TIMEOUT:
                self.get_logger().warn(
                    f'[TTR] Recovery FAILED - Timeout after {self.TTR_TIMEOUT}s'
                )
                self.ttr_events.append({
                    't_out': self.t_out,
                    't_in': None,
                    'TTR': None,
                    'success': False,
                    'reason': 'timeout'
                })
                # 상태 리셋
                self.ttr_state = "normal"
                self.t_out = None
                self.recovery_start = None
                self.last_ttr_event = current_time
                return
            
            # 복귀 조건 체크
            if current_cte <= self.CTE_IN and abs(heading_error) <= self.HE_IN:
                if self.recovery_start is None:
                    # 복귀 조건 시작
                    self.recovery_start = current_time
                    self.ttr_state = "recovering"
                    self.get_logger().info(
                        f'[TTR] Recovery started at t={current_time:.2f}s, '
                        f'CTE={current_cte:.3f}m, HE={heading_error:.1f}°'
                    )
        
        elif self.ttr_state == "recovering":
            # 복귀 조건 위반 체크
            if current_cte > self.CTE_IN or abs(heading_error) > self.HE_IN:
                self.get_logger().info(
                    f'[TTR] Recovery interrupted at t={current_time:.2f}s, '
                    f'CTE={current_cte:.3f}m, HE={heading_error:.1f}° - Resetting hold timer'
                )
                self.recovery_start = None
                self.ttr_state = "off track"
                return
            
            # 유지 시간 체크
            hold_duration = current_time - self.recovery_start
            if hold_duration >= self.T_HOLD:
                # 성공!
                t_in = current_time
                ttr = t_in - self.t_out
                
                self.get_logger().info(
                    f'[TTR] Recovery SUCCESS - TTR={ttr:.2f}s '
                    f'(t_out={self.t_out:.2f}s, t_in={t_in:.2f}s)'
                )
                
                self.ttr_events.append({
                    't_out': self.t_out,
                    't_in': t_in,
                    'TTR': ttr,
                    'success': True,
                    'cte_max': max([v for v in self.cte_values if v is not None], default=0),
                    'hold_duration': hold_duration
                })
                
                # 상태 리셋
                self.ttr_state = "normal"
                self.t_out = None
                self.recovery_start = None
                self.last_ttr_event = current_time
    
    def _calculate_ttr_statistics(self):
        """TTR 이벤트 통계 계산"""
        if not self.ttr_events:
            return {
                'total_events': 0,
                'successful_recoveries': 0,
                'failed_recoveries': 0,
                'avg_ttr': 0.0,
                'max_ttr': 0.0,
                'min_ttr': 0.0
            }
        
        successful = [e for e in self.ttr_events if e['success']]
        failed = [e for e in self.ttr_events if not e['success']]
        
        ttr_values = [e['TTR'] for e in successful if e['TTR'] is not None]
        
        return {
            'total_events': len(self.ttr_events),
            'successful_recoveries': len(successful),
            'failed_recoveries': len(failed),
            'avg_ttr': np.mean(ttr_values) if ttr_values else 0.0,
            'max_ttr': max(ttr_values) if ttr_values else 0.0,
            'min_ttr': min(ttr_values) if ttr_values else 0.0
        }


def main(args=None):
    rclpy.init(args=args)
    plotter = PlotPPC()
    
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(plotter,), daemon=True)
    spin_thread.start()
    
    plt.show()
    
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()