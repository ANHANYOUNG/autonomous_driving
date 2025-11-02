#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from datetime import datetime
import math
import json
import os


class StateMachineForCalibration(Node):
    def __init__(self):
        super().__init__('state_machine_for_cal_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.get_logger().info('State Machine For Calibration has been started.')

        # ==================== 상태 변수 ====================
        self.current_state = 'INIT'
        self.cal_state = 'IDLE'

        # ==================== Publishers ====================
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel_stop', 10)
        self.cal_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_pub = self.create_publisher(String, '/state_command', 10)
        self.imu_offset_pub = self.create_publisher(Float64, '/yaw_offset', 10)
        
        # PPC Enable (주기적 발행)
        self.ppc_enable_pub = self.create_publisher(Bool, '/ppc/enable', 10)
        self.ppc_enabled = False
        self.ppc_publish_timer = self.create_timer(1.0, self.publish_ppc_enable)

        # ==================== Subscribers ====================
        self.create_subscription(String, '/robot_state', self.state_callback, 10)
        
        # UWB Raw Data 구독
        self.uwb_raw_sub = self.create_subscription(PoseStamped, '/abs_xy', self.uwb_raw_callback, 10)

        # IMU (EKF 통과한 값에서 yaw만 사용)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ekf_single', self.odom_callback, 10)

        # ==================== 정지 명령용 Twist ====================
        self.zero_twist = Twist()
        self.zero_twist.linear.x = 0.0
        self.zero_twist.linear.y = 0.0
        self.zero_twist.linear.z = 0.0
        self.zero_twist.angular.x = 0.0
        self.zero_twist.angular.y = 0.0
        self.zero_twist.angular.z = 0.0

        # ==================== UWB Raw Data 변수 ====================
        self.uwb_x = 0.0
        self.uwb_y = 0.0
        self.uwb_data_received = False

        # ==================== IMU Data 변수 ====================
        self.imu_yaw = 0.0
        self.imu_data_received = False

        # ==================== Calibration 변수 ====================
        self.cal_points = []  # UWB raw 위치 저장
        self.cal_start_time = None
        
        # Calibration Parameters
        self.forward_time = 2.0  # 전진 시간 (초)
        self.backward_time = 2.0  # 후진 시간 (초)
        self.forward_speed = 0.30  # 전진 속도 (m/s)
        self.backward_speed = -0.30  # 후진 속도 (m/s)
        self.max_yaw_correction_deg = 40.0  # 최대 보정 각도 (도)
        
        # Calibration Data Logging
        self.cal_data_dir = os.path.expanduser('~/calibration_data')
        os.makedirs(self.cal_data_dir, exist_ok=True)

        # ==================== State Machine Timer ====================
        # 0.1초마다 상태머신 루프 실행
        self.state_machine_timer = self.create_timer(0.1, self.state_machine_loop)

        self.get_logger().info('[INIT] Waiting for UWB and IMU data...')

    # ==================== PPC Enable 주기 발행 ====================
    def publish_ppc_enable(self):
        """1초마다 PPC enable 상태 발행"""
        self.ppc_enable_pub.publish(Bool(data=self.ppc_enabled))

    # ==================== UWB Raw Data Callback ====================
    def uwb_raw_callback(self, msg):
        """UWB raw position data 수신"""
        self.uwb_x = msg.pose.position.x
        self.uwb_y = msg.pose.position.y
        
        if not self.uwb_data_received:
            self.uwb_data_received = True
            self.get_logger().info(f'[UWB] First data received: x={self.uwb_x:.3f}, y={self.uwb_y:.3f}')

    # ==================== Odometry (IMU Yaw) Callback ====================
    def odom_callback(self, msg):
        """EKF Odometry에서 IMU yaw만 추출"""
        q = msg.pose.pose.orientation
        self.imu_yaw = self._yaw_from_quat(q)
        
        if not self.imu_data_received:
            self.imu_data_received = True
            self.get_logger().info(f'[IMU] First data received: yaw={math.degrees(self.imu_yaw):.2f}°')

    # ==================== State Callback ====================
    def state_callback(self, msg):
        """마스터 상태 변경 감지"""
        new_state = msg.data
        if new_state == self.current_state:
            return

        old_state = self.current_state
        self.current_state = new_state
        self.get_logger().info(f'[STATE] Changed: {old_state} -> {new_state}')

        # === Exit 로직 ===
        if old_state == 'RUN':
            self.ppc_enabled = False
            self.get_logger().info('[EXIT] RUN: PPC disabled')
        
        if old_state == 'CALIBRATION':
            self.cal_state = 'IDLE'
            self.cal_pub.publish(self.zero_twist)
            self.get_logger().info('[EXIT] CALIBRATION: Motors stopped')

        # === Entry 로직 ===
        if new_state == 'STOP':
            self.stop_pub.publish(self.zero_twist)
            self.get_logger().info('[ENTRY] STOP: Emergency stop published')
        
        elif new_state == 'RUN':
            self.ppc_enabled = True
            self.get_logger().info('[ENTRY] RUN: PPC enabled')

        elif new_state == 'KEY':
            self.get_logger().info('[ENTRY] KEY: Manual mode')

        elif new_state == 'CALIBRATION':
            if not self.uwb_data_received or not self.imu_data_received:
                self.get_logger().error('[ENTRY] CALIBRATION: No UWB or IMU data! Aborting.')
                self.command_pub.publish(String(data='STOP'))
                return
            
            self.cal_state = 'FORWARD'
            self.cal_points = []
            self.cal_start_time = self.get_clock().now()
            
            # 전진 시작
            cmd = Twist()
            cmd.linear.x = self.forward_speed
            self.cal_pub.publish(cmd)
            
            self.get_logger().info('[ENTRY] CALIBRATION: Starting forward motion')

    # ==================== State Machine Loop ====================
    def state_machine_loop(self):
        """0.1초마다 실행되는 상태머신 루프"""
        if self.current_state == 'CALIBRATION':
            self.run_calibration_step()

    # ==================== Calibration Step Execution ====================
    def run_calibration_step(self):
        """CALIBRATION 단계별 실행"""
        if self.cal_state == 'IDLE':
            return

        now = self.get_clock().now()
        
        if self.cal_state == 'FORWARD':
            elapsed = (now - self.cal_start_time).nanoseconds / 1e9
            
            # UWB 위치 저장
            if self.uwb_data_received:
                self.cal_points.append({
                    'x': self.uwb_x,
                    'y': self.uwb_y,
                    'time': elapsed,
                    'phase': 'forward'
                })
            
            # 전진 완료
            if elapsed >= self.forward_time:
                self.cal_pub.publish(self.zero_twist)
                self.cal_state = 'BACKWARD'
                self.cal_start_time = now
                
                # 후진 시작
                cmd = Twist()
                cmd.linear.x = self.backward_speed
                self.cal_pub.publish(cmd)
                
                self.get_logger().info(f'[CAL] Forward done ({len([p for p in self.cal_points if p["phase"]=="forward"])} points), starting backward')
        
        elif self.cal_state == 'BACKWARD':
            elapsed = (now - self.cal_start_time).nanoseconds / 1e9
            
            # UWB 위치 계속 저장
            if self.uwb_data_received:
                self.cal_points.append({
                    'x': self.uwb_x,
                    'y': self.uwb_y,
                    'time': self.forward_time + elapsed,
                    'phase': 'backward'
                })
            
            # 후진 완료
            if elapsed >= self.backward_time:
                self.cal_pub.publish(self.zero_twist)
                self.cal_state = 'CALCULATING'
                self.get_logger().info(f'[CAL] Backward done ({len([p for p in self.cal_points if p["phase"]=="backward"])} points), calculating offset')
        
        elif self.cal_state == 'CALCULATING':
            self._calculate_uwb_absolute_yaw()
            self.cal_state = 'IDLE'

    # ==================== UWB Absolute Yaw Calculation ====================
    def _calculate_uwb_absolute_yaw(self):
        """UWB raw data만으로 절대 yaw 계산"""
        forward_points = [p for p in self.cal_points if p['phase'] == 'forward']
        backward_points = [p for p in self.cal_points if p['phase'] == 'backward']
        
        self.get_logger().info(f'[CAL] Total points: {len(self.cal_points)} (forward: {len(forward_points)}, backward: {len(backward_points)})')
        
        if len(forward_points) < 10:
            self.get_logger().error(f'[CAL] Not enough forward points: {len(forward_points)} < 10')
            self._save_calibration_data(success=False, reason='insufficient_forward_data')
            self.command_pub.publish(String(data='STOP'))
            return
        
        if len(backward_points) < 10:
            self.get_logger().error(f'[CAL] Not enough backward points: {len(backward_points)} < 10')
            self._save_calibration_data(success=False, reason='insufficient_backward_data')
            self.command_pub.publish(String(data='STOP'))
            return
        
        # 각 구간의 시작/끝 평균 위치 계산 (노이즈 제거)
        forward_start = self._average_points(forward_points[:5])
        forward_end = self._average_points(forward_points[-5:])
        backward_start = self._average_points(backward_points[:5])
        backward_end = self._average_points(backward_points[-5:])
        
        self.get_logger().info(f'[CAL] Forward: start=({forward_start["x"]:.3f}, {forward_start["y"]:.3f}), end=({forward_end["x"]:.3f}, {forward_end["y"]:.3f})')
        self.get_logger().info(f'[CAL] Backward: start=({backward_start["x"]:.3f}, {backward_start["y"]:.3f}), end=({backward_end["x"]:.3f}, {backward_end["y"]:.3f})')
        
        # 방향 벡터 계산 (UWB 절대 좌표계)
        forward_vec = (
            forward_end['x'] - forward_start['x'],
            forward_end['y'] - forward_start['y']
        )
        backward_vec = (
            backward_end['x'] - backward_start['x'],
            backward_end['y'] - backward_start['y']
        )
        
        forward_dist = math.sqrt(forward_vec[0]**2 + forward_vec[1]**2)
        backward_dist = math.sqrt(backward_vec[0]**2 + backward_vec[1]**2)
        
        self.get_logger().info(f'[CAL] Forward vector: ({forward_vec[0]:.3f}, {forward_vec[1]:.3f}), distance: {forward_dist:.3f}m')
        self.get_logger().info(f'[CAL] Backward vector: ({backward_vec[0]:.3f}, {backward_vec[1]:.3f}), distance: {backward_dist:.3f}m')
        
        # 이동 거리가 너무 작으면 에러
        expected_dist = self.forward_speed * self.forward_time
        if forward_dist < expected_dist * 0.3:  # 기대 거리의 30% 이하
            self.get_logger().error(f'[CAL] Forward distance too small: {forward_dist:.3f}m < {expected_dist*0.3:.3f}m')
            self._save_calibration_data(success=False, reason='forward_distance_too_small')
            self.command_pub.publish(String(data='STOP'))
            return
        
        if backward_dist < expected_dist * 0.3:
            self.get_logger().error(f'[CAL] Backward distance too small: {backward_dist:.3f}m < {expected_dist*0.3:.3f}m')
            self._save_calibration_data(success=False, reason='backward_distance_too_small')
            self.command_pub.publish(String(data='STOP'))
            return
        
        # 절대 yaw 각도 계산 (UWB 좌표계 기준)
        forward_yaw_uwb = math.atan2(forward_vec[1], forward_vec[0])
        backward_yaw_uwb = math.atan2(backward_vec[1], backward_vec[0])
        
        self.get_logger().info(f'[CAL] Forward yaw (UWB): {math.degrees(forward_yaw_uwb):.2f}°')
        self.get_logger().info(f'[CAL] Backward yaw (UWB): {math.degrees(backward_yaw_uwb):.2f}°')
        
        # 전진/후진이 반대 방향인지 확인
        yaw_diff = self._normalize_angle(forward_yaw_uwb - backward_yaw_uwb)
        
        self.get_logger().info(f'[CAL] Forward-Backward yaw difference: {math.degrees(yaw_diff):.2f}°')
        
        if abs(abs(yaw_diff) - math.pi) > math.radians(30.0):  # 30도 이상 차이나면 비정상
            self.get_logger().error(f'[CAL] Forward/Backward not opposite! diff={math.degrees(yaw_diff):.1f}° (expected ~180°)')
            self._save_calibration_data(success=False, reason='direction_mismatch', yaw_diff_deg=math.degrees(yaw_diff))
            self.command_pub.publish(String(data='STOP'))
            return
        
        # 평균 절대 yaw (UWB 좌표계 기준)
        # 전진 방향을 기준으로 사용
        absolute_yaw_uwb = forward_yaw_uwb
        
        # IMU 현재 yaw
        current_imu_yaw = self.imu_yaw
        
        # 보정값 계산: yaw_offset = UWB 절대 yaw - IMU yaw
        # 나중에 보정: corrected_yaw = IMU_yaw + yaw_offset
        yaw_offset = self._normalize_angle(absolute_yaw_uwb - current_imu_yaw)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('[CAL RESULT]')
        self.get_logger().info(f'  UWB absolute yaw: {math.degrees(absolute_yaw_uwb):.2f}°')
        self.get_logger().info(f'  IMU current yaw:  {math.degrees(current_imu_yaw):.2f}°')
        self.get_logger().info(f'  Yaw offset:       {math.degrees(yaw_offset):.2f}°')
        self.get_logger().info('=' * 60)
        
        # 안전성 체크
        if abs(yaw_offset) > math.radians(self.max_yaw_correction_deg):
            self.get_logger().error(f'[CAL] Yaw offset too large: {math.degrees(yaw_offset):.1f}° > {self.max_yaw_correction_deg}°')
            self._save_calibration_data(
                success=False, 
                reason='offset_too_large',
                uwb_absolute_yaw_deg=math.degrees(absolute_yaw_uwb),
                imu_yaw_deg=math.degrees(current_imu_yaw),
                yaw_offset_deg=math.degrees(yaw_offset)
            )
            self.command_pub.publish(String(data='STOP'))
            return
        
        # 보정값 발행
        self.imu_offset_pub.publish(Float64(data=yaw_offset))
        self.get_logger().info(f'[CAL] Published yaw_offset: {math.degrees(yaw_offset):.2f}° to /yaw_offset')
        
        # 데이터 저장
        self._save_calibration_data(
            success=True,
            reason='',
            uwb_absolute_yaw_deg=math.degrees(absolute_yaw_uwb),
            imu_yaw_deg=math.degrees(current_imu_yaw),
            yaw_offset_deg=math.degrees(yaw_offset),
            forward_yaw_uwb_deg=math.degrees(forward_yaw_uwb),
            backward_yaw_uwb_deg=math.degrees(backward_yaw_uwb),
            yaw_diff_deg=math.degrees(yaw_diff),
            forward_vec=forward_vec,
            backward_vec=backward_vec,
            forward_dist=forward_dist,
            backward_dist=backward_dist
        )
        
        self.get_logger().info('[CAL] SUCCESS! Returning to STOP state')
        self.command_pub.publish(String(data='STOP'))

    # ==================== Utility Functions ====================
    def _average_points(self, points):
        """여러 점의 평균 위치 계산 (노이즈 제거용)"""
        n = len(points)
        avg_x = sum(p['x'] for p in points) / n
        avg_y = sum(p['y'] for p in points) / n
        return {'x': avg_x, 'y': avg_y}

    def _normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _yaw_from_quat(self, q):
        """쿼터니언에서 yaw 추출"""
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def _save_calibration_data(self, success=True, reason='', **extra_data):
        """캘리브레이션 데이터 저장"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.cal_data_dir, f'cal_uwb_{timestamp}.json')
        
        data = {
            'timestamp': timestamp,
            'success': success,
            'reason': reason,
            'parameters': {
                'forward_time': self.forward_time,
                'backward_time': self.backward_time,
                'forward_speed': self.forward_speed,
                'backward_speed': self.backward_speed,
                'max_yaw_correction_deg': self.max_yaw_correction_deg
            },
            'points': self.cal_points,
            'num_points': {
                'total': len(self.cal_points),
                'forward': len([p for p in self.cal_points if p['phase'] == 'forward']),
                'backward': len([p for p in self.cal_points if p['phase'] == 'backward'])
            },
            **extra_data
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f'[CAL] Data saved to {filename}')
        except Exception as e:
            self.get_logger().error(f'[CAL] Failed to save data: {e}')


def main(args=None):
    rclpy.init(args=args)
    state_machine_for_cal = StateMachineForCalibration()
    
    try:
        rclpy.spin(state_machine_for_cal)
    except KeyboardInterrupt:
        state_machine_for_cal.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        state_machine_for_cal.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()