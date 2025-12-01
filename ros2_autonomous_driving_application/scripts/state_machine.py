#!/usr/bin/env python3
"""State Machine: Execution of States"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from datetime import datetime
import os
import json

class StateMachineExecutor(Node):
    def __init__(self):
        super().__init__('state_machine_executor_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.get_logger().info('State Machine Executor has been started.')

        # Initial State
        self.current_state = 'INIT'

        # Publishers
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel_stop', 1)
        self.ppc_enable_pub = self.create_publisher(Bool, '/ppc/enable', 5)
        self.ppc_enable_pub.publish(Bool(data=False))
        self.ppc_enabled = False
        
        # Zero Twist for stopping
        self.zero_twist = Twist()
        self.zero_twist.linear.x = 0.0
        self.zero_twist.linear.y = 0.0
        self.zero_twist.linear.z = 0.0
        self.zero_twist.angular.x = 0.0
        self.zero_twist.angular.y = 0.0
        self.zero_twist.angular.z = 0.0

        # Subscriptions
        self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.create_subscription(Bool, '/person_detected', self.person_detected_callback, 10)
        self.person_detected = False

        # Calibration and Align Variables

        # Calibration I/O
        self.cal_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_pub = self.create_publisher(String, '/state_command', 10)
        self.imu_offset_pub = self.create_publisher(Float64, '/yaw_offset', 10)
        self.yaw_offset_est = 0.0
        
        # UWB + IMU for CAL
        self.uwb_sub = self.create_subscription(PoseWithCovarianceStamped, '/abs_xy', self.uwb_callback, 10)
        self.imu_cal_sub = self.create_subscription(Imu, '/imu_cal', self.imu_cal_callback, 10)
        
        # EKF for Align
        self.odom_sub = self.create_subscription(Odometry, '/odometry/ekf_single', self.odom_callback, 10)

        self.cal_state = 'IDLE'
        self.current_ekf = None
        
        # CAL Data Storage
        self.cal_uwb_points = []  # [(x, y), ...] UWB position data
        self.cal_imu_yaws = []    # [yaw, ...] IMU Yaw data
        
        # CAL Repetition Indices
        self.cal_rep_indices = []  # [(forward_start, forward_end, backward_end), ...]
        self.cal_rep_start_idx = 0  # Start index of the current repetition
        
        # CAL Time
        self.cal_start_time = None
        
        # CAL Time Compensation for Pauses When Person Detected
        self.cal_pause_start_time = None  # Pause start time
        self.cal_total_paused_duration = 0.0  # Total paused duration
        
        # CAL Parameters
        self.forward_time = 10.0
        self.backward_time = 10.0
        self.forward_speed = 0.30
        self.backward_speed = -0.30
        self.num_repetitions = 3  # ★ Number of forward-backward repetitions (modifiable)
        self.calculated_yaw_error = 0.0
        self.max_yaw_correction_deg = 360.0 # No limitation for correnction
        
        # CAL Repetition Counter
        self.cal_current_rep = 0
        
        # CAL Data Logging
        self.cal_data_dir = os.path.expanduser('~/calibration_data')
        os.makedirs(self.cal_data_dir, exist_ok=True)
        self.cal_session_data = None
        
        # ALIGN I/O
        self.align_cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initial ALIGN State
        self.align_state = 'IDLE'
        
        # ALIGN Positions
        self.align_current_x = 0.0
        self.align_current_y = 0.0
        self.align_current_yaw = 0.0
        self.start_pos_for_dot_product = None

        # ALIGN Parameters
        self.align_target_pos = (3.0, 6.0) # Target position (x, y)
        self.align_target_yaw = math.pi * 0.5 # Target yaw (rad)
        self.align_linear_speed = 0.3
        self.align_angular_speed = 0.2
        self.align_yaw_threshold = 0.01 # Approximately 0.57 degrees
        
        # Log output control (for change detection)
        self.last_log_uwb_count = 0
        self.last_log_imu_count = 0
        self.last_log_align_state = None
        self.last_log_align_distance = None
        self.last_log_align_yaw_error = None
        
        # Control Frequency
        self.publish_timer = self.create_timer(0.1, self.state_machine_loop)

    # State Machine Loop
    def state_machine_loop(self):
        """Loop executed every 0.1 seconds"""
        # PPC enable publishing (at 1 Hz)
        if hasattr(self, '_loop_counter'):
            self._loop_counter += 1
        else:
            self._loop_counter = 0
        
        if self._loop_counter % 10 == 0:  # Every 1 second
            if self.ppc_enabled and not self.person_detected:
                self.ppc_enable_pub.publish(Bool(data=True))
            else:
                self.ppc_enable_pub.publish(Bool(data=False))
        
        # Checkpoint: Pause when person detected
        if self.person_detected:
            # Immediate stop in all states (state is maintained)
            self.stop_pub.publish(self.zero_twist)
            
            # PPC pause (progress is maintained but movement stops)
            if self.current_state == 'RUN':
                self.ppc_enable_pub.publish(Bool(data=False))
            
            return  # Skip subsequent logic
        
        # Normal operation only when no person detected
        # Reactivate PPC in RUN state
        if self.current_state == 'RUN':
            self.ppc_enable_pub.publish(Bool(data=True))
        
        if self.current_state == 'STOP':
            self.stop_pub.publish(self.zero_twist)

        elif self.current_state == 'CALIBRATION':
            self.run_calibration_step()

        elif self.current_state == 'ALIGN':
            self.align_callback()

    # State Transition
    def state_callback(self, msg):
        """Detect state change and perform entry/exit actions"""
        new_state = msg.data
        if new_state == self.current_state:
            return

        old_state = self.current_state
        self.current_state = new_state
        self.get_logger().info(f'[STATE_MACHINE] Changed: {old_state} -> {new_state}')

        # Exit logic
        # Stop command after task completion
        if old_state == 'RUN':
            self.ppc_enabled = False
            self.get_logger().info('[STATE_MACHINE] RUN: PPC disabled')
        
        if old_state == 'CALIBRATION':
            self.cal_state = 'IDLE'
            self.cal_pub.publish(self.zero_twist)
            self.get_logger().info('[STATE_MACHINE] CALIBRATION: Motors stopped')

        if old_state == 'ALIGN':
            self.align_state = 'IDLE'
            self.align_cmd_vel_pub.publish(self.zero_twist)
            self.get_logger().info('[STATE_MACHINE] ALIGN: Motors stopped')

        # Entry logic
        if new_state == 'STOP':
            self.stop_pub.publish(self.zero_twist)
            self.get_logger().info('[STATE_MACHINE] STOP: Emergency Stop')

        elif new_state == 'RUN':
            self.ppc_enabled = True
            self.get_logger().info('[STATE_MACHINE] RUN: PPC enabled')

        elif new_state == 'KEY':
            self.get_logger().info('[STATE_MACHINE] KEY: Manual mode')

        elif new_state == 'CALIBRATION':
            self.cal_uwb_points = []
            self.cal_imu_yaws = []
            self.cal_rep_indices = []  # Repetition segment indices initialization
            self.cal_rep_start_idx = 0  # Start index initialization
            self.calculated_yaw_error = 0.0
            self.cal_state = 'FORWARD'
            self.cal_start_time = self.get_clock().now()
            
            # Time compensation variable initialization
            self.cal_pause_start_time = None
            self.cal_total_paused_duration = 0.0
            
            # Repetition counter initialization
            self.cal_current_rep = 0
            
            # Session data initialization
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.cal_session_data = {
                'timestamp': timestamp,
                'uwb_points': [],  # [(x, y), ...]
                'imu_yaws': [],    # [yaw, ...]
                'parameters': {
                    'forward_time': self.forward_time,
                    'backward_time': self.backward_time,
                    'forward_speed': self.forward_speed,
                    'backward_speed': self.backward_speed,
                    'num_repetitions': self.num_repetitions  # ★ Parameter recording
                },
                'results': {}
            }

            self.get_logger().info(
                f'[STATE_MACHINE] CALIBRATION: Started ({self.num_repetitions} repetitions)'
            )

        elif new_state == 'ALIGN':
            self.align_state = 'ALIGN_ROTATE_1'
            self.start_pos_for_dot_product = None
            self.get_logger().info('[STATE_MACHINE] ALIGN: Started')

    # Person Detected
    def person_detected_callback(self, msg: Bool):
        """Update person detection status - log only on state change"""
        was_detected = self.person_detected
        self.person_detected = msg.data
        
        # Log and time compensation only on state change
        if not was_detected and self.person_detected:
            # Person Detected: Pause Start
            self.get_logger().warn('[SAFETY] Person DETECTED - PAUSING all operations')
            
            if self.current_state == 'RUN':
                self.ppc_enable_pub.publish(Bool(data=False))
            
            # If in CALIBRATION, pause the stopwatch
            if self.current_state == 'CALIBRATION' and self.cal_state in ['FORWARD', 'BACKWARD']:
                self.cal_pause_start_time = self.get_clock().now()
                self.get_logger().info('[CAL] Stopwatch PAUSED')
                
        elif was_detected and not self.person_detected:
            # Person Cleared: Resume
            self.get_logger().info('[SAFETY] Person CLEARED - RESUMING operations')
            
            if self.current_state == 'RUN':
                self.ppc_enable_pub.publish(Bool(data=True))
            
            # If in CALIBRATION, compensate time (adjust stopwatch time)
            if self.current_state == 'CALIBRATION' and self.cal_pause_start_time is not None:
                now = self.get_clock().now()
                paused_duration = (now - self.cal_pause_start_time).nanoseconds * 1e-9
                self.cal_total_paused_duration += paused_duration
                
                self.get_logger().info(
                    f'[CAL] Stopwatch RESUMED - Paused for {paused_duration:.1f}s '
                    f'(Total paused: {self.cal_total_paused_duration:.1f}s)'
                )
                
                self.cal_pause_start_time = None

    # Sensor & Input 
    def odom_callback(self, msg):
        # Only used in ALIGN
        if self.current_state != 'ALIGN':
            return

        # Save current EKF odometry
        self.current_ekf = msg
    
    def uwb_callback(self, msg: PoseWithCovarianceStamped):
        if self.current_state != 'CALIBRATION':
            return
        
        if self.cal_state not in ['FORWARD', 'BACKWARD']:
            return
        
        # Pause recording if person detected
        if self.person_detected:
            return
        
        # Save only position (ignore timestamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Validity check
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        
        self.cal_uwb_points.append((x, y))
        
        # Log every 10 points, only if count has changed
        if len(self.cal_uwb_points) // 10 > self.last_log_uwb_count:
            self.last_log_uwb_count = len(self.cal_uwb_points) // 10
            self.get_logger().info(
                f'[CAL] UWB data collected: {len(self.cal_uwb_points)} points'
            )
    
    def imu_cal_callback(self, msg: Imu):
        """IMU 보정 결과 수신 (/imu_cal)"""
        if self.current_state != 'CALIBRATION':
            return
        
        if self.cal_state not in ['FORWARD', 'BACKWARD']:
            return
        
        # Pause recording if person detected
        if self.person_detected:
            return
        
        # Save only Yaw
        yaw = self._yaw_from_quat(msg.orientation)
        self.cal_imu_yaws.append(yaw)
        
        # Log every 100 values, only if count has changed
        if len(self.cal_imu_yaws) // 100 > self.last_log_imu_count:
            self.last_log_imu_count = len(self.cal_imu_yaws) // 100
            self.get_logger().info(
                f'[CAL] IMU data collected: {len(self.cal_imu_yaws)} yaw values'
            )

    # Calibration
    def run_calibration_step(self):
        if self.current_state != 'CALIBRATION':
            return
            
        now = self.get_clock().now()
        
        # Actual elapsed time = (current time - start time) - total paused duration
        raw_elapsed = (now - self.cal_start_time).nanoseconds * 1e-9 if self.cal_start_time else 0.0
        dt = raw_elapsed - self.cal_total_paused_duration

        if self.cal_state == 'FORWARD':
            tw = Twist()
            tw.linear.x = self.forward_speed
            self.cal_pub.publish(tw)
            
            if dt >= self.forward_time:
                # Record forward segment end index
                forward_end_idx = len(self.cal_uwb_points) - 1
                
                self.cal_state = 'BACKWARD'
                self.cal_start_time = now
                self.get_logger().info(
                    f'[CAL] Rep {self.cal_current_rep + 1}/{self.num_repetitions}: Forward complete -> Backward\n'
                    f'  Forward range: [{self.cal_rep_start_idx}, {forward_end_idx}] ({forward_end_idx - self.cal_rep_start_idx + 1} points)\n'
                    f'  Total - UWB: {len(self.cal_uwb_points)} points, IMU: {len(self.cal_imu_yaws)} values'
                )
                
                # Temporarily save forward_end_idx (will be finalized after backward)
                self.cal_forward_end_idx = forward_end_idx

        elif self.cal_state == 'BACKWARD':
            tw = Twist()
            tw.linear.x = self.backward_speed
            self.cal_pub.publish(tw)
            
            if dt >= self.backward_time:
                # Record backward segment end index
                backward_end_idx = len(self.cal_uwb_points) - 1
                
                # Save current repetition segment indices
                self.cal_rep_indices.append({
                    'rep_num': self.cal_current_rep + 1,
                    'forward_start': self.cal_rep_start_idx,
                    'forward_end': self.cal_forward_end_idx,
                    'backward_end': backward_end_idx,
                    'num_forward_points': self.cal_forward_end_idx - self.cal_rep_start_idx + 1,
                    'num_backward_points': backward_end_idx - self.cal_forward_end_idx
                })
                
                # Increase repetition counter
                self.cal_current_rep += 1
                
                # Check if all repetitions are complete
                if self.cal_current_rep >= self.num_repetitions:
                    # All repetitions complete -> Calculating
                    self.cal_state = 'CALCULATING'
                    self.cal_start_time = now
                    self.cal_pub.publish(self.zero_twist)
                    self.get_logger().info(
                        f'[CAL] All {self.num_repetitions} repetitions complete -> Calculating\n'
                        f'  Backward range: [{self.cal_forward_end_idx + 1}, {backward_end_idx}] ({backward_end_idx - self.cal_forward_end_idx} points)\n'
                        f'  Total UWB: {len(self.cal_uwb_points)} points, IMU: {len(self.cal_imu_yaws)} values'
                    )
                else:
                    # Next repetition start -> Return to FORWARD
                    # Set start index for next repetition
                    self.cal_rep_start_idx = backward_end_idx + 1
                    
                    self.cal_state = 'FORWARD'
                    self.cal_start_time = now
                    self.get_logger().info(
                        f'[CAL] Rep {self.cal_current_rep}/{self.num_repetitions}: Backward complete -> Next Forward\n'
                        f'  Backward range: [{self.cal_forward_end_idx + 1}, {backward_end_idx}] ({backward_end_idx - self.cal_forward_end_idx} points)\n'
                        f'  Next rep starts at index {self.cal_rep_start_idx}\n'
                        f'  Total - UWB: {len(self.cal_uwb_points)} points, IMU: {len(self.cal_imu_yaws)} values'
                    )
                
        elif self.cal_state == 'CALCULATING':
            # Check minimum data
            if len(self.cal_uwb_points) < 20:
                self.get_logger().error(f'[CAL] Not enough UWB data: {len(self.cal_uwb_points)} < 20')
                self.cal_state = 'FINISHED'
                self.command_pub.publish(String(data='STOP'))
                self._save_calibration_data(success=False, reason='insufficient_uwb_data')
                return
            
            if len(self.cal_imu_yaws) < 50:
                self.get_logger().error(f'[CAL] Not enough IMU data: {len(self.cal_imu_yaws)} < 50')
                self.cal_state = 'FINISHED'
                self.command_pub.publish(String(data='STOP'))
                self._save_calibration_data(success=False, reason='insufficient_imu_data')
                return
            
            # UWB angle
            # Method: Calculate angle by finding the principal component (direction vector) of the coordinates using PCA
            uwb_array = np.array(self.cal_uwb_points)
            
            if len(uwb_array) < 20:
                self.get_logger().error(f'[CAL] Not enough UWB data: {len(uwb_array)} < 20')
                self.cal_state = 'FINISHED'
                self.command_pub.publish(String(data='STOP'))
                self._save_calibration_data(success=False, reason='insufficient UWB data')
                return
            
            # Separate Forward and Backward segments based on the point with the maximum distance
            start_pt = uwb_array[0]
            distances = np.linalg.norm(uwb_array - start_pt, axis=1)
            max_dist_idx = np.argmax(distances)  # Point with maximum distance (turning point)
            uwb_forward = uwb_array[:max_dist_idx+1]
            
            # Perform PCA on the entire segment
            mean_pt = np.mean(uwb_array, axis=0)
            centered = uwb_array - mean_pt
            
            # Extract principal components using SVD(Singular Value Decomposition)
            U, S, Vt = np.linalg.svd(centered, full_matrices=False)
            pc1 = Vt[0]  # First principal component (direction vector)
            
            # Direction consistency: Adjust to match Forward start->end direction (forward direction)
            end_pt = uwb_forward[-1]  # start_pt is already calculated above
            forward_vec = end_pt - start_pt
            if np.dot(pc1, forward_vec) < 0:
                pc1 = -pc1  # Flip PCA direction to forward direction
            
            # Check forward driving distance (start -> forward end)
            forward_distance = math.hypot(forward_vec[0], forward_vec[1])
            
            # Check backward driving distance (turning point -> end)
            backward_vec = uwb_array[-1] - end_pt
            backward_distance = math.hypot(backward_vec[0], backward_vec[1])
            
            spread_along_pc = np.std(np.dot(centered, pc1))
            
            # UWB precision: Mean absolute perpendicular distance from PCA line (mm)
            residuals = np.cross(centered, pc1)  # Signed residuals
            uwb_precision_mm = np.mean(np.abs(residuals)) * 1000  # In mm
            
            if forward_distance < 0.5:  # Forward driving distance too short
                self.get_logger().error(f'[CAL] Forward distance too short: {forward_distance:.2f}m < 0.5m')
                self.cal_state = 'FINISHED'
                self.command_pub.publish(String(data='STOP'))
                self._save_calibration_data(success=False, reason='short_forward_distance')
                return
            
            # Calculate angle from PCA direction vector (forward direction)
            uwb_angle = math.atan2(pc1[1], pc1[0])  # UWB path angle (rad)
            
            # IMU angle
            # Method: Convert each yaw to a unit vector and calculate the angle of the mean vector
            imu_array = np.array(self.cal_imu_yaws)
            
            # Convert angles to 2D unit vectors: (cos(yaw), sin(yaw))
            imu_vectors = np.column_stack([np.cos(imu_array), np.sin(imu_array)])  # (N, 2)
            
            # Vector mean (mean direction)
            imu_mean = np.mean(imu_vectors, axis=0)
            
            imu_angle = math.atan2(imu_mean[1], imu_mean[0])  # IMU direction angle (rad)
            # Calculate angle from mean vector
            
            # Calculate IMU deviation (standard deviation in angle space)
            angle_diffs = np.array([self._normalize_angle(yaw - imu_angle) for yaw in imu_array])
            imu_std_dev = np.std(angle_diffs)
            
            # Offset Angle
            # Yaw offset = (UWB) - (IMU)
            yaw_offset_angle = self._normalize_angle(uwb_angle - imu_angle)
            
            self.calculated_yaw_error = yaw_offset_angle
            
            # Save results
            if self.cal_session_data is not None:
                self.cal_session_data['results'] = {
                    'num_repetitions_completed': self.cal_current_rep,  # Completed repetitions
                    'repetition_indices': self.cal_rep_indices,  # Indices of each repetition segment
                    'num_uwb_points': len(self.cal_uwb_points),
                    'num_uwb_forward': len(uwb_forward),
                    'max_dist_idx': int(max_dist_idx),
                    'num_imu_yaws': len(self.cal_imu_yaws),
                    'forward_distance_m': float(forward_distance),
                    'backward_distance_m': float(backward_distance),
                    'uwb_angle_deg': float(math.degrees(uwb_angle)),
                    'imu_angle_deg': float(math.degrees(imu_angle)),
                    'yaw_offset_angle_deg': float(math.degrees(yaw_offset_angle)),
                    'uwb_spread_along_pc': float(spread_along_pc),
                    'uwb_precision_mm': float(uwb_precision_mm),
                    'imu_std_dev_deg': float(math.degrees(imu_std_dev)),
                    'start_point': [float(start_pt[0]), float(start_pt[1])],
                    'forward_end_point': [float(end_pt[0]), float(end_pt[1])],
                    'pca_direction_uwb': [float(pc1[0]), float(pc1[1])],
                    'mean_direction_imu': [float(imu_mean[0]), float(imu_mean[1])]
                }
            
            self.get_logger().info(
                f'[CAL] Analysis complete:\n'
                f'  UWB Angle: {math.degrees(uwb_angle):.2f}° (from {len(uwb_array)} total UWB points)\n'
                f'  IMU Angle: {math.degrees(imu_angle):.2f}° (from {len(self.cal_imu_yaws)} IMU values)\n'
                f'  Yaw Offset Angle: {math.degrees(yaw_offset_angle):.2f}°\n'
                f'  Forward distance: {forward_distance:.2f}m, Backward distance: {backward_distance:.2f}m\n'
            )
            self.cal_state = 'CORRECTING'

        elif self.cal_state == 'CORRECTING':
            yaw_err = self.calculated_yaw_error
            if abs(yaw_err) > math.radians(self.max_yaw_correction_deg):
                self.get_logger().warn(f'[CAL] Error too large: {math.degrees(yaw_err):.1f}° > {self.max_yaw_correction_deg}° -> STOP')
                self.command_pub.publish(String(data='STOP'))
                self.cal_state = 'FINISHED'
                self._save_calibration_data(success=False, reason='error_too_large')
                return
            
            # imu_cal = current imu + yaw_err
            self.imu_offset_pub.publish(Float64(data=yaw_err))
            self.yaw_offset_est = self._normalize_angle(self.yaw_offset_est + yaw_err)
            
            self.get_logger().info(
                f'[CAL] Offset published: {math.degrees(yaw_err):.2f}°\n'
                f'  Total offset: {math.degrees(self.yaw_offset_est):.2f}°'
            )
            
            self.cal_pub.publish(self.zero_twist)
            self.command_pub.publish(String(data='STOP'))
            self.cal_state = 'FINISHED'
            self._save_calibration_data(success=True)

    def _save_calibration_data(self, success=True, reason=''):
        """Save calibration data to file"""
        if self.cal_session_data is None:
            return
        
        # Save UWB and IMU data
        self.cal_session_data['uwb_points'] = [
            [float(x), float(y)] for x, y in self.cal_uwb_points
        ]
        self.cal_session_data['imu_yaws'] = [
            float(yaw) for yaw in self.cal_imu_yaws
        ]
        self.cal_session_data['success'] = success
        self.cal_session_data['failure_reason'] = reason
        
        timestamp = self.cal_session_data['timestamp']
        
        # Save as JSON file
        json_file = os.path.join(self.cal_data_dir, f'cal_{timestamp}.json')
        with open(json_file, 'w') as f:
            json.dump(self.cal_session_data, f, indent=2)
        
        # Save as NumPy arrays
        npy_uwb_file = os.path.join(self.cal_data_dir, f'cal_{timestamp}_uwb.npy')
        npy_imu_file = os.path.join(self.cal_data_dir, f'cal_{timestamp}_imu.npy')
        np.save(npy_uwb_file, np.array(self.cal_uwb_points))
        np.save(npy_imu_file, np.array(self.cal_imu_yaws))
        
        self.get_logger().info(f'[CAL] Data saved: {json_file}')
        self.get_logger().info(f'[CAL] UWB data: {npy_uwb_file} ({len(self.cal_uwb_points)} points)')
        self.get_logger().info(f'[CAL] IMU data: {npy_imu_file} ({len(self.cal_imu_yaws)} yaw values)')

    # Align
    # TODO align control
    def align_callback(self):
        # Return if no saved data
        if not hasattr(self, 'current_ekf') or self.current_ekf is None:
            self.align_cmd_vel_pub.publish(self.zero_twist)
            return
        # Calculate using EKF coordinates
        msg = self.current_ekf
        self.align_current_x = msg.pose.pose.position.x
        self.align_current_y = msg.pose.pose.position.y
        self.align_current_yaw = self._yaw_from_quat(msg.pose.pose.orientation)
        twist_msg = Twist()
        
        if self.align_state == 'ALIGN_ROTATE_1':
            target_pos = self.align_target_pos
            angle_to_target = math.atan2(
                target_pos[1] - self.align_current_y,
                target_pos[0] - self.align_current_x)
            yaw_error = self._normalize_angle(angle_to_target - self.align_current_yaw)
            
            # Log only on state change or significant error change
            yaw_error_deg = math.degrees(yaw_error)
            if (self.last_log_align_state != 'ALIGN_ROTATE_1' or 
                self.last_log_align_yaw_error is None or
                abs(yaw_error_deg - self.last_log_align_yaw_error) > 5.0):
                self.get_logger().info(f'[ALIGN] ROTATE_1: Error={yaw_error_deg:.1f}°')
                self.last_log_align_state = 'ALIGN_ROTATE_1'
                self.last_log_align_yaw_error = yaw_error_deg
            
            if abs(yaw_error) < self.align_yaw_threshold:
                self.align_state = 'ALIGN_FORWARD'
                self.start_pos_for_dot_product = (self.align_current_x, self.align_current_y)
                self.get_logger().info('[ALIGN] Rotation complete -> Forward')
                self.last_log_align_state = None
            else:
                twist_msg.angular.z = np.clip(yaw_error, -self.align_angular_speed, self.align_angular_speed)

        elif self.align_state == 'ALIGN_FORWARD':
            target_pos = self.align_target_pos
            prev_wp = np.array(self.start_pos_for_dot_product)
            curr_wp = np.array(target_pos)
            robot_pos = np.array([self.align_current_x, self.align_current_y])
            vec_path = curr_wp - prev_wp
            vec_robot = robot_pos - curr_wp

            if np.linalg.norm(vec_path) < 0.01:
                dot = 1.0
            else:
                vec_path_norm = vec_path / np.linalg.norm(vec_path)
                dot = np.dot(vec_path_norm, vec_robot)
            
            distance = np.linalg.norm(robot_pos - curr_wp)
            
            # Log only if distance changes by more than 0.5m
            if (self.last_log_align_state != 'ALIGN_FORWARD' or
                self.last_log_align_distance is None or
                abs(distance - self.last_log_align_distance) > 0.5):
                self.get_logger().info(f'[ALIGN] FORWARD: Distance={distance:.2f}m, Dot={dot:.3f}')
                self.last_log_align_state = 'ALIGN_FORWARD'
                self.last_log_align_distance = distance
            
            if dot > 0.0:
                self.align_state = 'ALIGN_ROTATE_2'
                self.get_logger().info('[ALIGN] Forward complete -> Final rotation')
                self.last_log_align_state = None
            else:
                twist_msg.linear.x = self.align_linear_speed
                
        elif self.align_state == 'ALIGN_ROTATE_2':
            target_yaw = self.align_target_yaw
            yaw_error = self._normalize_angle(target_yaw - self.align_current_yaw)
            
            # Log only on state change or significant error change
            yaw_error_deg = math.degrees(yaw_error)
            if (self.last_log_align_state != 'ALIGN_ROTATE_2' or
                self.last_log_align_yaw_error is None or
                abs(yaw_error_deg - self.last_log_align_yaw_error) > 5.0):
                self.get_logger().info(f'[ALIGN] ROTATE_2: Error={yaw_error_deg:.1f}°')
                self.last_log_align_state = 'ALIGN_ROTATE_2'
                self.last_log_align_yaw_error = yaw_error_deg
            
            if abs(yaw_error) < self.align_yaw_threshold:
                self.align_state = 'Finished'
                self.get_logger().info('[ALIGN] Final rotation complete -> Finished')
                self.last_log_align_state = None
            else:
                twist_msg.angular.z = np.clip(yaw_error, -self.align_angular_speed, self.align_angular_speed)
                
        elif self.align_state == 'Finished':
            self.get_logger().info('[ALIGN] Alignment finished -> STOP')
            self.command_pub.publish(String(data='STOP'))
            self.align_cmd_vel_pub.publish(self.zero_twist)
            return
        
        elif self.align_state == 'IDLE':
            return
            
        self.align_cmd_vel_pub.publish(twist_msg)

    # Utility
    def _normalize_angle(self, a):
        """각도를 -pi ~ pi 범위로 정규화"""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a
        
    def _yaw_from_quat(self, q):
        """쿼터니언에서 Yaw 각도 추출"""
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw
    
def main(args=None):
    rclpy.init(args=args)
    state_machine_executor = StateMachineExecutor()
    rclpy.spin(state_machine_executor)
    state_machine_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()