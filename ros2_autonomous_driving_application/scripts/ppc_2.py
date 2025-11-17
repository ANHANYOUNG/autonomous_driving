#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from nav_msgs.msg import Odometry, Path
import math
from sensor_msgs.msg import Imu, NavSatFix
import pyproj
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_matrix
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String, Int32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node_2')

        # ========== 파라미터 선언 ==========
        self.declare_parameter('lookahead_distance', 1.5)  # 런치 인자로 주입됨
        
        # Publishers
        
        # 속도
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_ppc', 10)
        # plot 용
        self.lookahead_pub = self.create_publisher(PoseStamped, '/ppc/lookahead_point', 10)
        self.state_pub = self.create_publisher(String, '/ppc/state', 10)
        self.idx_pub = self.create_publisher(Int32, '/ppc/lookahead_idx', 10)
        self.heading_error_pub = self.create_publisher(Float64, '/ppc/heading_error', 10)
        self.cte_pub = self.create_publisher(Float64, '/ppc/cte', 10)
        self.waypoint_idx_pub = self.create_publisher(Int32, '/ppc/waypoint_idx', 10)
        self.waypoints_path_pub = self.create_publisher(Path, '/waypoints_path', 10)
        # 미션 완료 시 state machine에 stop 전송
        self.command_pub = self.create_publisher(String, '/state_command', 10)
        # 센서
        self.absxy_pub = self.create_publisher(PoseWithCovarianceStamped, '/abs_xy', 1)

        # Subscribers
        self.dt_gps = 0.1
        self.gps_sub = self.create_subscription(NavSatFix, '/navsat', self.gps_callback, 1)

        self.dt_imu = 0.01
        self.imu_sub = self.create_subscription(Imu, '/imu_cal', self.imu_callback, 1)

        self.mag_sub = self.create_subscription(MagneticField, '/magnetometer', self.mag_callback, 1)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 1)

        self.dt_timer = 0.25
        self.timer = self.create_timer(self.dt_timer, self.timer_callback)

        # EKF
        self.global_odom_sub = self.create_subscription(Odometry, '/odometry/ekf_single', self.global_odom_callback, 1)
        
        # ppc enable
        self.ppc_enable_sub = self.create_subscription(Bool, '/ppc/enable', self.ppc_enable_callback, 10)
        # 활성화 상태 변수
        self.is_enabled = False 
        # 정지 명령용 Twist
        self.zero_twist = Twist()

        # 예시 경로점
        self.waypoints = [
            (-9,9), (-1,9),
            (-1,5), (-9,5),
            (-9,1), (-1,1)
        ]
        
        # 런치 인자에서 받은 lookahead_distance 적용
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.get_logger().info(f'[RUN] Ld={self.lookahead_distance}m')

        # 현재 목표 waypoint index (원본 waypoints 기준)
        self.current_waypoint_idx = 0  # 처음 시작은 waypoint[0] → waypoint[1]로 향함
        
        # 현재 탐색 중인 경로 세그먼트 인덱스
        self.current_segment_idx = 0

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.global_x = None
        self.global_y = None
        self.global_yaw = None

        # 기준점 위도, 경도 (예시: world의 (0,0)에 해당하는 위경도)
        origin_lat = 37.0
        origin_lon = 127.0

        # WGS84 -> ENU 변환기 생성
        self.enu_converter = pyproj.Transformer.from_crs(
            crs_from="epsg:4326",
            crs_to=pyproj.CRS.from_proj4(
                f"+proj=tmerc +lat_0={origin_lat} +lon_0={origin_lon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84"
            ),
            always_xy=True
        )

        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0
        self.mag_yaw = None  # 자북 방향(heading) 추정값 (rad)

        self.clock_cnt = 0
        self.clock_cnt_pre = 0
        self.current_time = 0.0
        self.current_time_pre = 0.0

        self.rotate_flag = 0
        self.state = "move"  # "move" 또는 "rotate"
        
        # 완료 판정
        self.mission_completed = False

        # clock이 시작되면 1번만 발행하도록 플래그 추가
        # Waypoints를 Path로 발행
        self.publish_waypoints_path()
        self.waypoints_published = False
        
        # Lookahead point (매 순간 갱신)
        self.lookahead_point = None

    # ========== Timer Loop ==========
    def timer_callback(self):
        # 1. 주행 실행 조건 검사
        if not self.check_run():
            return
        
        # 2. Lookahead Point 계산 (매 순간 갱신)
        self.lookahead_point = self.find_lookahead_point()
        
        # 3. 도착 판정 및 상태 갱신
        self.check_arrival()
        
        # 4. 코너 진입 감지
        self.check_rotate()
        
        # 5. 속도 명령 계산
        twist = Twist()
        if self.state == "move":
            twist = self.move_state()
        elif self.state == "rotate":
            twist = self.rotate_state()
        # 6. 속도 명령 발행
        self.publish_cmd_vel(twist)
        
        # 7. 시각화 데이터 발행
        self.publish_for_plot()
    
    # ========== Main Control Logic ==========
    def check_run(self):
        # 비활성화 시 정지
        if not self.is_enabled:
            self.cmd_pub.publish(self.zero_twist)
            return False
        
        # 완료 시 정지
        if self.mission_completed:
            self.cmd_pub.publish(self.zero_twist)
            self.command_pub.publish(String(data="STOP"))
            return False
        
        # EKF 위치 정보가 없으면 종료
        if self.global_x is None or self.global_y is None or self.global_yaw is None:
            return False
        
        return True
    
    def find_lookahead_point(self):
        """
        Classic Pure Pursuit: 로봇 중심 원(반지름 Ld)과 경로 선분의 교점 찾기
        
        Returns:
            (x, y): Lookahead point 좌표, 없으면 마지막 waypoint 반환
        """
        x = self.global_x
        y = self.global_y
        ld = self.lookahead_distance
        
        # 현재 세그먼트부터 탐색 시작
        for i in range(self.current_segment_idx, len(self.waypoints) - 1):
            p1 = np.array(self.waypoints[i])
            p2 = np.array(self.waypoints[i + 1])
            robot_pos = np.array([x, y])
            
            # 선분 p1-p2와 원(중심: robot_pos, 반지름: ld)의 교점 찾기
            intersection = self.circle_line_segment_intersection(robot_pos, ld, p1, p2)
            
            if intersection is not None:
                # 교점 발견 시 세그먼트 인덱스 업데이트
                self.current_segment_idx = i
                return tuple(intersection)
        
        # 교점이 없으면 마지막 waypoint 반환
        return self.waypoints[-1]
    
    def circle_line_segment_intersection(self, center, radius, p1, p2):
        """
        원과 선분의 교점 계산 (Pure Pursuit의 핵심 알고리즘)
        
        Args:
            center: 원의 중심 (로봇 위치) [x, y]
            radius: 원의 반지름 (lookahead_distance)
            p1: 선분의 시작점 [x, y]
            p2: 선분의 끝점 [x, y]
        
        Returns:
            교점 [x, y] 또는 None (교점이 없거나 선분 범위 밖)
        """
        # 선분 벡터
        d = p2 - p1
        # 중심에서 p1으로의 벡터
        f = p1 - center
        
        # 2차 방정식의 계수: at^2 + bt + c = 0
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - radius ** 2
        
        discriminant = b ** 2 - 4 * a * c
        
        # 교점이 없음
        if discriminant < 0:
            return None
        
        # 교점이 있을 경우
        discriminant = math.sqrt(discriminant)
        
        # 두 개의 t 값 (선분 파라미터)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        
        # 선분 범위 [0, 1] 내의 교점 찾기
        # t2를 우선 선택 (경로의 앞쪽 교점)
        if 0 <= t2 <= 1:
            intersection = p1 + t2 * d
            return intersection
        elif 0 <= t1 <= 1:
            intersection = p1 + t1 * d
            return intersection
        
        # 선분 범위 밖
        return None
    
    def check_arrival(self):
        """원본 waypoint 도착 판정 (내적 기반)"""
        x = self.global_x
        y = self.global_y
        
        current_idx = self.current_waypoint_idx
        
        # 마지막 waypoint 도착 판정
        if current_idx >= len(self.waypoints) - 1:
            last_wp = self.waypoints[-1]
            dist = np.hypot(last_wp[0] - x, last_wp[1] - y)
            
            if dist < 0.3:  # 30cm 이내 도착
                self.get_logger().info("[RUN] Reached final waypoint")
                self.mission_completed = True
            return
        
        # 다음 waypoint로의 진행 판정 (내적 기반)
        if current_idx + 1 < len(self.waypoints):
            prev_wp = self.waypoints[current_idx]
            curr_wp = self.waypoints[current_idx + 1]
            robot_pos = np.array([x, y])
            
            vec_path = np.array([curr_wp[0] - prev_wp[0], curr_wp[1] - prev_wp[1]])
            vec_robot = np.array([robot_pos[0] - curr_wp[0], robot_pos[1] - curr_wp[1]])
            dot = np.dot(vec_path, vec_robot)
            
            if dot > 0:  # waypoint 통과
                if self.rotate_flag == 1:
                    self.rotate_flag = 0
                    self.state = "rotate"
                    self.get_logger().info("[RUN] Entering rotate state")
                
                self.current_waypoint_idx += 1
                self.get_logger().info(
                    f'[RUN] Reached waypoint {self.current_waypoint_idx-1}, '
                    f'[RUN] moving to waypoint {self.current_waypoint_idx}'
                )
    
    def check_rotate(self):
        """코너 진입 감지"""
        if self.lookahead_point is None:
            return
        
        # 현재 lookahead_point가 원본 waypoint와 가까우면 회전 플래그 on
        for wp in self.waypoints:
            if np.hypot(self.lookahead_point[0] - wp[0], self.lookahead_point[1] - wp[1]) < 0.1:
                self.rotate_flag = 1
                break
    
    def move_state(self):
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw
        
        if self.lookahead_point is None:
            # Lookahead point가 없으면 정지
            return Twist()
        
        dx = self.lookahead_point[0] - x
        dy = self.lookahead_point[1] - y
        ld = np.hypot(dx, dy)
        
        if ld < 0.01:  # 거의 도착한 경우
            return Twist()
        
        if ld > self.lookahead_distance * 0.25:
            # ========== Pure Pursuit 모드 (부드러움 우선) ==========
            # 차량의 heading 기준 lookahead point의 좌표
            x_r = math.cos(yaw) * dx + math.sin(yaw) * dy
            y_r = -math.sin(yaw) * dx + math.cos(yaw) * dy
            curvature = 2.0 * y_r / (ld ** 2)
        else:
            # ========== 정렬 모드 (정확성 우선) ==========
            # 다음 실제 웨이포인트를 타겟으로 설정
            if self.current_waypoint_idx < len(self.waypoints):
                target_wp = self.waypoints[self.current_waypoint_idx]
            else:
                # 마지막 웨이포인트 사용
                target_wp = self.waypoints[-1]
            
            # 타겟 웨이포인트 방향 계산
            dx_target = target_wp[0] - x
            dy_target = target_wp[1] - y
            ld_target = np.hypot(dx_target, dy_target)
            
            if ld_target > 0.01:  # 거의 도착한 경우 방지
                # 차량 좌표계로 변환
                x_r_target = math.cos(yaw) * dx_target + math.sin(yaw) * dy_target
                y_r_target = -math.sin(yaw) * dx_target + math.cos(yaw) * dy_target
                # 정렬 곡률 계산
                curvature = 2.0 * y_r_target / (ld_target ** 2)
            else:
                curvature = 0.0
        
        # 속도 명령 생성
        twist = Twist()
        if self.rotate_flag == 1:
            twist.linear.x = 0.1
            twist.angular.z = twist.linear.x * curvature
        else:
            twist.linear.x = 0.35
            twist.angular.z = twist.linear.x * curvature
        
        return twist
    
    def rotate_state(self):
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw
        
        # 가장 가까운 waypoint(시작점 제외)를 찾음
        min_wp_dist = float('inf')
        target_yaw = yaw
        for i in range(1, len(self.waypoints)):
            wp = self.waypoints[i]
            dist = np.hypot(wp[0] - x, wp[1] - y)
            if dist < min_wp_dist:
                min_wp_dist = dist
                prev_wp = wp
                if i + 1 < len(self.waypoints):
                    target_wp = self.waypoints[i + 1]
                else:
                    target_wp = wp
        
        # prev_wp → target_wp 방향을 목표 yaw로 사용
        target_yaw = math.atan2(target_wp[1] - prev_wp[1], target_wp[0] - prev_wp[0])
        yaw_error = self.normalize_angle(target_yaw - yaw)
        yaw_error_deg = abs(math.degrees(yaw_error))
        
        twist = Twist()
        if yaw_error_deg < 1.0:
            self.state = "move"
            self.get_logger().info(
                f" [RUN] Yaw aligned (error {yaw_error_deg:.2f} deg). Switching to move state."
            )
        else:
            twist.linear.x = 0.0
            twist.angular.z = np.sign(yaw_error) * (np.pi / 36)  # 10 deg/s, 방향 고려
            self.get_logger().info(
                f"[RUN] Rotating in place. Yaw error: {yaw_error_deg:.2f} current: {math.degrees(yaw):.2f} deg) | "
            )
        
        return twist

    def publish_cmd_vel(self, twist):
        self.cmd_pub.publish(twist)

    # ========== Input ==========
    def ppc_enable_callback(self, msg: Bool):
        # 상태가 변경될 때만 로그 출력
        if self.is_enabled != msg.data:
            if msg.data:
                # 재활성화 시 미션 완료 플래그 리셋
                self.mission_completed = False
                self.current_waypoint_idx = 0
                self.current_segment_idx = 0
                self.state = "move"
                self.get_logger().info('Pure Pursuit [ENABLED]')
            else:
                # 비활성화되는 즉시 0 속도를 1회 발행
                # (twist_mux가 이 토픽을 timeout 처리할 때까지 기다리지 않도록)
                self.cmd_pub.publish(self.zero_twist)
                self.mission_completed = True
                self.get_logger().info('Pure Pursuit [DISABLED]')
                
        self.is_enabled = msg.data

    def gps_callback(self, msg):
        lon = msg.longitude
        lat = msg.latitude
        x, y = self.enu_converter.transform(lon, lat)

        p = PoseWithCovarianceStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'  # ekf.yaml의 world_frame과 일치
        p.pose.pose.position.x = x
        p.pose.pose.position.y = y
        p.pose.pose.position.z = 0.0
        p.pose.pose.orientation.x = 0.0
        p.pose.pose.orientation.y = 0.0
        p.pose.pose.orientation.z = 0.0
        p.pose.pose.orientation.w = 1.0

        # x,y만 신뢰, 나머지는 크게(무시되도록)
        p.pose.covariance = [
            0.04, 0, 0, 0, 0, 0,
            0, 0.04, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e6,
        ]
        self.absxy_pub.publish(p)

    def imu_callback(self, msg):
        # 주석 해제: 시간이 변하면 업데이트
        if self.clock_cnt != self.clock_cnt_pre:
            self.clock_cnt_pre = self.clock_cnt

        # 쿼터니언 -> yaw 변환
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)

    def mag_callback(self, msg: MagneticField):
        # 원시 자기장 값 저장 (Tesla)
        self.mag_x = msg.magnetic_field.x
        self.mag_y = msg.magnetic_field.y
        self.mag_z = msg.magnetic_field.z

        # XY 평면에서 heading 계산
        # atan2(Y, X)으로 구면 좌표계의 방위각(자북 기준, rad)
        self.mag_yaw = math.atan2(self.mag_y, self.mag_x)

        # 지자기 편차(declination) 보정이 필요하면 여기서 더해줌
        declination = math.radians(-8.9)
        self.mag_yaw -= declination
        
    def clock_callback(self, msg):
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.current_time == 0.0:
            self.clock_cnt = 0.0
        elif self.current_time > self.current_time_pre:
            self.clock_cnt += 1.0
            self.current_time_pre = self.current_time
            
            # clock이 시작되면 1번만 waypoint 발행
            if not self.waypoints_published:
                self.publish_waypoints_path()
                self.waypoints_published = True

    def global_odom_callback(self, msg: Odometry):
        self.global_x = msg.pose.pose.position.x
        self.global_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, self.global_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # ========== Utility ==========
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # ========== For Plot ==========
    def publish_waypoints_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        self.waypoints_path_pub.publish(path)
        self.get_logger().info(f'Published {len(self.waypoints)} waypoints to /waypoints_path')
    
    def publish_for_plot(self):
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw
        
        if self.lookahead_point is None:
            return
        
        # CTE 계산 (경로 선분 기준)
        min_dist = float('inf')
        for i in range(len(self.waypoints) - 1):
            x1, y1 = self.waypoints[i]
            x2, y2 = self.waypoints[i + 1]
            
            dx = x2 - x1
            dy = y2 - y1
            
            if dx == 0 and dy == 0:
                dist = math.hypot(x - x1, y - y1)
            else:
                t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / (dx**2 + dy**2)))
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy
                dist = math.hypot(x - closest_x, y - closest_y)
            
            min_dist = min(min_dist, dist)
        cte = min_dist
        
        # Heading Error 계산
        target_yaw = math.atan2(self.lookahead_point[1] - y, self.lookahead_point[0] - x)
        heading_error = self.normalize_angle(target_yaw - yaw)
        
        # Lookahead Point 발행
        lookahead_msg = PoseStamped()
        lookahead_msg.header.frame_id = 'map'
        lookahead_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_msg.pose.position.x = float(self.lookahead_point[0])
        lookahead_msg.pose.position.y = float(self.lookahead_point[1])
        lookahead_msg.pose.position.z = 0.0
        lookahead_msg.pose.orientation.w = 1.0
        self.lookahead_pub.publish(lookahead_msg)
        
        # State 발행
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        
        # Waypoint Index 발행 (도착 판정 기반)
        waypoint_idx_msg = Int32()
        waypoint_idx_msg.data = self.current_waypoint_idx
        self.waypoint_idx_pub.publish(waypoint_idx_msg)
        
        # Lookahead Index 발행 (현재 세그먼트 인덱스)
        idx_msg = Int32()
        idx_msg.data = self.current_segment_idx
        self.idx_pub.publish(idx_msg)
        
        # Heading Error 발행
        heading_msg = Float64()
        heading_msg.data = math.degrees(heading_error)
        self.heading_error_pub.publish(heading_msg)
        
        # CTE 발행
        cte_msg = Float64()
        cte_msg.data = cte
        self.cte_pub.publish(cte_msg)



def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
