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
        super().__init__('pure_pursuit_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_ppc', 10)

        # plot 용
        self.lookahead_pub = self.create_publisher(PoseStamped, '/ppc/lookahead_point', 10)
        self.state_pub = self.create_publisher(String, '/ppc/state', 10)
        self.idx_pub = self.create_publisher(Int32, '/ppc/lookahead_idx', 10)
        self.heading_error_pub = self.create_publisher(Float64, '/ppc/heading_error', 10)
        self.cte_pub = self.create_publisher(Float64, '/ppc/cte', 10)
        self.waypoint_idx_pub = self.create_publisher(Int32, '/ppc/waypoint_idx', 10)  # ✅ 새로운 Publisher


        self.dt_gps = 0.1
        self.gps_sub = self.create_subscription(NavSatFix, '/navsat', self.gps_callback, 1)
        self.absxy_pub = self.create_publisher(PoseWithCovarianceStamped, '/abs_xy', 1)

        self.dt_imu = 0.01
        self.imu_sub = self.create_subscription(Imu, '/imu_cal', self.imu_callback, 1)

        self.mag_sub = self.create_subscription(MagneticField, '/magnetometer', self.mag_callback, 1)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 1)

        self.dt_timer = 0.25
        self.timer = self.create_timer(self.dt_timer, self.timer_callback)

        self.global_odom_sub = self.create_subscription(Odometry, '/odometry/ekf_single', self.global_odom_callback, 1)
        # self.local_odom_sub = self.create_subscription(Odometry, '/odometry/local', self.local_odom_callback, 1)
        
        # ppc enable
        self.ppc_enable_sub = self.create_subscription(Bool, '/ppc/enable', self.ppc_enable_callback, 10)
        # 활성화 상태 변수
        self.is_enabled = False 
        # 정지 명령용 Twist
        self.zero_twist = Twist()

        # 예시 경로점
        self.waypoints = [
            (-9, 3.75), (9, 3.75),
            (9, 2.25), (-9, 2.25),
            (-9, 0.75), (9, 0.75),
            (9, -0.75), (-9, -0.75),
            (-9, -2.25), (9, -2.25),
            (9, -3.75), (-9, -3.75)
        ]
        
        self.lookahead_distance = 1.0 # gazebo: 1.0, real: 2.0
        self.lookahead_idx = 1
        self.interpolated_waypoints = self.interpolate_waypoints(self.waypoints, self.lookahead_distance)

        # ✅ 추가: 현재 목표 waypoint index (원본 waypoints 기준)
        self.current_waypoint_idx = 0  # 처음 시작은 waypoint[0] → waypoint[1]로 향함

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.global_x = None
        self.global_y = None
        self.global_yaw = None

        self.local_x = None
        self.local_y = None
        self.local_yaw = None

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

        self.imu_cnt = 0

        self.clock_cnt = 0
        self.clock_cnt_pre = 0
        self.current_time = 0.0
        self.current_time_pre = 0.0

        self.rotate_flag = 0
        self.state = "move"  # "move" 또는 "rotate"

        # ========== 추가: Path Publishers ==========
        self.waypoints_path_pub = self.create_publisher(Path, '/waypoints_path', 10)
        # clock이 시작되면 1번만 발행하도록 플래그 추가
        self.waypoints_published = False

        # Waypoints를 Path로 발행
        self.publish_waypoints_path()
    
    # ========== 추가: Waypoints Path 발행 메서드 ==========
    def publish_waypoints_path(self):
        """Waypoints를 nav_msgs/Path로 발행 (plot_ppc.py가 수신)"""
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
    
    def ppc_enable_callback(self, msg: Bool):
        """PPC 활성화/비활성화 상태를 수신하고 즉각적인 조치를 취합니다."""
        
        # 상태가 변경될 때만 로그 출력
        if self.is_enabled != msg.data:
            if msg.data:
                self.get_logger().info('Pure Pursuit [ENABLED]')
            else:
                # 비활성화되는 즉시 0 속도를 1회 발행
                # (twist_mux가 이 토픽을 timeout 처리할 때까지 기다리지 않도록)
                self.cmd_pub.publish(self.zero_twist)
                
        self.is_enabled = msg.data

    def gps_callback(self, msg):
        lon = msg.longitude
        lat = msg.latitude
        x, y = self.enu_converter.transform(lon, lat)
        # self.current_x = x
        # self.current_y = y

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
        # ✅ 주석 해제: 시간이 변하면 업데이트
        if self.clock_cnt != self.clock_cnt_pre:
            self.clock_cnt_pre = self.clock_cnt

        # 쿼터니언 -> yaw 변환
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        # self.current_yaw = yaw

        self.imu_cnt += 10


    def mag_callback(self, msg: MagneticField):
        """✅ 주석 해제: Magnetometer 콜백"""
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

        # self.get_logger().info(
        #     f"Mag: x={self.mag_x:.3e}, y={self.mag_y:.3e}, z={self.mag_z:.3e}, yaw={math.degrees(self.mag_yaw):.1f}°"
        # )
        

    def clock_callback(self, msg):
        """✅ Clock 콜백 - Gazebo가 시작되면 waypoint 발행"""
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.current_time == 0.0:
            self.clock_cnt = 0.0
        elif self.current_time > self.current_time_pre:
            self.clock_cnt += 1.0
            self.current_time_pre = self.current_time
            
            # ✅ clock이 시작되면 1번만 waypoint 발행
            if not self.waypoints_published:
                self.publish_waypoints_path()
                self.waypoints_published = True


    def global_odom_callback(self, msg: Odometry):
        """/odometry/global 콜백. EKF의 전역 위치 추정 결과를 사용."""
        self.global_x = msg.pose.pose.position.x
        self.global_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        _, _, self.global_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])


    def local_odom_callback(self, msg: Odometry):
        """/odometry/local 콜백. 지역 위치 추정 결과를 저장."""
        self.local_x = msg.pose.pose.position.x
        self.local_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.local_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])


    def interpolate_waypoints(self, waypoints, lookahead_distance):
        """
        주어진 경로점(waypoints) 리스트를 lookahead_distance 간격으로 보간하여
        새로운 경로점 리스트를 반환합니다.
        waypoints: [(x1, y1), (x2, y2), ...]
        lookahead_distance: float
        return: [(x, y), ...] (보간된 경로점 리스트)
        """
        if len(waypoints) < 2:
            return waypoints

        interpolated = []
        for i in range(len(waypoints) - 1):
            x0, y0 = waypoints[i]
            x1, y1 = waypoints[i + 1]
            dx = x1 - x0
            dy = y1 - y0
            segment_length = np.hypot(dx, dy)
            num_points = max(int(segment_length // lookahead_distance), 1)
            for j in range(num_points):
                t = j * lookahead_distance / segment_length
                xi = x0 + t * dx
                yi = y0 + t * dy
                interpolated.append((xi, yi))
        # 마지막 점 추가
        interpolated.append(waypoints[-1])

        return interpolated
    
    def normalize_angle(self, angle):
        """-pi ~ pi 범위로 각도 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    # plot def
    def calculate_cte(self, x, y):
        """보간된 경로 기준 Cross-Track Error 계산"""
        min_dist = float('inf')
        
        for i in range(len(self.interpolated_waypoints) - 1):
            x1, y1 = self.interpolated_waypoints[i]
            x2, y2 = self.interpolated_waypoints[i + 1]
            
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
        
        return min_dist
    
    def timer_callback(self):
        """메인 제어 루프 (250ms마다 실행)"""
        if not self.is_enabled:
            self.cmd_pub.publish(self.zero_twist)
            return
        
        twist = Twist()
        
        if self.global_x is None or self.global_y is None or self.global_yaw is None:
            return
        
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw

        # lookahead_idx를 순차적으로 따라감
        min_idx = self.lookahead_idx
        lookahead_point = self.interpolated_waypoints[min_idx]
        min_dist = np.hypot(lookahead_point[0] - x, lookahead_point[1] - y)

        # yaw 오차 계산
        yaw_error = self.normalize_angle(
            math.atan2(
                self.interpolated_waypoints[min_idx][1] - y,
                self.interpolated_waypoints[min_idx][0] - x
            ) - yaw
        )

        # ✅ 내적기반 도착 판정 + waypoint index 증가
        if min_idx > 0 and min_idx + 1 < len(self.interpolated_waypoints):
            prev_wp = self.interpolated_waypoints[min_idx - 1]
            curr_wp = self.interpolated_waypoints[min_idx]
            robot_pos = np.array([x, y])
            vec_path = np.array([curr_wp[0] - prev_wp[0], curr_wp[1] - prev_wp[1]])
            vec_robot = np.array([robot_pos[0] - curr_wp[0], robot_pos[1] - curr_wp[1]])
            dot = np.dot(vec_path, vec_robot)
            
            if dot > 0:
                if self.rotate_flag == 1:
                    self.rotate_flag = 0
                    self.state = "rotate"
                    
                    # ✅ waypoint 도착 판정 시 index 증가
                    if self.current_waypoint_idx + 1 < len(self.waypoints):
                        self.current_waypoint_idx += 1
                        self.get_logger().info(
                            f'Reached waypoint {self.current_waypoint_idx-1}, '
                            f'moving to waypoint {self.current_waypoint_idx}'
                        )
                
                min_idx += 1

        # lookahead_point 업데이트
        self.lookahead_idx = min_idx
        lookahead_point = self.interpolated_waypoints[self.lookahead_idx]

        # waypoints 중 하나와 lookahead_point의 거리가 0.01 이하이면 회전 플래그 on
        for wp in self.waypoints:
            if np.hypot(lookahead_point[0] - wp[0], lookahead_point[1] - wp[1]) < 0.01:
                self.rotate_flag = 1
                break
        
        if self.clock_cnt >= 999:
            if self.state == "move":
                dx = lookahead_point[0] - x
                dy = lookahead_point[1] - y
                ld = np.hypot(dx, dy)

                # 이전 curvature 저장용 변수 추가
                if not hasattr(self, 'prev_curvature'):
                    self.prev_curvature = 0.0

                if ld < self.lookahead_distance*0.25 : # gazebo: 0.25, real: 0.75
                    curvature = self.prev_curvature if hasattr(self, 'prev_curvature') else 0.0
                else:
                    # 차량의 heading 기준 lookahead point의 좌표
                    x_r = math.cos(yaw) * dx + math.sin(yaw) * dy
                    y_r = -math.sin(yaw) * dx + math.cos(yaw) * dy
                    curvature = 2.0 * y_r / (ld ** 2)
                    self.prev_curvature = curvature 

                if self.rotate_flag == 1:
                    twist.linear.x = 0.1
                    twist.angular.z = twist.linear.x * curvature
                else:
                    twist.linear.x = 0.3
                    twist.angular.z = twist.linear.x * curvature

                # self.get_logger().info(
                #     f"Current: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg | "
                #     f"Target: x={lookahead_point[0]:.2f}, y={lookahead_point[1]:.2f} | "
                #     f"Curvature: {curvature:.4f} | "
                #     f"Waypoint idx: {self.lookahead_idx}, dist: {min_dist:.4f} | "
                #     f"cnt: {(self.clock_cnt - self.imu_cnt):.4f}, {self.clock_cnt:.4f}/{self.imu_cnt:.4f} | "
                # )

            elif self.state == "rotate":
                # 가장 가까운 waypoint(시작점 제외)를 찾음
                min_wp_dist = float('inf')
                target_yaw = yaw
                for i in range(1, len(self.waypoints)):
                    wp = self.waypoints[i]
                    dist = np.hypot(wp[0] - x, wp[1] - y)
                    if dist < min_wp_dist:
                        min_wp_dist = dist
                        prev_wp = wp
                        target_wp = self.waypoints[i+1]

                # prev_wp → target_wp 방향을 목표 yaw로 사용
                target_yaw = math.atan2(target_wp[1] - prev_wp[1], target_wp[0] - prev_wp[0])
                yaw_error = self.normalize_angle(target_yaw - yaw)
                yaw_error_deg = abs(math.degrees(yaw_error))

                if yaw_error_deg < 1.0:
                    self.state = "move"
                    self.get_logger().info(
                        f"Yaw aligned (error {yaw_error_deg:.2f} deg). Switching to move state."
                    )
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = np.sign(yaw_error) * (np.pi / 36)  # 10 deg/s, 방향 고려
                    self.get_logger().info(
                        f"Rotating in place. Yaw error: {yaw_error_deg:.2f} current: {math.degrees(yaw):.2f} deg) | "
                        f"cnt: {(self.clock_cnt - self.imu_cnt):.4f}, {self.clock_cnt:.4f}/{self.imu_cnt:.4f} | "
                    )
        
        # plot 용 데이터 계산 및 발행
        # CTE 계산
        cte = self.calculate_cte(x, y)
        
        # Heading Error 계산
        target_yaw = math.atan2(lookahead_point[1] - y, lookahead_point[0] - x)
        heading_error = self.normalize_angle(target_yaw - yaw)
        
        # Lookahead Point 발행
        lookahead_msg = PoseStamped()
        lookahead_msg.header.frame_id = 'map'
        lookahead_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_msg.pose.position.x = lookahead_point[0]
        lookahead_msg.pose.position.y = lookahead_point[1]
        lookahead_msg.pose.position.z = 0.0
        lookahead_msg.pose.orientation.w = 1.0
        self.lookahead_pub.publish(lookahead_msg)
        
        # State 발행
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        
        # ✅ Waypoint Index 발행 (도착 판정 기반)
        waypoint_idx_msg = Int32()
        waypoint_idx_msg.data = self.current_waypoint_idx
        self.waypoint_idx_pub.publish(waypoint_idx_msg)
        
        # Lookahead Index 발행
        idx_msg = Int32()
        idx_msg.data = self.lookahead_idx
        self.idx_pub.publish(idx_msg)
        
        # Heading Error 발행
        heading_msg = Float64()
        heading_msg.data = math.degrees(heading_error)
        self.heading_error_pub.publish(heading_msg)
        
        # CTE 발행
        cte_msg = Float64()
        cte_msg.data = cte
        self.cte_pub.publish(cte_msg)

        self.cmd_pub.publish(twist) 

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()