#!/usr/bin/env python3
"""Pure Pursuit Intersection Point, Sim"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from nav_msgs.msg import Odometry, Path
import math
from sensor_msgs.msg import Imu, NavSatFix
import pyproj
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import MagneticField
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String, Int32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Parameters
        self.declare_parameter('lookahead_distance', 1.5)  # Declared in launch file
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_ppc', 10)
        # For plot
        self.lookahead_pub = self.create_publisher(PoseStamped, '/ppc/lookahead_point', 10)
        self.state_pub = self.create_publisher(String, '/ppc/state', 10)
        self.heading_error_pub = self.create_publisher(Float64, '/ppc/heading_error', 10)
        self.cte_pub = self.create_publisher(Float64, '/ppc/cte', 10)
        self.waypoint_idx_pub = self.create_publisher(Int32, '/ppc/waypoint_idx', 10)
        self.waypoints_path_pub = self.create_publisher(Path, '/waypoints_path', 10)

        # Send stop command when completed
        self.command_pub = self.create_publisher(String, '/state_command', 10)
        # Grid based UWB position
        self.absxy_pub = self.create_publisher(PoseWithCovarianceStamped, '/abs_xy', 1)

        # Subscriptions
        self.dt_gps = 0.1
        self.gps_sub = self.create_subscription(NavSatFix, '/navsat', self.gps_callback, 1)

        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 1)

        self.global_odom_sub = self.create_subscription(Odometry, '/odometry/ekf_single', self.global_odom_callback, 1)
        # ppc enable
        self.ppc_enable_sub = self.create_subscription(Bool, '/ppc/enable', self.ppc_enable_callback, 10)

        # Control Frequency
        self.dt_timer = 0.25
        self.timer = self.create_timer(self.dt_timer, self.timer_callback)

        # Initial ppc_enable
        self.is_enabled = False 

        # STOP twist
        self.zero_twist = Twist()

        # Target Waypoints
        self.waypoints = [
            # (-9, 3.75), (9, 3.75),
            # (9, 2.25), (-9, 2.25),
            # (-9, 0.75), (9, 0.75),
            # (9, -0.75), (-9, -0.75),
            # (-9, -2.25), (9, -2.25),
            # (9, -3.75), (-9, -3.75)
            (-9,9), (-5,9),
            (-5,7), (-8,7)
            # (-1,5), (-9,5),
            # (-9,1), (-1,1)
        ]
        
        # Apply ld from launch file
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.get_logger().info(f'[RUN] Line Following PPC, Ld={self.lookahead_distance}m')

        # First target waypoint index
        self.current_waypoint_idx = 1  # waypoint[0] → waypoint[1]

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.global_x = None
        self.global_y = None
        self.global_yaw = None

        # Reference point latitude and longitude (example: latitude and longitude corresponding to (0,0) in the world)
        origin_lat = 37.0
        origin_lon = 127.0

        # Create WGS84 -> ENU converter
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
        self.mag_yaw = None  # Magnetic heading estimate (rad)

        self.clock_cnt = 0
        self.clock_cnt_pre = 0
        self.current_time = 0.0
        self.current_time_pre = 0.0

        self.rotate_flag = 0
        self.state = "move"
        
        self.mission_completed = False

        # Publish waypoints path for plot
        self.publish_waypoints_path()
        self.waypoints_published = False

    # Timer Loop
    def timer_callback(self):
        # Check Conditions before run
        if not self.check_run():
            return
        
        # Calculate when enabled for CPU usage
        if hasattr(self, 'current_odom_msg') and self.current_odom_msg is not None:
            msg = self.current_odom_msg
            self.global_x = msg.pose.pose.position.x
            self.global_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            _, _, self.global_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Return when no odom
        if self.global_x is None or self.global_y is None or self.global_yaw is None:
            return
        
        # Arrival check and state update
        self.check_arrival()
     
        # Corner entry detection
        self.check_rotate()
        
        # Calculate velocity command
        twist = Twist()
        if self.state == "move":
            twist = self.move_state()
        elif self.state == "rotate":
            twist = self.rotate_state()
        # Publish velocity command
        self.publish_cmd_vel(twist)
        
        # Publish plot data
        self.publish_for_plot()
    
    # Main Control Logic
    def check_run(self):
        # Stop if disabled
        if not self.is_enabled:
            self.cmd_pub.publish(self.zero_twist)
            return False
        
        # Stop if mission completed
        if self.mission_completed:
            self.cmd_pub.publish(self.zero_twist)
            self.command_pub.publish(String(data="STOP"))
            return False
        
        return True
    
    def check_arrival(self):
        """Arrival check based on dot product"""
        x = self.global_x
        y = self.global_y
        
        # Final waypoint arrival check
        if self.current_waypoint_idx >= len(self.waypoints) - 1:
            target_wp = self.waypoints[-1]
            prev_wp = self.waypoints[-2]
            
            # Path vector (previous → target)
            vec_path = np.array([target_wp[0] - prev_wp[0], target_wp[1] - prev_wp[1]])
            # Robot vector (target → robot)
            vec_robot = np.array([x - target_wp[0], y - target_wp[1]])
            dot = np.dot(vec_path, vec_robot)
            
            if dot > 0 and not self.mission_completed:
                self.get_logger().info("[RUN] Reached final waypoint - Mission completed")
                self.mission_completed = True
                self.current_waypoint_idx = len(self.waypoints) - 1
            return
        
        # Intermediate waypoint arrival check
        target_wp = self.waypoints[self.current_waypoint_idx]
        prev_wp = self.waypoints[self.current_waypoint_idx - 1]
        
        vec_path = np.array([target_wp[0] - prev_wp[0], target_wp[1] - prev_wp[1]])
        vec_robot = np.array([x - target_wp[0], y - target_wp[1]])
        dot = np.dot(vec_path, vec_robot)
        
        if dot > 0:
            # Passed point → move to next waypoint
            if self.rotate_flag == 1:
                self.rotate_flag = 0
                self.state = "rotate"
                self.get_logger().info("[RUN] Entering rotate state")
            
            self.get_logger().info(
                f'[RUN] Reached waypoint {self.current_waypoint_idx}, '
                f'moving to waypoint {self.current_waypoint_idx + 1}'
            )
            self.current_waypoint_idx += 1
    
    def check_rotate(self):
        """Corner entry detection"""
        if self.current_waypoint_idx >= len(self.waypoints):
            return
        
        target_wp = self.waypoints[self.current_waypoint_idx]
        dist_to_target = np.hypot(target_wp[0] - self.global_x, target_wp[1] - self.global_y)
        
        # Prepare to rotate if close enough to target waypoint
        if dist_to_target < self.lookahead_distance * 0.5:
            self.rotate_flag = 1
    
    def intersection_point(self, p1, p2, robot_x, robot_y, ld):
        """
        Calculate the intersection point between the robot's radius (Ld) and an infinite line,
        and select the point in the direction of travel using the dot product.
        """
        # Line vector (d) and robot relative vector (f)
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        fx = p1[0] - robot_x
        fy = p1[1] - robot_y
        
        # Quadratic equation coefficients and discriminant (circle-line intersection formula)
        a = dx**2 + dy**2
        b = 2 * (fx * dx + fy * dy)
        c = (fx**2 + fy**2) - ld**2
        discriminant = b**2 - 4*a*c
        
        # No intersection (too far from line)
        if discriminant < 0:
            return None
            
        # Calculate the two intersection points (t1, t2)
        sqrt_dis = math.sqrt(discriminant)
        t1 = (-b - sqrt_dis) / (2*a)
        t2 = (-b + sqrt_dis) / (2*a)
        
        i1_x, i1_y = p1[0] + t1 * dx, p1[1] + t1 * dy
        i2_x, i2_y = p1[0] + t2 * dx, p1[1] + t2 * dy
        
        # Select the point in the direction of travel using the dot product
        # (Path vector) • (Robot->intersection vector) > 0 is forward
        dot1 = dx * (i1_x - robot_x) + dy * (i1_y - robot_y)
        dot2 = dx * (i2_x - robot_x) + dy * (i2_y - robot_y)
        
        return (i1_x, i1_y) if dot1 > dot2 else (i2_x, i2_y)
    
    def move_state(self):
        """Move State Logic"""
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw
        
        # Define the current segment to follow (previous waypoint → target waypoint)
        if self.current_waypoint_idx < 1:
            self.current_waypoint_idx = 1
        
        if self.current_waypoint_idx >= len(self.waypoints):
            # Stop at final waypoint
            twist = Twist()
            return twist
        
        p1 = self.waypoints[self.current_waypoint_idx - 1]  # Start point
        p2 = self.waypoints[self.current_waypoint_idx]      # End point
        
        # Calculate the intersection point between the robot's radius (Ld) and the active segment
        lookahead_point = self.intersection_point(p1, p2, x, y, self.lookahead_distance)
        
        # If no intersection, fallback to the perpendicular foot (closest point on the path)
        if lookahead_point is None:
            # Calculate the closest point on the segment (perpendicular foot)
            dx_seg = p2[0] - p1[0]
            dy_seg = p2[1] - p1[1]
            
            if dx_seg == 0 and dy_seg == 0:
                # Segment is a point
                closest_point = p1
            else:
                # Calculate parameter t (clamped between 0 and 1)
                t = max(0, min(1, ((x - p1[0]) * dx_seg + (y - p1[1]) * dy_seg) / (dx_seg**2 + dy_seg**2)))
                closest_point = (p1[0] + t * dx_seg, p1[1] + t * dy_seg)
            
            self.get_logger().warn(
                f"[RUN] No intersection found (too far from path). "
                f"Using closest point on segment for recovery."
            )
            lookahead_point = closest_point
        
        # Distance to the lookahead point
        dx = lookahead_point[0] - x
        dy = lookahead_point[1] - y
        ld = np.hypot(dx, dy)
        
        # Calculate Pure Pursuit curvature
        if ld > 0.01:
            # Transform to vehicle coordinate frame
            x_r = math.cos(yaw) * dx + math.sin(yaw) * dy
            y_r = -math.sin(yaw) * dx + math.cos(yaw) * dy
            curvature = 2.0 * y_r / (ld ** 2)
        else:
            curvature = 0.0
        
        # Generate velocity command
        twist = Twist()
        if self.rotate_flag == 1:
            twist.linear.x = 0.15
            twist.angular.z = twist.linear.x * curvature
        else:
            twist.linear.x = 0.4
            twist.angular.z = twist.linear.x * curvature
        
        # Save lookahead_point for plot
        self.current_lookahead_point = lookahead_point
        
        return twist
    
    def rotate_state(self):
        """Rotate in place to align with the next path direction"""
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw
        
        # Calculate direction from current waypoint to next waypoint
        if self.current_waypoint_idx >= len(self.waypoints):
            # Last waypoint reached
            self.state = "move"
            return Twist()
        
        curr_wp = self.waypoints[self.current_waypoint_idx - 1]
        next_wp = self.waypoints[self.current_waypoint_idx]
        
        # Target direction
        target_yaw = math.atan2(next_wp[1] - curr_wp[1], next_wp[0] - curr_wp[0])
        yaw_error = self.normalize_angle(target_yaw - yaw)
        yaw_error_deg = abs(math.degrees(yaw_error))
        
        twist = Twist()
        if yaw_error_deg < 1.0:
            self.state = "move"
            self.get_logger().info(
                f"[RUN] Yaw aligned (error {yaw_error_deg:.2f} deg). Switching to move state."
            )
        else:
            twist.linear.x = 0.0
            twist.angular.z = np.sign(yaw_error) * (np.pi / 36)  # 10 deg/s
            self.get_logger().info(
                f"[RUN] Rotating in place. Yaw error: {yaw_error_deg:.2f}°"
            )
        
        return twist

    def publish_cmd_vel(self, twist):
        self.cmd_pub.publish(twist)

    # Input
    def ppc_enable_callback(self, msg: Bool):
        # Log only when state changes
        if self.is_enabled != msg.data:
            if msg.data:
                # ENABLE
                if self.mission_completed:
                    # Start a new mission
                    self.mission_completed = False
                    self.current_waypoint_idx = 1
                    self.state = "move"
                    self.rotate_flag = 0
                    self.get_logger().info('[PPC] ENABLED - Starting NEW mission')
                else:
                    # Continue from paused state
                    self.get_logger().info(
                        f'[PPC] RESUMED - Continuing from waypoint {self.current_waypoint_idx}'
                    )
            else:
                # DISABLE
                self.cmd_pub.publish(self.zero_twist)
                self.get_logger().info(
                    f'[PPC] PAUSED - State preserved (mission_completed={self.mission_completed})'
                )
                
        self.is_enabled = msg.data

    # gps data to abs_xy for UWB in gazebo
    def gps_callback(self, msg):
        lon = msg.longitude
        lat = msg.latitude
        x, y = self.enu_converter.transform(lon, lat)

        p = PoseWithCovarianceStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        p.pose.pose.position.x = x
        p.pose.pose.position.y = y
        p.pose.pose.position.z = 0.0
        p.pose.pose.orientation.x = 0.0
        p.pose.pose.orientation.y = 0.0
        p.pose.pose.orientation.z = 0.0
        p.pose.pose.orientation.w = 1.0

        p.pose.covariance = [
            0.04, 0, 0, 0, 0, 0,
            0, 0.04, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e6,
        ]
        self.absxy_pub.publish(p)
        
    def clock_callback(self, msg):
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        if self.current_time == 0.0:
            self.clock_cnt = 0.0
        elif self.current_time > self.current_time_pre:
            self.clock_cnt += 1.0
            self.current_time_pre = self.current_time
            
            if not self.waypoints_published:
                self.publish_waypoints_path()
                self.waypoints_published = True

    def global_odom_callback(self, msg: Odometry):
        self.current_odom_msg = msg
    
    # Utility
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # For Plot
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
    
    # Publish data for plot
    def publish_for_plot(self):
        x = self.global_x
        y = self.global_y
        yaw = self.global_yaw
        
        # Define active segment
        if self.current_waypoint_idx < 1 or self.current_waypoint_idx >= len(self.waypoints):
            return
        
        p1 = self.waypoints[self.current_waypoint_idx - 1]
        p2 = self.waypoints[self.current_waypoint_idx]
        
        # Lookahead Point calculation (already computed in move_state)
        lookahead_point = getattr(self, 'current_lookahead_point', p2)
        
        # CTE calculation (shortest distance to current active segment)
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        # Case when segment is a point
        if dx == 0 and dy == 0:
            # Euclidean distance
            cte = math.hypot(x - p1[0], y - p1[1])
        else:
            # Calculate the projection factor 't'
            # Normalized by the squared length of the segment.
            # Formula: t = dot_product(robot_vec, path_vec) / length_squared(path_vec)
            t = max(0, min(1, ((x - p1[0]) * dx + (y - p1[1]) * dy) / (dx**2 + dy**2)))
            # Clamp 't' to the range [0, 1] to stay within the segment bounds
            # t = 0: Closest point is P1 (Start)
            # t = 1: Closest point is P2 (End)
            # 0 < t < 1: Closest point is strictly on the line segment
            closest_x = p1[0] + t * dx
            closest_y = p1[1] + t * dy
            # Find the coordinates of the closest point on the segment
            cte = math.hypot(x - closest_x, y - closest_y)
        
        # Heading Error calculation
        # Calculate the target yaw angle from the robot's current position to the lookahead point
        target_yaw = math.atan2(lookahead_point[1] - y, lookahead_point[0] - x)
        # Heading error = target yaw - current yaw
        heading_error = self.normalize_angle(target_yaw - yaw)
        
        # Lookahead Point publishing
        lookahead_msg = PoseStamped()
        lookahead_msg.header.frame_id = 'map'
        lookahead_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_msg.pose.position.x = float(lookahead_point[0])
        lookahead_msg.pose.position.y = float(lookahead_point[1])
        lookahead_msg.pose.position.z = 0.0
        lookahead_msg.pose.orientation.w = 1.0
        self.lookahead_pub.publish(lookahead_msg)
        
        # State publishing
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        
        # Waypoint Index publishing
        waypoint_idx_msg = Int32()
        waypoint_idx_msg.data = self.current_waypoint_idx
        self.waypoint_idx_pub.publish(waypoint_idx_msg)
        
        # Heading Error publishing
        heading_msg = Float64()
        heading_msg.data = math.degrees(heading_error)
        self.heading_error_pub.publish(heading_msg)
        
        # CTE publishing
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
