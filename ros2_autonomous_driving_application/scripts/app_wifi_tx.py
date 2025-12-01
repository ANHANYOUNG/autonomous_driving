#!/usr/bin/env python3
"""App WiFi Transmitter: Send UWB and EKF data to mobile app"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import json
import threading
import math
from http.server import HTTPServer, BaseHTTPRequestHandler


class AppDataHandler(BaseHTTPRequestHandler):
    """HTTP request handler"""
    
    def __init__(self, *args, ros_node=None, **kwargs):
        self.ros_node = ros_node
        super().__init__(*args, **kwargs)
    
    # Handle HTTP GET requests to send the latest UWB/EKF data to the app.
    def do_GET(self):
        """Handle GET request"""
        if self.path == '/to_app':
            try:
                if self.ros_node:
                    json_str = self.ros_node.get_uwb_json_data()
                else:
                    json_str = json.dumps({"error": "ROS node not available"})
                
                # Success response
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()

                self.wfile.write(json_str.encode('utf-8'))
                
            except Exception as e:
                # Server error
                self.send_error(500, f"Server error: {e}")
        else:
            # Invalid path
            self.send_error(404, "Not Found")
    
    # Handle HTTP OPTIONS requests to allow GET access for CORS(Security Check) compliance.
    def do_OPTIONS(self):
        """Handle CORS preflight request"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    def log_message(self, format, *args):
        pass  # Do not output logs


class AppWiFiTransmitter(Node):
    def __init__(self):
        super().__init__('app_wifi_transmitter')
        
        # Subscriptions
        self.uwb_subscription = self.create_subscription(String,'/uwb_raw_data',self.uwb_data_callback,1)
        self.ekf_subscription = self.create_subscription(Odometry,'/odometry/ekf_single',self.ekf_data_callback,1)
        
        # HTTP server settings
        self.declare_parameter('port', 8888)
        self.declare_parameter('host', '0.0.0.0')  # Allow connections from all interfaces
        
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        
        # Data storage (do not store msg objects)
        self.latest_uwb_data = None
        self.has_ekf_data = False  # Use bool flag instead of msg
        self.ekf_callback_count = 0  # Callback counter
        
        self.uwb_json_data = {
            "anchors": [
                {"id": "A1", "x": 0.00, "y": 0.00},
                {"id": "A2", "x": 0.00, "y": 0.00},
                {"id": "A3", "x": 0.00, "y": 0.00},
                {"id": "A4", "x": 0.00, "y": 0.00}
            ],
            "tag": {"x": 0.00, "y": 0.00},
            "stop_flag": 0,
            "err_msg": [],
            "tag_vel": 0.00,
            "tag_ori": 0.00
        }
        
        # Data access lock
        self.data_lock = threading.Lock()
        
        # Statistics
        self.request_count = 0
        self.data_count = 0
        
        # Start HTTP server
        self.start_http_server()

        self.get_logger().info(f'[WIFI_TX] App WiFi Transmitter started on {self.host}:{self.port}')
        self.get_logger().info('[WIFI_TX] Endpoint: http://192.168.4.1:8888/to_app')
        self.get_logger().info('[WIFI_TX] Subscribed to /uwb_raw_data and /odometry/ekf_single')

    def start_http_server(self):
        """Start HTTP server in a separate thread"""
        def handler_factory(*args, **kwargs):
            return AppDataHandler(*args, ros_node=self, **kwargs)
        
        max_retries = 5
        for retry in range(max_retries):
            try:
                current_port = self.port + retry
                # Optimize server settings
                self.server = HTTPServer((self.host, current_port), handler_factory)
                self.server.timeout = 1.0  # 1 second timeout
                
                # Run server in a daemon thread (auto shutdown on main process exit)
                self.server_thread = threading.Thread(
                    target=self.serve_forever_with_shutdown, 
                    daemon=True
                )
                self.server_thread.start()

                self.get_logger().info(f'[WIFI_TX] HTTP Server listening on {self.host}:{current_port}')
                self.port = current_port  # Update actual port in use
                break
                
            except OSError as e:
                if e.errno == 98:  # Address already in use
                    self.get_logger().warn(f'[WIFI_TX] Port {current_port} in use, trying {current_port + 1}')
                    continue
                else:
                    self.get_logger().error(f'[WIFI_TX] Failed to start HTTP server: {e}')
                    break
        else:
            self.get_logger().error(f'[WIFI_TX] Could not find available port after {max_retries} attempts')

    def serve_forever_with_shutdown(self):
        """Run server (periodically check for shutdown)"""
        try:
            self.server.serve_forever()
        except Exception as e:
            self.get_logger().error(f'[WIFI_TX] HTTP server error: {e}')

    def quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle (degree)"""
        # yaw (z-axis rotation)
        yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        yaw_deg = math.degrees(yaw_rad)
        return yaw_deg
    
    def ekf_data_callback(self, msg):
        """Receive EKF odometry data (process only once every 50 callbacks)"""
        try:
            with self.data_lock:  # Thread safety
                self.ekf_callback_count += 1
                self.has_ekf_data = True  # Use bool flag instead of storing msg
                
                # Update data only once every 50 callbacks
                if self.ekf_callback_count % 50 == 0:
                    # Position information (x, y)
                    position_x = msg.pose.pose.position.x
                    position_y = msg.pose.pose.position.y
                    
                    # Velocity information (x, y plane velocity)
                    vel_x = msg.twist.twist.linear.x
                    vel_y = msg.twist.twist.linear.y
                    velocity_magnitude = math.sqrt(vel_x**2 + vel_y**2)
                    
                    # Orientation information (yaw angle)
                    orientation = msg.pose.pose.orientation
                    yaw_deg = self.quaternion_to_yaw(
                        orientation.x, orientation.y, orientation.z, orientation.w
                    )
                    
                    # Update JSON data
                    self.uwb_json_data["tag"]["x"] = round(position_x, 2)
                    self.uwb_json_data["tag"]["y"] = round(position_y, 2)
                    self.uwb_json_data["tag_vel"] = round(velocity_magnitude, 2)
                    self.uwb_json_data["tag_ori"] = round(yaw_deg, 2)
            
        except Exception as e:
            self.get_logger().error(f'[WIFI_TX] Error processing EKF data: {e}')

    def uwb_data_callback(self, msg):
        """Receive UWB raw data and convert to JSON (anchors only)"""
        try:
            with self.data_lock:  # Thread safety
                # Store raw data
                self.latest_uwb_data = msg.data
                self.data_count += 1
                
                # Parse uwb_raw_data: "0.00, 0.00, 10.00, 0.00, 10.00, 15.00, 0.00, 15.00, 5.00, 10.00"
                data_parts = [float(x.strip()) for x in msg.data.split(',')]
                
                if len(data_parts) >= 8:  # 4 anchors = 8 values (tag info from EKF)
                    # Anchor positions (4 pairs of x, y)
                    self.uwb_json_data["anchors"] = [
                        {"id": "A1", "x": round(data_parts[0], 2), "y": round(data_parts[1], 2)},
                        {"id": "A2", "x": round(data_parts[2], 2), "y": round(data_parts[3], 2)},
                        {"id": "A3", "x": round(data_parts[4], 2), "y": round(data_parts[5], 2)},
                        {"id": "A4", "x": round(data_parts[6], 2), "y": round(data_parts[7], 2)}
                    ]
                    
        except Exception as e:
            self.get_logger().error(f'[WIFI_TX] Error processing UWB data: {e}')
    
    def get_uwb_json_data(self):
        """Return the latest UWB JSON data upon app request"""
        with self.data_lock:  # Thread safety
            self.request_count += 1
            
            # Warn and use default if no EKF data
            if not self.has_ekf_data:  # Check flag instead of msg object
                self.uwb_json_data["stop_flag"] = 1
                self.uwb_json_data["err_msg"] = ["No EKF data available"]
            else:
                self.uwb_json_data["stop_flag"] = 0
                self.uwb_json_data["err_msg"] = []
            
            # Warn if no UWB data (anchor info)
            if self.latest_uwb_data is None:
                if "No anchor data available" not in self.uwb_json_data["err_msg"]:
                    self.uwb_json_data["err_msg"].append("No anchor data available")
            return json.dumps(self.uwb_json_data)
    
    def destroy_node(self):
        """Clean up HTTP server on node shutdown"""
        self.get_logger().info('[WIFI_TX] Shutting down App WiFi Transmitter...')
        
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server.server_close()
        
        self.get_logger().info(f'[WIFI_TX] Total requests served: {self.request_count}')
        self.get_logger().info(f'[WIFI_TX] Total UWB data received: {self.data_count}')
        self.get_logger().info(f'[WIFI_TX] Total EKF callbacks: {self.ekf_callback_count}')

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AppWiFiTransmitter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
