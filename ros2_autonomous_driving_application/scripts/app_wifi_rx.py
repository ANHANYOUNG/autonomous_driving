#!/usr/bin/env python3
"""App WiFi Receiver: Receive JSON data from mobile app and publish to ROS2 topics"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse


class AppDataHandler(BaseHTTPRequestHandler):
    """HTTP request handler"""
    def __init__(self, *args, ros_node=None, **kwargs):
        self.ros_node = ros_node
        super().__init__(*args, **kwargs)
    
    # Handle HTTP POST requests to receive JSON commands from the mobile app.
    def do_POST(self):
        """Handle POST request"""
        if self.path == '/to_rasp':
            try:
                # Check data size from Content-Length header
                content_length = int(self.headers['Content-Length'])
                
                # Read POST data
                post_data = self.rfile.read(content_length)
                
                # Parse JSON
                json_data = json.loads(post_data.decode('utf-8'))
                
                # Pass data to ROS node
                if self.ros_node:
                    self.ros_node.process_app_data(json_data)
                
                # Send success response
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(b'{"status":"ok"}')
                
            except json.JSONDecodeError as e:
                # JSON parsing error
                self.send_error(400, f"Invalid JSON: {e}")
                
            except Exception as e:
                # Other errors
                self.send_error(500, f"Server error: {e}")
        else:
            # Invalid path
            self.send_error(404, "Not Found")
    
    # Handle HTTP OPTIONS requests to support CORS(Security Check) preflight checks.
    def do_OPTIONS(self):
        """Handle CORS preflight request"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    def log_message(self, format, *args):
        """Log messages"""
        if self.ros_node:
            self.ros_node.get_logger().debug(f"HTTP: {format % args}")


class AppWiFiReceiver(Node):
    def __init__(self):
        super().__init__('app_wifi_receiver')
        
        # Publishers
        self.sw_bits_pub = self.create_publisher(String, '/app/sw_bits', 10)
        self.key_bits_pub = self.create_publisher(String, '/app/key_bits', 10)
        self.speed_bits_pub = self.create_publisher(String, '/app/speed_bits', 10)
        self.raw_app_data_pub = self.create_publisher(String, '/app/raw_data', 10)
        self.video_bit_pub = self.create_publisher(String, '/app/video_bit', 10)
        self.safe_bit_pub = self.create_publisher(String, '/app/safe_bit', 10)
        
        # HTTP server settings
        self.declare_parameter('port', 8889)
        self.declare_parameter('host', '0.0.0.0')  # Allow connections from all interfaces
        
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        
        # Data counter (for statistics)
        self.data_count = 0
        self.last_data_time = None
        
        # Start HTTP server
        self.start_http_server()
        
        self.get_logger().info(f'[WIFI_RX] App WiFi Receiver started on {self.host}:{self.port}')
        self.get_logger().info('[WIFI_RX] ip: http://192.168.4.1:8889/to_rasp')
    
    def start_http_server(self):
        """Start HTTP server in a separate thread"""
        def handler_factory(*args, **kwargs):
            return AppDataHandler(*args, ros_node=self, **kwargs)
        
        try:
            self.server = HTTPServer((self.host, self.port), handler_factory)
            
            # Run server in a separate thread to avoid stopping 
            self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.server_thread.start()

            self.get_logger().info(f'[WIFI_RX] HTTP Server listening on {self.host}:{self.port}')

        except Exception as e:
            self.get_logger().error(f'[WIFI_RX] Failed to start HTTP server: {e}')

    def process_app_data(self, json_data):
        """Publish JSON data received from the app to ROS2 topics"""
        try:
            # Increment data counter
            self.data_count += 1
            current_time = self.get_clock().now()
            
            # Publish entire JSON data as raw data
            # raw_msg = String()
            # raw_msg.data = json.dumps(json_data)
            # self.raw_app_data_pub.publish(raw_msg)
            
            # sw_bits data
            if 'sw_bits' in json_data:
                sw_msg = String()
                sw_msg.data = str(json_data['sw_bits'])
                self.sw_bits_pub.publish(sw_msg)
                # self.get_logger().debug(f'[WIFI_RX] Published sw_bits: {json_data["sw_bits"]}')
            
            # key_bits data
            if 'key_bits' in json_data:
                key_msg = String()
                key_msg.data = str(json_data['key_bits'])
                self.key_bits_pub.publish(key_msg)
                # self.get_logger().debug(f'[WIFI_RX] Published key_bits: {json_data["key_bits"]}')

            # speed_bits data
            if 'speed_bits' in json_data:
                speed_msg = String()
                speed_msg.data = str(json_data['speed_bits'])
                self.speed_bits_pub.publish(speed_msg)
                # self.get_logger().debug(f'[WIFI_RX] Received speed_bits: {json_data["speed_bits"]}')

            # video_bit = 0 or 1
            if 'video_bit' in json_data:
                video_msg = String()
                video_msg.data = str(json_data['video_bit'])
                self.video_bit_pub.publish(video_msg)
                # self.get_logger().debug(f'[WIFI_RX] Received video_bit: {json_data["video_bit"]}')

            # safe_bit = 0 or 1
            if 'safe_bit' in json_data:
                safe_msg = String()
                safe_msg.data = str(json_data['safe_bit'])
                self.safe_bit_pub.publish(safe_msg)
                # self.get_logger().debug(f'[WIFI_RX] Received safe_bit: {json_data["safe_bit"]}')

            self.last_data_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'[WIFI_RX] Error processing app data: {e}')

    def destroy_node(self):
        """Clean up HTTP server on node shutdown"""
        self.get_logger().info('[WIFI_RX] Shutting down App WiFi Receiver...')
    
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server.server_close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AppWiFiReceiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
