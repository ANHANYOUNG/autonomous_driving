#!/usr/bin/env python3
"""UWB Publisher Node"""
import rclpy
import threading
import queue
from rclpy.node import Node
import serial
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


class UWB_Publisher(Node):
    def __init__(self):
        super().__init__('uwb_publisher')
        self.absxy_pub = self.create_publisher(PoseWithCovarianceStamped, '/abs_xy', 1)
        self.raw_data_pub = self.create_publisher(String, '/uwb_raw_data', 1)

        # Parameters
        self.declare_parameter('port', '/dev/usb-left-bottom')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Data queue and thread control flag
        self.data_queue = queue.Queue()
        self.running = True

        try:
            self.ser = serial.Serial(port, baud, timeout=None)  # blocking mode
            self.get_logger().info(f'Opened serial port: {port} at {baud} baud')
            
            # Start reading serial data in a separate thread
            self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
            self.serial_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.ser = None

        # Timer for processing queue (fast cycle - check queue every 10ms)
        self.timer = self.create_timer(0.01, self.process_queue)

    def serial_reader(self):
        """Read serial data in real-time in a separate thread"""
        while self.running and self.ser and self.ser.is_open:
            try:
                # Read one line until newline character (blocking)
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Add data to queue (to be processed in main thread)
                    self.data_queue.put(line)
                    self.get_logger().debug(f'[UWB] Queued data: "{line}"')
            except Exception as e:
                if self.running:  # Log error only if not normal shutdown
                    self.get_logger().error(f'Serial read error: {e}')
                break

    def process_queue(self):
        """Process data from the queue (executed in main thread)"""
        processed_count = 0
        max_process_per_cycle = 10  # Process up to 10 items at once
        
        while not self.data_queue.empty() and processed_count < max_process_per_cycle:
            try:
                line = self.data_queue.get_nowait()
                self.process_serial_data(line)
                processed_count += 1
            except queue.Empty:
                break

    def process_serial_data(self, line):
        """Process actual data and publish ROS messages"""
        # Debugging log
        # self.get_logger().info(f'Processing data: "{line}"')

        # Publish raw data as String message
        # raw_data : "Anchor1_x, Anchor1_y, Anchor2_x, Anchor2_y, ..., Tag_x, Tag_y"
        raw_msg = String()
        raw_msg.data = line
        self.raw_data_pub.publish(raw_msg)
        self.get_logger().debug(f'[UWB] Published raw data: "{line}"')
        
        # Extract only the tag position and publish as PoseWithCovarianceStamped
        try:
            parts = line.split(',')
            # Last two values: pos_x, pos_y
            if len(parts) >= 2:
                # Extract Last two values as floats
                x = float(parts[-2].strip())  # Second last
                y = float(parts[-1].strip())  # Last
                
                # Create PoseWithCovarianceStamped message
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

                # Trust only x,y, set large covariance for others (ignored)
                p.pose.covariance = [
                    0.5, 0,    0,    0,    0,    0,
                    0,    0.5, 0,    0,    0,    0,
                    0,    0,    1e6,  0,    0,    0,
                    0,    0,    0,    1e6,  0,    0,
                    0,    0,    0,    0,    1e6,  0,
                    0,    0,    0,    0,    0,    1e6,
                ]
                self.absxy_pub.publish(p)
                # self.get_logger().info(f'Published position: x={x:.3f}, y={y:.3f}')   
            else:
                self.get_logger().warn(f'[UWB] Not enough data values: {len(parts)} values in "{line}"')
        except ValueError as e:
            self.get_logger().warn(f'[UWB] Failed to parse position data: {line}, error: {e}')

    def destroy_node(self):
        """Clean up threads on node shutdown"""
        self.running = False
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UWB_Publisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()