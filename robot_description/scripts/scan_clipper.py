#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanClipper(Node):
    def __init__(self):
        super().__init__('scan_clipper')
        
        # Declare parameters
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('input_topic', '/rplidar/scan')
        self.declare_parameter('output_topic', '/scan')
        
        # Get parameters
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
        # Create publisher and subscriber
        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            10
        )
        
        self.get_logger().info(f'Scan Clipper started: {self.input_topic} -> {self.output_topic}')
        self.get_logger().info(f'Range limits: [{self.min_range}, {self.max_range}]')
        
    def scan_callback(self, msg):
        """
        Callback function to process laser scan data
        Replaces inf values with max_range and clips values to valid range
        """
        # Convert to numpy array for efficient processing
        ranges = np.array(msg.ranges)
        
        # Replace inf and nan values with max_range
        ranges[np.isinf(ranges) | np.isnan(ranges)] = self.max_range
        
        # Clip values to valid range
        ranges = np.clip(ranges, self.min_range, self.max_range)
        
        # Update message
        #msg.ranges = ranges.tolist()
        #msg.range_min = self.min_range
        #msg.range_max = self.max_range
        
        # Publish filtered scan
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    scan_clipper = ScanClipper()
    
    try:
        rclpy.spin(scan_clipper)
    except KeyboardInterrupt:
        pass
    finally:
        scan_clipper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()