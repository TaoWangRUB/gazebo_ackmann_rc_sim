#!/usr/bin/env python3
# imu_covariance_injector.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuCovarianceInjector(Node):
    def __init__(self):
        super().__init__('imu_covariance_injector')
        # Declare parameters
        self.declare_parameter('input_topic', '/l515/imu/raw')
        self.declare_parameter('output_topic', '/l515/imu/data')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
        self.sub = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            10)

        self.pub = self.create_publisher(
            Imu,
            self.output_topic,  # Output topic with fixed covariances
            10)

    def imu_callback(self, msg):
        # Set non-zero covariances
        msg.orientation_covariance = [0.01, 0.0, 0.0,
                                      0.0, 0.01, 0.0,
                                      0.0, 0.0, 0.01]

        msg.angular_velocity_covariance = [0.001, 0.0, 0.0,
                                           0.0, 0.001, 0.0,
                                           0.0, 0.0, 0.001]

        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                              0.0, 0.01, 0.0,
                                              0.0, 0.0, 0.01]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceInjector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
