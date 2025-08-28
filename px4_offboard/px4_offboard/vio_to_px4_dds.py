#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

import numpy as np
from time import time
from tf_transformations import quaternion_multiply, quaternion_from_euler, euler_from_quaternion

import threading  # Import threading for lock

class VIOPublisher(Node):
    def __init__(self):
        super().__init__('vio_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        #self.odom_sub = self.create_subscription(Odometry, '/camera/pose/sample', self.odom_callback, qos_profile)
        # self.imu_sub = self.create_subscription(Imu, '/camera/imu', self.imu_callback, 10)
        self.px4_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        
        self.is_publishing = False
        # Keep track of the last message received time
        self.last_odom_time = time()  
        self.odom_timeout = 5.0  # Timeout threshold in seconds
        
        # Timer to check subscription status every 1 second
        self.timer = self.create_timer(1.0, self.check_odom_status)
        
        # Timer to check subscription status every 1 second
        self.timer_output = self.create_timer(0.02, self.pub_px4_odom)
        
        # Store received odom
        self.px4_odom = VehicleOdometry()
        
        self.lock = threading.Lock()

    def odom_callback(self, msg):
        """
        Callback to convert ROS 2 Odometry to PX4 VehicleOdometry message.
        """
        # directly forwarding
        if not self.is_publishing:
            self.get_logger().info(f'{self.odom_sub.topic_name} is forwareded to {self.px4_pub.topic_name}')
            self.is_publishing = True
        
        self.px4_odom.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # PX4 expects time in microseconds
        self.px4_odom.timestamp_sample = int(msg.header.stamp.nanosec / 1000) + (msg.header.stamp.sec * 1_000_000)

        # Update last received time
        self.last_odom_time = time()
        
        # set position frame
        self.px4_odom.pose_frame = VehicleOdometry.POSE_FRAME_NED  # PX4 uses NED frame
        # Convert position (ENU -> NED)
        self.px4_odom.position[0] = -msg.pose.pose.position.x
        self.px4_odom.position[1] = -msg.pose.pose.position.y
        self.px4_odom.position[2] = -msg.pose.pose.position.z
        # self.get_logger().info('x = %f, y = %f, z = %f' %(px4_odom.position[0], px4_odom.position[1], px4_odom.position[2]))
        
        # Set position variance
        self.px4_odom.position_variance[0] = 0.01  # Variance in X position (m^2)
        self.px4_odom.position_variance[1] = 0.01  # Variance in Y position (m^2)
        self.px4_odom.position_variance[2] = 0.01  # Variance in Z position (m^2)
        
        # Convert orientation (ENU -> NED)
        q_blu = [-msg.pose.pose.orientation.x,
                 -msg.pose.pose.orientation.y,
                 -msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w]
	
        # Step 1: 180° X-rotation (BLU -> FRU)
        q_rot_x = quaternion_from_euler(np.pi, 0, 0)  # [1, 0, 0, 0]
        
        q_frd = quaternion_multiply(q_rot_x, q_blu)

        # Reorder to PX4's [w,x,y,z] format
        self.px4_odom.q = [q_blu[3], q_blu[0], q_blu[1], q_blu[2]]
        #self.get_logger().info('x = %f, y = %f, z = %f w = %f' %(q_blu[0], q_blu[1], q_blu[2], q_blu[3]))
        #self.get_logger().warn('x = %f, y = %f, z = %f w = %f' %(q_frd[0], q_frd[1], q_frd[2], q_frd[3]))
        
        # Set orientation variance
        self.px4_odom.orientation_variance[0] = 0.01  # Variance in roll (rad^2)
        self.px4_odom.orientation_variance[1] = 0.01  # Variance in pitch (rad^2)
        self.px4_odom.orientation_variance[2] = 0.01  # Variance in yaw (rad^2)
        
        # Convert velocity (ENU -> NED)
        # set position frame
        self.px4_odom.pose_frame = VehicleOdometry.VELOCITY_FRAME_NED  # PX4 uses NED frame
        
        self.px4_odom.velocity[0] = -msg.twist.twist.linear.x
        self.px4_odom.velocity[1] = -msg.twist.twist.linear.y
        self.px4_odom.velocity[2] = -msg.twist.twist.linear.z
        
        # Set velocity frame
        self.px4_odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED  # PX4 uses NED frame
        # Set velocity variance
        self.px4_odom.velocity_variance[0] = 0.01  # Variance in X velocity (m^2/s^2)
        self.px4_odom.velocity_variance[1] = 0.01  # Variance in Y velocity (m^2/s^2)
        self.px4_odom.velocity_variance[2] = 0.01  # Variance in Z velocity (m^2/s^2)
        
        # Convert angular velocity (ENU -> NED)
        self.px4_odom.angular_velocity[0] = -msg.twist.twist.angular.x
        self.px4_odom.angular_velocity[1] = -msg.twist.twist.angular.y
        self.px4_odom.angular_velocity[2] = -msg.twist.twist.angular.z
        
        # set signal quality
        self.px4_odom.quality = 100
    
    def pub_px4_odom(self):
        # Publish to PX4
        with self.lock:  # Ensure thread-safe access
            self.px4_odom.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # PX4 expects time in microseconds
            self.px4_pub.publish(self.px4_odom)
        
    def check_odom_status(self):
        # Check if /odom topic is still alive by checking the last received time
        if time() - self.last_odom_time > self.odom_timeout and self.is_publishing:
            self.get_logger().warning('/odom topic has not published for {:.2f} seconds.'.format(time() - self.last_odom_time))
            self.is_publishing = False
        else:
            pass

def main():
    rclpy.init()
    node = VIOPublisher()
    print("spin node!")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
