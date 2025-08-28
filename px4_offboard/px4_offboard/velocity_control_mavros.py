#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import math

class RoverControl(Node):
    def __init__(self):
        super().__init__('rover_control')
        
        # Subscriber for MAVROS state
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        
        # Publisher for velocity setpoints
        #self.velocity_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.velocity_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Service clients for arming and setting mode
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Timer for publishing velocity setpoints
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        # Variables
        self.current_state = State()
        self.offboard_setpoint_counter = 0

        # Rover control parameters
        self.speed = 1.0  # Forward speed (m/s)
        self.yaw_rate = 0.5  # Yaw rate (rad/s)
        self.twist = Twist()
        #self.twist = TwistStamped()

    def state_callback(self, msg):
        # Callback for MAVROS state
        self.current_state = msg

    def timer_callback(self):
        # Publish velocity setpoints
        if self.current_state.connected:
            
            # Enter Offboard mode and arm the vehicle
            if self.offboard_setpoint_counter < 100:
                self.offboard_setpoint_counter += 1
            else:
                if not self.current_state.armed:
                    self.arm_vehicle()
                if self.current_state.mode != 'OFFBOARD':
                    self.set_offboard_mode()
            
            # Create a Twist message for velocity setpoint
            if self.current_state.armed and self.current_state.mode == 'OFFBOARD':
                #self.twist.header.stamp = self.get_clock().now().to_msg()
                #self.twist.header.frame_id = 'base_link'
                self.twist.linear.x = self.speed  # Forward speed
                self.twist.angular.z = self.yaw_rate  # Yaw rate (steering)
            
            # Publish the velocity setpoint
            self.velocity_pub.publish(self.twist)

    def arm_vehicle(self):
        # Arm the vehicle
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        
        request = CommandBool.Request()
        request.value = True
        future = self.arming_client.call_async(request)
        future.add_done_callback(self.arming_response_callback)

    def arming_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Vehicle armed!')
            else:
                self.get_logger().warn('Failed to arm vehicle.')
        except Exception as e:
            self.get_logger().error('Service call ARM failed: %r' % (e,))

    def set_offboard_mode(self):
        # Set Offboard mode
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')
        
        request = SetMode.Request()
        request.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self.set_mode_response_callback)

    def set_mode_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('Offboard mode enabled!')
            else:
                self.get_logger().warn('Failed to enable Offboard mode.')
        except Exception as e:
            self.get_logger().error('Service call OFFBOARD_MODE failed: %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = RoverControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
