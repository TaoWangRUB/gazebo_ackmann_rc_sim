#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import math

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        
        # Subscriber for MAVROS state
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        
        # Publisher for setpoint position
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Service clients for arming and setting mode
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Timer for publishing setpoints
        self.dt = 0.2
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        # Variables
        self.current_state = State()
        self.offboard_setpoint_counter = 0

        # Circular trajectory parameters
        self.radius = 10.0  # Radius of the circle (meters)
        self.yaw_rate = 0.5  # Angular velocity (radians/second)
        self.center_x = 0.0  # X-coordinate of the circle center
        self.center_y = 0.0  # Y-coordinate of the circle center
        self.yaw = 0.0  # Current yaw angle
        self.pose = PoseStamped()

    def state_callback(self, msg):
        # Callback for MAVROS state
        self.current_state = msg

    def timer_callback(self):
        # Publish setpoints
        if self.current_state.connected:
            
            # Enter Offboard mode and arm the vehicle
            if self.offboard_setpoint_counter < 100:
                self.offboard_setpoint_counter += 1
            else:
                if not self.current_state.armed:
                    self.arm_vehicle()
                if self.current_state.mode != "OFFBOARD":
                    self.set_offboard_mode()
            
            if self.current_state.armed and self.current_state.mode == "OFFBOARD":
                # Calculate position along the circular trajectory
                self.yaw += self.yaw_rate * self.dt  # Update yaw angle
                x = self.center_x + self.radius * math.cos(self.yaw)
                y = self.center_y + self.radius * math.sin(self.yaw)
                z = 0.0  # Fixed altitude (2 meters)

                # Create a PoseStamped message for setpoint
                self.pose.header.stamp = self.get_clock().now().to_msg()
                self.pose.header.frame_id = 'map'
                self.pose.pose.position.x = x
                self.pose.pose.position.y = y
                self.pose.pose.position.z = z
                self.pose.pose.orientation.z = math.sin(self.yaw / 2)  # Yaw orientation
                self.pose.pose.orientation.w = math.cos(self.yaw / 2)
            
            # Publish the setpoint
            self.setpoint_pub.publish(self.pose)

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
    node = OffboardControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
