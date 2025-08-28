import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class Twist2RcOverride(Node):

    def __init__(self):
        super().__init__('rcoverride_publisher')
        # Subscriber for MAVROS state
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        
        # Publisher for RC override
        self.rc_override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        self.twistsubscription = self.create_subscription(Twist, '/twist', self.twistcallback, 10)
        
        # Service clients for arming and setting mode
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Timer for publishing RC override commands
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        ### <---- Parameters ---- >
        self.steeringMin = 1100
        self.steeringMax = 1900
        self.throttleMin = 1100
        self.throttleMax = 1900
        self.angularVelocityMax = 0.5
        self.angularVelocityMin = (-1)*self.angularVelocityMax
        self.velocityMax = 2.78
        self.velocityMin = (-1)*self.velocityMax
        
        # Initializing
        self.rc_override = OverrideRCIn()
        self.rc_override.channels[0] = 1500  # Roll (neutral) yaw for rover
        self.rc_override.channels[1] = 1500  # Pitch (neutral)
        self.rc_override.channels[2] = 1500  # Throttle (slightly up)
        self.rc_override.channels[3] = 1500  # Yaw (slightly left)
        self.rc_override.channels[4] = 1500  # Aux 1 (neutral)
        self.rc_override.channels[5] = 1500  # Aux 2 (neutral)
        self.rc_override.channels[6] = 1500  # Aux 3 (neutral)
        self.rc_override.channels[7] = 1500  # Aux 4 (neutral)

        self.current_state = State()
        self.offboard_setpoint_counter = 0

        # RC override parameters
        self.yaw = 0
        self.vx = 0
        self.throttle_pwm = 1600  # Neutral throttle
        self.yaw_pwm = 1400  # Neutral yaw

    def state_callback(self, msg):
        # Callback for MAVROS state
        self.current_state = msg

    def timer_callback(self):
        # Publish RC override commands
        if self.current_state.connected:
            # Create an OverrideRCIn message
            self.rc_override.channels[0] = self.yaw_pwm  # Yaw
            self.rc_override.channels[2] = self.throttle_pwm  # Throttle
            
            # Publish the RC override command
            self.rc_override_pub.publish(self.rc_override)
            
            # Enter Offboard mode and arm the vehicle
            if self.offboard_setpoint_counter < 100:
                self.offboard_setpoint_counter += 1
            else:
                if not self.current_state.armed:
                    self.arm_vehicle()
            #    if self.current_state.mode != 'OFFBOARD':
            #        self.set_offboard_mode()
        
    def twistcallback(self, msg):
        self.get_logger().debug('I heard: %s' %(str(msg)))
        self.yaw, self.vx = msg.angular.z, msg.linear.x
        self.to_pwm(self.steering, self.throttle)
    
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
    
    def to_pwm(self, yaw, throttle):
        self.yaw_pwm = 1500 + 400 * self.yaw / self.angularVelocityMax
        self.throttle_pwm = 1500 + 400 * self.vx / self.velocityMax
        if self.yaw_pwm < self.steeringMin:
            self.yaw_pwm = self.steeringMin
        elif self.yaw_pwm > self.steeringMax:
            self.yaw_pwm = self.steeringMax
        if self.throttle_pwm < self.throttleMin:
            self.throttle_pwm = self.throttleMin
        elif self.throttle_pwm > self.throttleMax:
            self.throttle_pwm = self.throttleMax
        return


def main(args=None):
    rclpy.init(args=args)
    node = Twist2RcOverride()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
