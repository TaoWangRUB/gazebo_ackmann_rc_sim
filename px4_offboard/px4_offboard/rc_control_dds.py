import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode, ActuatorMotors, VehicleAttitudeSetpoint, VehicleStatus, VehicleRatesSetpoint
import numpy as np

from enum import Enum, auto

# Define the enum
class OffboardControlFlag(Enum):
    POSITION = auto()
    VELOCITY = auto()
    ACCELERATION = auto()
    ATTITUDE = auto()
    BODY_RATE = auto()
    THRUST_AND_TORQUE = auto()
    DIRECT_ACTUATOR = auto()

# Define the global constant mapping
FLAG_MAPPING = {
    OffboardControlFlag.POSITION: "position",
    OffboardControlFlag.VELOCITY: "velocity",
    OffboardControlFlag.ACCELERATION: "acceleration",
    OffboardControlFlag.ATTITUDE: "attitude",
    OffboardControlFlag.BODY_RATE: "body_rate",
    OffboardControlFlag.THRUST_AND_TORQUE: "thrust_and_torque",
    OffboardControlFlag.DIRECT_ACTUATOR: "direct_actuator"
}

class PX4OffboardControl(Node):
    def __init__(self):
        super().__init__('px4_offboard_control')
        
        # Publishers for sending commands to PX4
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        # offboard heartbeat publisher
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        # vehicle command publisher via MavLink
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # setpointer publisher
        # control via motor directly
        self.publisher_actuator = self.create_publisher(
            ActuatorMotors,'/fmu/in/actuator_motors', qos_profile)
        
        # control via trajectory setpoint
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # control via attitude setpoint
        self.publisher_attitude = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        
        # control via body rate setpoint
        self.publisher_bodyrate = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        
        # subscribe to vehicle status
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.publish_velocity_setpoint)
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Parameters (Adjust as needed)
        self.offboard_setpoint_counter = 0
        self.wheelbase = 0.5  # Wheelbase length in meters
        self.speed = 5.0  # Speed in m/s
        self.steering_angle = 0.2  # Steering angle in radians
        self.yaw = 0.0  # Initial yaw angle
        
        self.actuators = ActuatorMotors()
        self.THROTTLE_CHANNEL = 2   # throttle channel
        self.STEERING_CHANNEL = 0   # steering channel
	    
	    # PWM configuration (adjust based on your RC/controller setup)
        self.PWM_MIN = 1000    # Minimum PWM value (full reverse/left)
        self.PWM_NEUTRAL = 1500  # Center/neutral position
        self.PWM_MAX = 2000    # Maximum PWM value (full forward/right)
        self.PWM_RANGE = self.PWM_MAX - self.PWM_NEUTRAL  # 500 for symmetric control
        self.V_MAX = 5       # maximum allowed speed
        self.STEER_MAX = 30  # maximum allowed steering angle
        
        # Trajectory setup
        self.declare_parameter('radius', 50.0)
        self.declare_parameter('omega', 0.1)
        self.declare_parameter('altitude', 5.0)
        
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value
        self.get_logger().info("initializing done")
    
    def vehicle_status_callback(self, msg):
        """
        vehicle status callback to check if vehicle is armed and in which mode
        """
        if(self.nav_state != msg.nav_state):
            self.get_logger().info("offboard status: change from %d to %d" %(self.nav_state, msg.nav_state))
            self.nav_state = msg.nav_state
        if(self.arming_state != msg.arming_state):
            self.get_logger().info("arm status: change from %d to %d" %(self.arming_state, msg.arming_state))
            self.arming_state = msg.arming_state
        
    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")
    
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
    
    def disarm(self):
        """Send an disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Arm command sent')

    def actuator_control(self, steering, throttle):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, param1=steering, param3=throttle)
    
    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def publish_offboard_control_heartbeat_signal(self, flag_selector: OffboardControlFlag):
        """
        Publish the offboard control mode.
        
        :param flag_selector: An OffboardControlFlag enum value to select which flag to set to True.
        """
        msg = OffboardControlMode()
        
        # Initialize all flags to False
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        
        # Use the global FLAG_MAPPING constant
        if flag_selector not in FLAG_MAPPING:
            raise ValueError(f"Invalid flag_selector: {flag_selector}. Must be one of {list(FLAG_MAPPING.keys())}")
        
        # Set the corresponding flag to True
        setattr(msg, FLAG_MAPPING[flag_selector], True)
        
        # Set the timestamp
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Publish the message
        self.offboard_control_mode_publisher.publish(msg)
    
    def calculate_ackermann_actuators(self, speed, steering_angle):
        """Convert PWM inputs to normalized actuator values with full bi-directional control"""
        
        # Convert throttle PWM to [-1, 1]
        # PWM: 1000 (full reverse) -> 1500 (stop) -> 2000 (full forward)
        throttle_pwm = self.PWM_NEUTRAL + 400 * speed / self.V_MAX
        throttle = (throttle_pwm - self.PWM_NEUTRAL) / self.PWM_RANGE
        throttle = np.clip(throttle, -1.0, 1.0)

        # Convert steering PWM to [-1, 1] 
        # PWM: 1000 (full left) -> 1500 (center) -> 2000 (full right)
        steering_pwm = self.PWM_NEUTRAL + 400 * steering_angle / self.STEER_MAX
        steering = (steering_pwm - self.PWM_NEUTRAL) / self.PWM_RANGE
        steering = np.clip(steering, -1.0, 1.0)

        return throttle, steering
    
    def publish_actuator_setpoint(self):
        """ 
        timer callback to send actuator setpoint
        Actuator Output has to be set for correct channel that been connected with servo/motor
        peripheral via Actuator Set X should be selected
        """ 
        
        # offboard streaming should be available before arming
        self.publish_offboard_control_heartbeat_signal(OffboardControlFlag.DIRECT_ACTUATOR)
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # Compute actuators control value [-1, 1]
            # Calculate actuator outputs
            throttle, steering = self.calculate_ackermann_actuators(self.speed, self.steering_angle)  # <---

            # Set actuator values (adjust channels based on your mixer)
            self.actuators.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.actuators.timestamp_sample = int(self.get_clock().now().nanoseconds / 1000)
            self.actuators.control[self.THROTTLE_CHANNEL] = throttle  # <---
            self.actuators.control[self.STEERING_CHANNEL] = steering  # <---
            #self.publisher_actuator.publish(self.actuators)
            self.actuator_control(-0.5, 0.2)
            #self.get_logger().info("throttle = %f, steering = %f" % (throttle, steering) )
	
    def publish_trajectory_setpoint(self):
        """ 
        timer callback to send trajectory setpoint in NED frame
        """ 
        # offboard streaming should be available before arming
        self.publish_offboard_control_heartbeat_signal(OffboardControlFlag.POSITION)
        
        # publish setpoint in offboard mode and vehicle is armed
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            #trajectory_msg.position[2] = -self.altitude 
            #trajectory_msg.velocity[0] = self.omega * self.radius
            #trajectory_msg.yaw = self.theta
            trajectory_msg.yawspeed = self.omega
            self.publisher_trajectory.publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.dt

    def publish_velocity_setpoint(self):
        """ 
        timer callback to send trajectory setpoint in NED frame
        """ 
        # offboard streaming should be available before arming
        self.publish_offboard_control_heartbeat_signal(OffboardControlFlag.VELOCITY)
        
        # publish setpoint in offboard mode and vehicle is armed
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            speed = self.omega * self.radius
            trajectory_msg.velocity[0] = speed * np.cos(self.theta)
            trajectory_msg.velocity[1] = speed * np.sin(self.theta)
            #trajectory_msg.yaw = self.theta
            #trajectory_msg.yawspeed = self.omega
            self.publisher_trajectory.publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.dt
            
    def publish_attitude_setpoint(self):
        """ 
        timer callback to send attitude setpoint in local odom frame
        """
        # offboard streaming should be available before arming
        self.publish_offboard_control_heartbeat_signal(OffboardControlFlag.ATTITUDE)
        
        # publish setpoint in offboard mode and vehicle is armed
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            
            msg = VehicleAttitudeSetpoint()

            # Set the timestamp (in microseconds)
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            # Set a desired quaternion.
            # For a no-rotation (neutral attitude) the quaternion is identity: [w, x, y, z] = [1, 0, 0, 0].
            msg.q_d = [1.0, 0.0, 0.0, 0.0]
            
            # Set yaw rate setpoint (rad/s); adjust this value as needed.
            msg.yaw_sp_move_rate = 0.0
            
            # Set thrust command in the body frame.
            # For multicopters, typically thrust_body[2] is negative throttle demand.
            # For a rover, adjust these values based on your vehicle configuration.
            msg.thrust_body = [0.0, 0.0, 0.0]
            
            # Optional flags:
            msg.reset_integral = False
            msg.fw_control_yaw_wheel = False
            
            # pub setpoint
            self.publisher_attitude.publish(msg)
	
    def publish_bodyrate_setpoint(self):
        """ 
        timer callback to send body setpoint in FRD frame
        """
        # offboard streaming should be available before arming
        self.publish_offboard_control_heartbeat_signal(OffboardControlFlag.BODY_RATE)
        
        # publish setpoint in offboard mode and vehicle is armed
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and \
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            
            msg = VehicleRatesSetpoint()
            # Set the timestamp (in microseconds)
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            
            # Set desired angular rates in rad/s (adjust as needed)
            msg.roll  = 0.0  # Commanded roll rate
            msg.pitch = 0.0  # Commanded pitch rate
            msg.yaw   = self.omega  # Commanded yaw rate
            
            # Set desired throttle in FxRyDz frame
            msg.thrust_body = [0.5, 0.0, 0.0]
            
            # Publish the rate setpoint
            self.publisher_bodyrate.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = PX4OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

