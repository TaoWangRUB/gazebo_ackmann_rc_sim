import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode
import numpy as np

class PX4OffboardControl(Node):
    def __init__(self):
        super().__init__('px4_offboard_control')
        
        # Publishers for sending commands to PX4
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.publish_setpoint)
        self.offboard_started = False

        # Parameters (Adjust as needed)
        self.offboard_setpoint_counter = 0
        self.wheelbase = 0.5  # Wheelbase length in meters
        self.speed = 5.0  # Speed in m/s
        self.steering_angle = 0.2  # Steering angle in radians
        self.yaw = 0.0  # Initial yaw angle
    
    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.offboard_started = True
        
        self.get_logger().info("Switching to offboard mode")
    
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
    
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
    
    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
    
    def publish_setpoint(self):
        """ timer callback """ 
        
        # offboard streaming should be available before arming
        self.publish_offboard_control_heartbeat_signal()
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        
        if self.offboard_started:
            msg = TrajectorySetpoint()
            # Compute velocity components in NED frame
            vx = self.speed * np.cos(self.yaw)
            vy = self.speed * np.sin(self.yaw)
            
            # Compute yaw rate from steering angle
            yaw_rate = (self.speed / self.wheelbase) * np.tan(self.steering_angle)
            # Update yaw estimate
            self.yaw += yaw_rate * self.dt

            #msg.position = [100.0, 0.0, 0.0]  # Move forward 100m
            msg.velocity = [vx, vy, 0.0]  # Move forward at 1 m/s
            msg.yawspeed = yaw_rate  # 0.5 radians (~28.6 degrees)
            self.setpoint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PX4OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

