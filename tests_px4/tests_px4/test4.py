import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from example_interfaces.msg import String
import math


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.keyboard_subscriber = self.create_subscription(String,"keyboard",self.keyboard_callback,10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.keyboard = String()
        self.mode = 0
        self.v = 5.0
        self.yawspeed = math.pi/2

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def keyboard_callback(self,keyboard):
        self.keyboard = keyboard

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def takeoff(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,param5=self.vehicle_local_position.ref_lat,param6=self.vehicle_local_position.ref_lon,param7=self.vehicle_local_position.ref_alt+5)
        self.get_logger().info("Switching to takeoff mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_movement_setpoint(self, x, y, z, yaw):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z]
        msg.velocity = [x, y, z]
        msg.yaw = self.vehicle_local_position.heading
        msg.yawspeed = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

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

    def timer_callback(self) -> None:
        
        self.publish_offboard_control_heartbeat_signal()

        if self.keyboard.data == "arm":
            self.arm()
            self.keyboard.data = "none"

        elif self.keyboard.data == "takeoff":
            self.takeoff()
            self.keyboard.data = "none"

        elif self.keyboard.data == "land":
            self.land()
            self.keyboard.data = "none"

        elif self.keyboard.data == "offboard":
            self.engage_offboard_mode()
            self.keyboard.data = "none"

        elif self.keyboard.data == "return":
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            self.keyboard.data = "none"

        elif self.keyboard.data == "plus":
            if self.v < 10.0:
                self.v += 0.5
            print("Speed: ", self.v)
            self.keyboard.data = "none"
        
        elif self.keyboard.data == "minus":
            if self.v > 0.0:
                self.v -= 0.5
            print("Speed:", self.v)
            self.keyboard.data = "none"
        
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            
            if self.keyboard.data == "right":
                vx = self.v * math.cos(self.vehicle_local_position.heading + math.pi/2)
                vy = self.v * math.sin(self.vehicle_local_position.heading + math.pi/2)
                self.publish_movement_setpoint(vx,vy, 0.0, 0.0)

            elif self.keyboard.data == "left":
                vx = -self.v * math.cos(self.vehicle_local_position.heading + math.pi/2)
                vy = -self.v * math.sin(self.vehicle_local_position.heading + math.pi/2)
                self.publish_movement_setpoint(vx,vy, 0.0, 0.0)

            elif self.keyboard.data == "front":
                vx = self.v * math.cos(self.vehicle_local_position.heading)
                vy = self.v * math.sin(self.vehicle_local_position.heading)
                self.publish_movement_setpoint(vx, vy, 0.0, 0.0)

            elif self.keyboard.data == "back":
                vx = -self.v * math.cos(self.vehicle_local_position.heading)
                vy = -self.v * math.sin(self.vehicle_local_position.heading)
                self.publish_movement_setpoint(vx, vy, 0.0, 0.0)

            elif self.keyboard.data == "up":
                self.publish_movement_setpoint(0.0, 0.0, -2.0, 0.0)

            elif self.keyboard.data == "down":
                self.publish_movement_setpoint(0.0, 0.0, 2.0, 0.0)

            elif self.keyboard.data == "rotate_right":
                self.publish_movement_setpoint(0.0, 0.0, 0.0, math.pi/4)

            elif self.keyboard.data == "rotate_left":
                self.publish_movement_setpoint(0.0, 0.0, 0.0, -math.pi/4)

            elif self.keyboard.data == "stop":
                self.publish_movement_setpoint(0.0, 0.0, 0.0, 0.0)

            self.keyboard.data = "stop"
            
def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)