import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleTrajectoryWaypoint, TrajectoryWaypoint
from math import pi, sin, cos


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Attributes
        self.mode = 0
        #Waypoint on takeoff
        self.waypoint_x = 0.0
        self.waypoint_y = 0.0
        self.waypoint_z = -5.0

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
        self.trajectort_wait_point = self.create_publisher(
            VehicleTrajectoryWaypoint, "fmu/in/vehicle_trajectory_waypoint", qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

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

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw = pi/2):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
    
    def publish_velocity_setpoint(self, v: float, theta: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        vz = 0.0
        vx = v*cos(theta)
        vy = v*sin(theta)
        msg.velocity = [vx, vy, vz]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocity setpoints {[vx, vy, vz]}")

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

    def publish_VHWP_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = VehicleTrajectoryWaypoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) 
        msg.type = 0

        # Solo se manda un trajectory waypoint
        TWP = TrajectoryWaypoint()
        TWP.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        TWP.position = [self.waypoint_x, self.waypoint_y, self.waypoint_z]
        TWP.velocity = [float('nan'), float('nan'), float('nan')]
        TWP.acceleration = [float('nan'), float('nan'), float('nan')]
        TWP.yaw = float('nan')
        TWP.yaw_speed = float('nan')
        TWP.point_valid = True
        TWP.type = 2
        msg.waypoints[0] = TWP

        # Los demas estan vacios
        TWP.timestamp = 0
        TWP.position = [float('nan'), float('nan'), float('nan')]
        TWP.point_valid = False
        TWP.type = 255
        for i in range(1,5):
            msg.waypoints[i] = TWP

        self.trajectort_wait_point.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_VHWP_heartbeat_signal()
        self.publish_offboard_control_heartbeat_signal()

        X = self.vehicle_local_position.x
        Y = self.vehicle_local_position.y
        Z = self.vehicle_local_position.z
        L = 50.0
        W = 50.0
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            match self.mode:
                case 0: #take off
                    self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, 0.0)
                    if abs(X) < 0.5 and abs(Y) < 0.5 and abs(Z + 5) < 0.5:
                        self.waypoint_x = L
                        self.mode = 1
                case 1: # (5, 0)
                    self.publish_position_setpoint(L, 0.0, self.takeoff_height, 0.0)
                    if abs(X - L) < 0.5 and abs(Y) < 0.5 and abs(Z + 5) < 0.5:
                        self.waypoint_y = L
                        self.mode = 2
                case 2: # (5, 5)
                    self.publish_position_setpoint(L, W, self.takeoff_height, pi/2)
                    if abs(X - L) < 0.5 and abs(Y - W) < 0.5 and abs(Z + 5) < 0.5:
                        self.waypoint_x = 0.0
                        self.mode = 3
                case 3: # (0, 5)
                    self.publish_position_setpoint(0.0, W, self.takeoff_height, pi)  
                    if abs(X) < 0.5 and abs(Y - W) < 0.5 and abs(Z + 5) < 0.5:
                        self.waypoint_y = 0.0
                        self.mode = 4  
                case 4: # (0, 0)
                    self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, -pi/2)  
                    if abs(X) < 0.5 and abs(Y) < 0.5 and abs(Z + 5) < 0.5:
                        self.waypoint_x = L
                        self.mode = 1
                case 5: # Landing
                    self.publish_position_setpoint(0.0, 0.0, self.takeoff_height, 0.0)
                    self.get_logger().info("Square made!")
                    self.mode = 1

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
