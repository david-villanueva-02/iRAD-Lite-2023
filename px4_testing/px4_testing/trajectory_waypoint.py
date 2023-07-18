import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleTrajectoryWaypoint, TrajectoryWaypoint, TelemetryStatus


class TrajectoryWaypointNode(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('trajectory_waypoint')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.trajectory_waypoint_publisher = self.create_publisher(VehicleTrajectoryWaypoint,"/fmu/in/vehicle_trajectory_waypoint",qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.heartbeat_publisher = self.create_publisher(TelemetryStatus,"/fmu/in/telemetry_status",qos_profile)

        # Create subscribers
        self.vehicle_trajectory_waypoint_desired_subscriber = self.create_subscription(VehicleTrajectoryWaypoint,"/fmu/out/vehicle_trajectory_waypoint_desired",self.callback_trajectory_waypoint_desired,qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.trajectory_waypoint_desired = VehicleTrajectoryWaypoint()
        self.counter = 0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback_trajectory_waypoint_desired(self,msg):
        self.trajectory_waypoint_desired = msg

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

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

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

    def publish_trajectory_waypoint(self):
        msg = VehicleTrajectoryWaypoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.type = 0
        msg.waypoints[0].timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.waypoints[0].position = self.trajectory_waypoint_desired.waypoints[2].position
        msg.waypoints[0].point_valid = True
        msg.waypoints[0].type = 0
        self.trajectory_waypoint_publisher.publish(msg)

    def publish_heartbeat(self):
        msg = TelemetryStatus()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.heartbeat_type_onboard_controller = True        
        msg.heartbeat_component_obstacle_avoidance = True
        msg.avoidance_system_healthy = True
        self.heartbeat_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_trajectory_waypoint()
        self.publish_heartbeat()

        if self.counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_MISSION_START,param1=1.0)
            print(self.trajectory_waypoint_desired.waypoints[2].position)
            pass

        if self.counter < 10:
            self.counter += 1

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryWaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
