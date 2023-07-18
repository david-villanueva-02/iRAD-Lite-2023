import rclpy
from rclpy.node import Node
import asyncio
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from px4_msgs.msg import VehicleLocalPosition, VehicleGlobalPosition

class MavSDKNode(Node):

    def __init__(self):
        super().__init__("mavsdk_test")
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_global_position = VehicleGlobalPosition()

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)
        
        asyncio.run(self.run())

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
    
    def vehicle_global_position_callback(self, vehicle_global_position):
        self.vehicle_global_position = vehicle_global_position

    async def run(self):
        drone = System()
        await drone.connect(system_address="udp://:14540")
        self.get_logger().info("Waiting for drone to connect...")
        #status_text_task = asyncio.ensure_future(self.print_status_text(drone))

        async for state in drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Connnected")
                break

        print_mission_progress_task = asyncio.ensure_future(self.print_mission_progress(drone))
        running_tasks = [print_mission_progress_task]
        termination_task = asyncio.ensure_future(self.observe_is_in_air(drone, running_tasks))

        mission_item1 = MissionItem(42.0 + 0.00009,
            -84 + 0.00009,
            10.0,
            10.0,
            True, 
            float('nan'),
            float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'),
            float('nan'),
            0.5,
            float('nan'),
            float('nan')
        )

        mission_item2 = MissionItem(42.0,
            -84.0,
            10.0,
            10.0,
            True, 
            float('nan'),
            float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'),
            float('nan'),
            0.5,
            float('nan'),
            float('nan')
        )

        mission_items = [mission_item1, mission_item2] 
        mission_plan = MissionPlan(mission_items)

        await drone.mission.set_return_to_launch_after_mission(True)
        await drone.mission.upload_mission(mission_plan)

        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("todo bn")
                break

        await drone.action.arm()
        await drone.mission.start_mission() 
        await termination_task

        if drone.mission.is_mission_finished():
            await drone.action.land()
            
    async def print_mission_progress(self, drone):
        async for mission_progress in drone.mission.mission_progress():
            self.get_logger().info(f"Mission progress: " + f"{mission_progress.current}/" + f"{mission_progress.total}")
        
    async def observe_is_in_air(self, drone, running_tasks):
        was_in_air = False
        async for is_in_air in drone.telemetry.in_air():
            if is_in_air:
                was_in_air = is_in_air
            
            if was_in_air and not is_in_air:
                for task in running_tasks:
                    task.cancel()
                    try: 
                        await task
                    except asyncio.CancelledError:
                        pass
                
                await asyncio.get_event_loop().shutdown_asyncgens()
                return
            
    async def print_status_text(self, drone):
        try: 
            async for status_text in drone.telemetry.status_text():
                self.get_logger().info(f"Status: {status_text.type}: {status_text.text}")
        except asyncio.CancelledError:
            return

def main(args=None):
    rclpy.init(args=args)
    node = MavSDKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()