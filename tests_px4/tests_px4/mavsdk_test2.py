import rclpy
from rclpy.node import Node
import asyncio
from mavsdk import System

class MavSDKNode(Node):

    def __init__(self):
        super().__init__("mavsdk_test")
        asyncio.run(self.run())

    async def run(self):
        drone = System()
        await drone.connect(system_address="udp://:14540")
        status_text_task = asyncio.ensure_future(self.print_status_text(drone))

        async for state in drone.core.connection_state():
            if state.is_connected:
                print("Connected")
                break
        
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("Todo bn")
                break

        print("Takeoff")
        await drone.action.arm()
        await drone.action.takeoff()
        await asyncio.sleep(120)

        print("Landing")
        await drone.action.land()

        status_text_task.cancel()

    async def print_status_text(self, drone):
        try: 
            async for status_text in drone.telemetry.status_text():
                print(f"Status: {status_text.type}: {status_text.text}")
        except asyncio.CancelledError:
            return

def main(args=None):
    rclpy.init(args=args)
    node = MavSDKNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()