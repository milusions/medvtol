import json
import asyncio
import threading
import websockets
import rclpy
from rclpy.node import Node
from medvtol_interfaces.msg import Telemetry


class MedVTOLControlPanel(Node):
    def __init__(self):
        super().__init__('telemetry_websocket_node')
        self.subscription = self.create_subscription(
            Telemetry,
            '/telemetry',
            self.telemetry_callback,
            10
        )
        self.latest_telemetry = {}
        self.get_logger().info("WebSocket Server starting on ws://localhost:8765...")

    def telemetry_callback(self, msg):
        """Packs ROS 2 message data into a Python dictionary for JSON serialization."""
        self.latest_telemetry = {
            "x": round(msg.x, 2),
            "y": round(msg.y, 2),
            "z": round(msg.z, 2),
            "lat": round(msg.lat, 6),
            "lon": round(msg.lon, 6),
            "altitude": round(msg.altitude, 2),
            "vx": round(msg.vx, 3),
            "vy": round(msg.vy, 3),
            "vz": round(msg.vz, 3),
            "flight_stage": msg.flight_stage,
            "flight_mode": msg.flight_mode
        }


async def websocket_handler(websocket, path, node):
    """Continuously streams the latest telemetry data at 10 Hz."""
    while True:
        if node.latest_telemetry:
            await websocket.send(json.dumps(node.latest_telemetry))
        await asyncio.sleep(0.1) 


def start_websocket_server(node):
    """Sets up and runs the asyncio event loop for the WebSocket server."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    start_server = websockets.serve(lambda ws, path: websocket_handler(ws, path, node), "0.0.0.0", 8765)
    loop.run_until_complete(start_server)
    loop.run_forever()


def main(args=None):
    rclpy.init(args=args)
    node = MedVTOLControlPanel()

    # Run the WebSocket server in a daemon thread so it closes when ROS shuts down
    ws_thread = threading.Thread(target=start_websocket_server, args=(node,), daemon=True)
    ws_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Telemetry Streamer.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()