import os
import json
import math
import threading
import http.server
import socketserver
import rclpy
from rclpy.node import Node
from medvtol_interfaces.srv import MissionCommand

# World origin from Gazebo spherical_coordinates
WORLD_LAT0 = 47.397971057728974
WORLD_LON0 = 8.546163739800146
WORLD_ALT0 = 0.0
EARTH_RADIUS = 6378137.0


def gazebo_to_gps(gz_x: float, gz_y: float, gz_z: float):
    """Converts Gazebo ENU coordinates (meters) to WGS84 GPS (degrees)."""
    d_lat = (gz_y / EARTH_RADIUS) * (180.0 / math.pi)
    d_lon = (gz_x / (EARTH_RADIUS * math.cos(math.radians(WORLD_LAT0)))) * (180.0 / math.pi)

    target_lat = WORLD_LAT0 + d_lat
    target_lon = WORLD_LON0 + d_lon
    target_alt = WORLD_ALT0 + gz_z

    return target_lat, target_lon, target_alt


class CustomWebHandler(http.server.SimpleHTTPRequestHandler):
    """Handles standard web traffic and custom API POST requests."""
    
    def translate_path(self, path):
        if path == '/':
            path = '/mevtol_control_panel.html'
        return super().translate_path(path)

    def do_POST(self):
        """Catches HTTP POST requests from the dashboard buttons."""
        if self.path == '/api/mission':
            # Parse the incoming JSON data
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            # Convert Gazebo coords to GPS
            target_lat, target_lon, target_alt = gazebo_to_gps(
                float(data['x']), float(data['y']), float(data['z'])
            )
            
            # Construct ROS 2 Service Request
            req = MissionCommand.Request()
            req.target_lat = target_lat
            req.target_lon = target_lon
            req.target_alt = target_alt
            req.cruise_alt = float(data['cruise_alt'])
            req.abort = False
            
            # Send through the ROS 2 node attached to the server
            self.server.ros_node.send_mission_request(req)
            
            self._send_success_response("Mission triggered")

        elif self.path == '/api/abort':
            req = MissionCommand.Request()
            req.abort = True
            
            self.server.ros_node.send_mission_request(req)
            self._send_success_response("Abort triggered")
            
        else:
            self.send_response(404)
            self.end_headers()

    def _send_success_response(self, message):
        """Helper to send HTTP 200 OK."""
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps({"status": "success", "message": message}).encode('utf-8'))


class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.port = 8890
        self.target_dir = os.path.expanduser('~/medvtol_ws/template')
        
        # Create ROS 2 Client
        self.mission_client = self.create_client(MissionCommand, 'trigger_mission')
        
        self.get_logger().info(f"Web server started at http://localhost:{self.port}")
        
        # Run HTTP Server in Background Thread
        self.server_thread = threading.Thread(target=self.start_server, daemon=True)
        self.server_thread.start()

    def send_mission_request(self, req):
        """Fires the service call asynchronously to avoid blocking the HTTP thread."""
        if self.mission_client.wait_for_service(timeout_sec=1.0):
            self.mission_client.call_async(req)
            self.get_logger().info(f"Command forwarded to Mission Manager (Abort: {req.abort}).")
        else:
            self.get_logger().error("trigger_mission service is not available!")

    def start_server(self):
        try:
            os.chdir(self.target_dir)
        except FileNotFoundError:
            self.get_logger().error(f"Directory not found: {self.target_dir}")
            return
            
        with socketserver.TCPServer(("0.0.0.0", self.port), CustomWebHandler) as httpd:
            # Attach the ROS node to the HTTP server instance so the handler can use it
            httpd.ros_node = self 
            httpd.serve_forever()


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Web Server Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()