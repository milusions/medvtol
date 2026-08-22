import sys
import math
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


def main():
    if len(sys.argv) < 5:
        print("Usage: python3 send_mission.py <gz_x> <gz_y> <gz_z> <cruise_alt>")
        return

    gz_x = float(sys.argv[1])
    gz_y = float(sys.argv[2])
    gz_z = float(sys.argv[3])
    cruise_alt = float(sys.argv[4])

    # Convert Gazebo coords to GPS
    target_lat, target_lon, target_alt = gazebo_to_gps(gz_x, gz_y, gz_z)

    rclpy.init()
    node = Node("mission_client_node")
    client = node.create_client(MissionCommand, "trigger_mission")

    while not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info("Waiting for /trigger_mission service...")

    req = MissionCommand.Request()
    req.target_lat = target_lat
    req.target_lon = target_lon
    req.target_alt = target_alt
    req.cruise_alt = cruise_alt
    req.abort = False

    node.get_logger().info(
        f"Converted Gazebo ({gz_x}, {gz_y}, {gz_z}) -> GPS Lat: {target_lat:.6f}, Lon: {target_lon:.6f}, Alt: {target_alt:.2f}"
    )

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Response: {future.result().message}")
    else:
        node.get_logger().error("Service call failed.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()