import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64MultiArray
from px4_msgs.msg import VehicleLocalPosition, VehicleGlobalPosition
from medvtol_companion.static_values import FlightMode, FlightStage

# Custom interfaces
from medvtol_interfaces.srv import MissionCommand 
from medvtol_interfaces.msg import Telemetry, DroneWaypoint


class MissionManagerNode(Node):

    def __init__(self):
        super().__init__('mission_manager_node')

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscriptions
        self.vehicle_location_sub = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.vehicle_location_callback, qos_profile
        )
        self.vehicle_global_sub = self.create_subscription(
            VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_callback, qos_profile
        )

        # Publishers & Services
        self.trajectory_pub = self.create_publisher(DroneWaypoint, '/companion_controller/set_trajectory', 10)
        self.mode_pub = self.create_publisher(Float64MultiArray, '/companion_controller/set_mode', 10)
        self.telemetry_pub = self.create_publisher(Telemetry, '/telemetry', 10)
        self.mission_service = self.create_service(MissionCommand, 'trigger_mission', self.mission_service_callback)

        # State Variables
        self.current_stage = FlightStage.IDLE
        self.current_flight_mode = FlightMode.NONE
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0        
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0       
        self.current_yaw = 0.0     

        # Live Drone GPS
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        
        # Drone Arm / Local Origin GPS (where x=0, y=0, z=0 is set)
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0  
        self.cruise_altitude = 10.0
        self.takeoff_altitude = 2.5

        self.xy_tolerance = 0.5     
        self.z_tolerance = 0.3
        self.yaw_tolerance = 0.1        
        
        self.land_stable_count = 0
        self.delay_ticks = 0       

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Mission Manager Node Initialized. Waiting for initial drone GPS lock...")

    def vehicle_global_callback(self, msg: VehicleGlobalPosition):
        self.current_lat = msg.lat
        self.current_lon = msg.lon
        self.current_alt = msg.alt

        # Save initial GPS position during arming/startup as home/local origin reference
        if self.home_lat is None and msg.lat != 0.0:
            self.home_lat = msg.lat
            self.home_lon = msg.lon
            self.home_alt = msg.alt
            self.get_logger().info(
                f"Drone Arm GPS Origin Locked -> Lat: {self.home_lat:.6f}, Lon: {self.home_lon:.6f}, Alt: {self.home_alt:.2f}m"
            )

    def vehicle_location_callback(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_vx = msg.vx
        self.current_vy = msg.vy
        self.current_vz = msg.vz
        self.current_yaw = msg.heading  

    def global_to_local_ned(self, target_lat: float, target_lon: float, target_alt: float):
        """Converts target GPS coordinates into PX4 local NED meters relative to the drone's arm position."""
        EARTH_RADIUS = 6378137.0
        d_lat = math.radians(target_lat - self.home_lat)
        d_lon = math.radians(target_lon - self.home_lon)

        x_ned = d_lat * EARTH_RADIUS
        y_ned = d_lon * EARTH_RADIUS * math.cos(math.radians(self.home_lat))
        z_ned = -(target_alt - self.home_alt)
        return x_ned, y_ned, z_ned

    def mission_service_callback(self, request, response):
        if request.abort:
            self.abort_mission()
            response.success = True
            response.message = "Abort initiated: Engaging HOLD, then RTL."
            return response

        if self.current_stage != FlightStage.IDLE:
            self.get_logger().warn("Service rejected: System is locked.")
            response.success = False
            response.message = "System is locked. Flight mode is not IDLE."
            return response

        if self.home_lat is None:
            self.get_logger().error("Service rejected: Drone arm GPS position not fixed yet.")
            response.success = False
            response.message = "Drone arm GPS lock missing."
            return response

        self.start_mission(request.target_lat, request.target_lon, request.target_alt, request.cruise_alt)
        response.success = True
        response.message = "Mission successfully triggered."
        return response

    def start_mission(self, target_lat: float, target_lon: float, target_alt: float, cruise_alt: float = 10.0):
        self.get_logger().info(f"Target GPS Received -> Lat: {target_lat}, Lon: {target_lon}, Alt: {target_alt}")
        
        # Calculate local setpoints relative to the drone's arm position
        self.target_x, self.target_y, self.target_z = self.global_to_local_ned(target_lat, target_lon, target_alt)
        self.get_logger().info(f"Calculated PX4 Local Setpoint -> X: {self.target_x:.2f}m, Y: {self.target_y:.2f}m, Z: {self.target_z:.2f}m")

        self.cruise_altitude = cruise_alt
        self.target_yaw = self.current_yaw 
        
        self.land_stable_count = 0
        self.delay_ticks = 0

        self.current_stage = FlightStage.TAKEOFF
        self.request_flight_mode(FlightMode.TAKEOFF)

    def abort_mission(self):
        self.get_logger().warn("ABORTING: Setting mode to HOLD...")
        self.request_flight_mode(FlightMode.HOLD)
        self.current_stage = FlightStage.ABORT
        self.land_stable_count = 0
        self.delay_ticks = 0
        self.rtl_timer = self.create_timer(2.0, self.trigger_rtl)

    def trigger_rtl(self):
        self.get_logger().warn("ABORTING: Setting mode to RTL...")
        self.request_flight_mode(FlightMode.RTL)
        self.rtl_timer.cancel()

    def request_flight_mode(self, mode: FlightMode):
        self.current_flight_mode = mode
        flight_mode_data = Float64MultiArray()
        flight_mode_data.data = [float(mode.value)]
        self.mode_pub.publish(flight_mode_data)

    def update_target_yaw(self):
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        if math.hypot(dx, dy) > 0.5:
            self.target_yaw = math.atan2(dy, dx)

    def publish_trajectory(self, x: float, y: float, z_ned: float, yaw: float):
        msg = DroneWaypoint()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z_ned)
        msg.yaw = float(yaw)
        self.trajectory_pub.publish(msg)

    def publish_telemetry(self):
        tel_msg = Telemetry()
        tel_msg.x = self.current_x
        tel_msg.y = self.current_y
        tel_msg.z = self.current_z
        tel_msg.lat = self.current_lat
        tel_msg.lon = self.current_lon
        tel_msg.altitude = self.current_alt
        tel_msg.vx = self.current_vx
        tel_msg.vy = self.current_vy
        tel_msg.vz = self.current_vz
        tel_msg.flight_stage = self.current_stage.name
        tel_msg.flight_mode = self.current_flight_mode.name
        self.telemetry_pub.publish(tel_msg)

    def has_reached_position(self, target_x: float, target_y: float, target_z_ned: float, check_xy: bool = True, check_z: bool = True) -> bool:
        xy_dist = math.sqrt((self.current_x - target_x) ** 2 + (self.current_y - target_y) ** 2)
        z_dist = abs(self.current_z - target_z_ned)
        xy_ok = (xy_dist < self.xy_tolerance) if check_xy else True
        z_ok = (z_dist < self.z_tolerance) if check_z else True
        return xy_ok and z_ok

    def has_reached_yaw(self, target_yaw: float) -> bool:
        diff = abs(math.atan2(math.sin(self.current_yaw - target_yaw), math.cos(self.current_yaw - target_yaw)))
        return diff < self.yaw_tolerance

    def perform_alignment(self) -> bool:
        return True 

    def control_loop(self):
        self.publish_telemetry()

        if self.current_stage == FlightStage.IDLE:
            return

        elif self.current_stage == FlightStage.TAKEOFF:
            takeoff_z_ned = -self.takeoff_altitude
            self.publish_trajectory(0.0, 0.0, takeoff_z_ned, self.target_yaw)
            
            if self.has_reached_position(0.0, 0.0, takeoff_z_ned, check_xy=False, check_z=True):
                self.delay_ticks += 1
                if self.delay_ticks >= 20:
                    self.get_logger().info("Takeoff complete. 2-second hold achieved. Transitioning to CLIMB.")
                    self.request_flight_mode(FlightMode.OFFBOARD)
                    self.current_stage = FlightStage.CLIMB
                    self.delay_ticks = 0
            else:
                self.delay_ticks = 0

        elif self.current_stage == FlightStage.CLIMB:
            cruise_z_ned = -self.cruise_altitude
            self.publish_trajectory(0.0, 0.0, cruise_z_ned, self.target_yaw)
            
            if self.has_reached_position(0.0, 0.0, cruise_z_ned, check_xy=False, check_z=True):
                self.delay_ticks += 1
                if self.delay_ticks >= 20: 
                    self.get_logger().info("Climb complete. 2-second hold achieved. Calculating yaw.")
                    self.update_target_yaw()  
                    self.current_stage = FlightStage.YAW_ALIGN
                    self.delay_ticks = 0
            else:
                self.delay_ticks = 0

        elif self.current_stage == FlightStage.YAW_ALIGN:
            cruise_z_ned = -self.cruise_altitude
            self.publish_trajectory(0.0, 0.0, cruise_z_ned, self.target_yaw)
            
            if self.has_reached_yaw(self.target_yaw):
                self.delay_ticks += 1
                if self.delay_ticks >= 20:
                    self.get_logger().info("Yaw alignment complete. 2-second hold achieved. Transitioning to CRUISE.")
                    self.current_stage = FlightStage.CRUISE
                    self.delay_ticks = 0
            else:
                self.delay_ticks = 0

        elif self.current_stage == FlightStage.CRUISE:
            cruise_z_ned = -self.cruise_altitude
            self.update_target_yaw() 
            self.publish_trajectory(self.target_x, self.target_y, cruise_z_ned, self.target_yaw)
            
            if self.has_reached_position(self.target_x, self.target_y, cruise_z_ned):
                self.delay_ticks += 1
                if self.delay_ticks >= 20:
                    self.get_logger().info("Cruise coordinates reached. 2-second hold achieved. Transitioning to DESCEND.")
                    self.current_stage = FlightStage.DESCEND
                    self.delay_ticks = 0
            else:
                self.delay_ticks = 0

        elif self.current_stage == FlightStage.DESCEND:
            descend_altitude = 2.5 + abs(self.target_z) 
            descend_z_ned = -descend_altitude
            self.publish_trajectory(self.target_x, self.target_y, descend_z_ned, self.target_yaw)
            
            if self.has_reached_position(self.target_x, self.target_y, descend_z_ned):
                self.delay_ticks += 1
                if self.delay_ticks >= 20:
                    self.get_logger().info("Descend complete. 2-second hold achieved. Transitioning to ALIGN.")
                    self.current_stage = FlightStage.ALIGN
                    self.delay_ticks = 0
            else:
                self.delay_ticks = 0

        elif self.current_stage == FlightStage.ALIGN:
            if self.perform_alignment():
                self.delay_ticks += 1
                if self.delay_ticks >= 20:
                    self.get_logger().info("Alignment complete. 2-second hold achieved. Transitioning to LAND.")
                    self.request_flight_mode(FlightMode.LAND)
                    self.current_stage = FlightStage.LAND
                    self.delay_ticks = 0
            else:
                self.delay_ticks = 0

        elif self.current_stage == FlightStage.LAND:
            is_near_ground = abs(self.current_z - self.target_z) < 0.4
            is_stationary_v = abs(self.current_vz) < 0.1

            if is_near_ground and is_stationary_v:
                self.land_stable_count += 1
            else:
                self.land_stable_count = 0

            if self.land_stable_count >= 20:
                self.get_logger().info("Landing confirmed. Resetting to IDLE state.")
                self.request_flight_mode(FlightMode.NONE)
                self.current_stage = FlightStage.IDLE

        elif self.current_stage == FlightStage.ABORT:
            is_near_ground = abs(self.current_z) < 0.5
            is_stationary = (abs(self.current_vz) < 0.1 and abs(self.current_vx) < 0.1 and abs(self.current_vy) < 0.1)

            if is_near_ground and is_stationary:
                self.land_stable_count += 1
            else:
                self.land_stable_count = 0

            if self.land_stable_count >= 20: 
                self.get_logger().info("RTL Landing confirmed. Resetting to IDLE state.")
                self.request_flight_mode(FlightMode.NONE)
                self.current_stage = FlightStage.IDLE


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()