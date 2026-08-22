import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64MultiArray
from px4_msgs.msg import (
    VehicleStatus, VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition
)

import medvtol_companion.gazebo_drone_coordinates as gz_px4_coord
from medvtol_companion.static_values import FlightMode, FlightStage

# Custom interface replacing Point
from medvtol_interfaces.msg import DroneWaypoint


class CompanionController(Node):

    def __init__(self, node_name: str, spawn_x: float = 0.0, spawn_y: float = 0.0, spawn_z: float = 0.0):
        super().__init__(node_name)

        self.update_cycle_frequency = 20.0
        self.spawn_offset_x = spawn_x
        self.spawn_offset_y = spawn_y
        self.spawn_offset_z = spawn_z

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.vehicle_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.target_setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)

        self.status_sub = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", self.vehicle_status_callback, qos_profile)
        self.vehicle_location_sub = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.vehicle_location_callback, qos_profile)

        self.trajectory_sub = self.create_subscription(DroneWaypoint, '/companion_controller/set_trajectory', self.trajectory_callback, 10)
        self.mode_sub = self.create_subscription(Float64MultiArray, '/companion_controller/set_mode', self.mode_callback, 10)

        self.vehicle_status = VehicleStatus()
        self.ned_location = np.array([0.0, 0.0, 0.0])

        self.currentFlightMode = FlightMode.NONE
        self.changeFlightMode = False

        self.offboard_update_timer = self.create_timer(1.0 / self.update_cycle_frequency, self.offboard_update_timer_callback)

    def offboard_update_timer_callback(self):
        if self.changeFlightMode:
            if self.currentFlightMode == FlightMode.TAKEOFF:
                self.set_takeoff_mode()
            elif self.currentFlightMode == FlightMode.LAND:
                self.set_land_mode()
            elif self.currentFlightMode == FlightMode.OFFBOARD:
                self.set_offboard_mode()
            elif self.currentFlightMode == FlightMode.HOLD:
                self.set_hold_mode()
            elif self.currentFlightMode == FlightMode.RTL:
                self.set_rtl_mode()
                
            self.changeFlightMode = False

        self.send_offboard_mode_heartbeat()

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def vehicle_location_callback(self, msg: VehicleLocalPosition):
        self.ned_location = np.array([msg.x, msg.y, msg.z])
        x, y, z = gz_px4_coord.px4_ned_to_gazebo(msg.x, msg.y, msg.z, spawn_x=self.spawn_offset_x, spawn_y=self.spawn_offset_y, spawn_z=self.spawn_offset_z)

    def trajectory_callback(self, msg: DroneWaypoint):
        self.set_target_position(msg.x, msg.y, msg.z, msg.yaw)

    def mode_callback(self, msg: Float64MultiArray):
        if not msg.data:
            return

        raw_mode_val = int(msg.data[0])
        try:
            mode_enum = FlightMode(raw_mode_val)
            if mode_enum in [FlightMode.TAKEOFF, FlightMode.OFFBOARD, FlightMode.LAND, FlightMode.HOLD, FlightMode.RTL]:
                if mode_enum == FlightMode.TAKEOFF:
                    self.send_arm_command()
                self.currentFlightMode = mode_enum
                self.changeFlightMode = True

        except ValueError:
            self.get_logger().warn(f"Received unrecognized mode enum value: {raw_mode_val}")

    def send_arm_command(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def set_offboard_mode(self):
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def set_takeoff_mode(self):
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 2.0)

    def set_land_mode(self):
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 6.0)

    def set_hold_mode(self):
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)

    def set_rtl_mode(self):
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 5.0)

    def set_target_position(self, x: float, y: float, z: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = self.get_timestamp()
        self.target_setpoint_pub.publish(msg)

    def send_offboard_mode_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = self.get_timestamp()
        self.offboard_control_mode_pub.publish(msg)

    def send_command(self, command: int, param1: float = 0.0, param2: float = 0.0, param3: float = 0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.timestamp = self.get_timestamp()
        self.vehicle_command_pub.publish(msg)

    def get_timestamp(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    companion_controller = CompanionController("companion_controller", spawn_y=5.0)
    try:
        rclpy.spin(companion_controller)
    except KeyboardInterrupt:
        pass
    finally:
        companion_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()