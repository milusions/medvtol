import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from falcon_interfaces.msg import Attitude
from actuator_msgs.msg import Actuators
import time
import numpy as np
from falcon_interfaces.msg import PIDSetpoint

class InnerLoopNode(Node):

    def __init__(self,node_name):
        super().__init__(node_name)
        best_effort_qos_profile = QoSProfile(
              history = HistoryPolicy.KEEP_LAST,
              depth=10,
              reliability=ReliabilityPolicy.BEST_EFFORT
          )
        motor_pub_qos_profile = QoSProfile(
              history = HistoryPolicy.KEEP_LAST,
              depth=10,
              reliability=ReliabilityPolicy.RELIABLE
          )
        self.current_attitude = None

        self.pid_setpoints = {"roll":0,"pitch":0,"yaw_rate":0,"throttle":0}

        # tuned gains: roll/pitch are angle-error control loops, yaw is a rate loop.
        # roll/pitch and motor layout are symmetric so they share gains.
        self.pid = {"roll":None,"pitch":None,"yaw":None}
        self.pid["roll"]  = {"Kp":220.0, "Kd":18.0, "Ki":40.0,
                              "previous_error":0.0, "integral":0.0,
                              "previous_control_time": self.get_time_seconds(),
                              "prev_derivative":0.0, "MAX_I":300}
        self.pid["pitch"] = {"Kp":220.0, "Kd":18.0, "Ki":40.0,
                              "previous_error":0.0, "integral":0.0,
                              "previous_control_time": self.get_time_seconds(),
                              "prev_derivative":0.0, "MAX_I":300}
        self.pid["yaw"]   = {"Kp":150.0, "Kd":4.0,  "Ki":15.0,
                              "previous_error":0.0, "integral":0.0,
                              "previous_control_time": self.get_time_seconds(),
                              "prev_derivative":0.0, "MAX_I":200}

        # derivative low-pass filter coefficient (0-1, higher = more smoothing)
        self.d_filter_alpha = 0.2

        # motor command range is 0-1200 rad/s (see morpho_gazebo.xacro maxRotVelocity)
        self.config = {"max_motor_output":1200.0, "min_motor_output":0.0}

        self.attitude_sub = self.create_subscription(Attitude,"/estimation/attitude",self.attitude_topic_callback,best_effort_qos_profile)
        self.motor_commands_pub = self.create_publisher(Actuators,"/model/medvtol/command/motor_speed",motor_pub_qos_profile);
        self.pid_setpoint_sub_ = self.create_subscription(PIDSetpoint,"/pid_setpoint",self.pid_setpoint_update_callback,best_effort_qos_profile);

        self.controlTimer = self.create_timer(1/50,self.control_timer_callback)

        self.get_logger().info("Falcon Inner Loop Controller node has started...")

    def pid_setpoint_update_callback(self,message):
        self.pid_setpoints["roll"] = message.roll
        self.pid_setpoints["pitch"] = message.pitch
        self.pid_setpoints["yaw_rate"] = message.yaw_rate
        self.pid_setpoints["throttle"] = message.throttle

    def attitude_topic_callback(self, message):
        self.current_attitude = message

    def control_timer_callback(self):
        if self.current_attitude is None:
            return

        control_motor_commands = Actuators()

        roll_error = self.pid_setpoints["roll"] - self.current_attitude.roll
        pitch_error = self.pid_setpoints["pitch"] - self.current_attitude.pitch
        yaw_error = self.pid_setpoints["yaw_rate"] - self.current_attitude.angular_velocity.z

        roll_pid_value = self.pid_alg(roll_error, self.pid["roll"])
        pitch_pid_value = self.pid_alg(pitch_error, self.pid["pitch"])
        yaw_pid_value = self.pid_alg(yaw_error, self.pid["yaw"])

        # if throttle is (near) zero, disarm PID output so the drone doesn't
        # twitch/flip on the ground and integrators don't wind up while idle.
        if self.pid_setpoints["throttle"] <= 1e-3:
            self.reset_pid_state()
            motors_mixed = [0.0, 0.0, 0.0, 0.0]
        else:
            motors_mixed = self.motor_mixing(self.pid_setpoints["throttle"],roll_pid_value,pitch_pid_value,yaw_pid_value)

        control_motor_commands.velocity = [float(np.clip(x, self.config["min_motor_output"], self.config["max_motor_output"])) for x in motors_mixed]
        self.get_logger().debug(f"control: {motors_mixed}")
        self.motor_commands_pub.publish(control_motor_commands)

    def reset_pid_state(self):
        for axis in self.pid.values():
            axis["previous_error"] = 0.0
            axis["integral"] = 0.0
            axis["prev_derivative"] = 0.0
            axis["previous_control_time"] = self.get_time_seconds()

    def pid_alg(self, current_error, pid_metadata):
        now = self.get_time_seconds()
        dt = now - pid_metadata["previous_control_time"]

        if dt <= 0:
            return 0.0

        proportional_term = pid_metadata["Kp"]*current_error

        raw_derivative = (current_error - pid_metadata["previous_error"])/dt
        # low-pass filter the derivative term to avoid noise amplification
        filtered_derivative = (self.d_filter_alpha*raw_derivative +
                                (1-self.d_filter_alpha)*pid_metadata["prev_derivative"])
        differential_term = pid_metadata["Kd"]*filtered_derivative

        pid_metadata["integral"] += pid_metadata["Ki"]*0.5*(current_error + pid_metadata["previous_error"])*dt
        pid_metadata["integral"] = float(np.clip(pid_metadata["integral"], -pid_metadata["MAX_I"], pid_metadata["MAX_I"]))
        integral_term = pid_metadata["integral"]

        pid_value = proportional_term + differential_term + integral_term

        pid_metadata["previous_error"] = current_error
        pid_metadata["prev_derivative"] = filtered_derivative
        pid_metadata["previous_control_time"] = now

        return pid_value

    def motor_mixing(self,throttle,roll,pitch,yaw):
        # motor order: 0=FL, 1=FR, 2=BR, 3=BL
        # signs derived from prop positions in morpho_urdf.xacro (x fwd, y left)
        # and turning_direction in morpho_gazebo.xacro (FL/BR=cw, FR/BL=ccw):
        #   +roll  -> more thrust on left  (FL,BL), less on right (FR,BR)
        #   +pitch -> more thrust on back  (BL,BR), less on front (FL,FR)
        #   +yaw   -> more thrust on cw motors (FL,BR), less on ccw (FR,BL)
        # If the drone still rolls/pitches/yaws the wrong way in sim, flip only
        # that one sign below rather than re-deriving everything.
        motors_mixed = [0,0,0,0]

        motor_coeff = 10
        motors_mixed[0] = motor_coeff*(throttle - roll + pitch - yaw)
        motors_mixed[1] = motor_coeff*(throttle + roll + pitch + yaw)
        motors_mixed[2] = motor_coeff*(throttle + roll - pitch - yaw)
        motors_mixed[3] = motor_coeff*(throttle - roll - pitch + yaw)

        min_out = self.config["min_motor_output"]
        max_out = self.config["max_motor_output"]

        supplement = min(motors_mixed) - min_out
        if supplement < 0:
            for i in range(len(motors_mixed)):
                motors_mixed[i] = motors_mixed[i] - supplement

        excess = max(motors_mixed) - max_out
        if excess > 0:
            for i in range(len(motors_mixed)):
                motors_mixed[i] = motors_mixed[i] - excess

        return motors_mixed

    def get_time_seconds(self):
         return (time.time_ns() // 1000000) / 1000.0

def main():
    rclpy.init()
    node = InnerLoopNode("inner_loop_node")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    