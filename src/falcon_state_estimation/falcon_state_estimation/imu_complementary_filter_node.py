from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from falcon_interfaces.msg import Attitude
import math
import time

class IMUComplementaryFilterNode(Node):
     
      def __init__(self,node_name):
          super().__init__(node_name)
          best_effort_qos_profile = QoSProfile(
              history = HistoryPolicy.KEEP_LAST,
              depth=10,
              reliability=ReliabilityPolicy.BEST_EFFORT
          )
          self.complementary_filter_alpha = 0.98
          self.prev_filtered_time = self.get_time_seconds()
          self.estimation_filter = "COMPLEMENTARY_FILTER"
          
          self.imu_sub = self.create_subscription(Imu,"/imu",self.raw_imu_topic_callback,best_effort_qos_profile)
          self.estimated_attitude_pub = self.create_publisher(Attitude,"/estimation/attitude",best_effort_qos_profile);
          
          self.get_logger().info("Falcon State Estimation Node started...")
      
      def raw_imu_topic_callback(self,raw_message):
        attitude_msg = Attitude()
        
        attitude_msg.angular_velocity.x = raw_message.angular_velocity.x
        attitude_msg.angular_velocity.y = raw_message.angular_velocity.x
        attitude_msg.angular_velocity.z = raw_message.angular_velocity.z
        
        attitude_msg.linear_acceleration.x = raw_message.linear_acceleration.x
        attitude_msg.linear_acceleration.y = raw_message.linear_acceleration.x
        attitude_msg.linear_acceleration.z = raw_message.linear_acceleration.z
        
        if self.estimation_filter == "COMPLEMENTARY_FILTER":
         self.estimation_complementary_filter(attitude_msg)
        
        self.estimated_attitude_pub.publish(attitude_msg)
        
      def complementary_filter(self, old_value, alpha, angular_velocity, roll_acc, dt):
        return alpha*(old_value +  angular_velocity*dt) + (1-alpha)*roll_acc
      
      def get_accel_attitude(self, linear_acceleration):
        
        roll = math.atan2(-linear_acceleration.y, linear_acceleration.z)
        pitch = math.atan2(linear_acceleration.x, math.sqrt(math.pow(linear_acceleration.y,2)+math.pow(linear_acceleration.z,2)))
        
        return (roll,pitch)
      
      def estimation_complementary_filter(self,attitude_msg):
        self.prev_roll = 0
        self.prev_pitch = 0
        
        
        roll_acc,pitch_acc = self.get_accel_attitude(attitude_msg.linear_acceleration)
        
        roll = self.complementary_filter(self.prev_roll,self.complementary_filter_alpha,
                                         attitude_msg.angular_velocity.x,
                                         roll_acc,(self.get_time_seconds() - self.prev_filtered_time))
        
        pitch = self.complementary_filter(self.prev_pitch,self.complementary_filter_alpha,
                                         attitude_msg.angular_velocity.y,
                                         pitch_acc,(self.get_time_seconds() - self.prev_filtered_time))
        
        yaw = 0.0
        
        self.prev_roll = roll
        self.prev_pitch = pitch
        self.prev_filtered_time = self.get_time_seconds()
        
        quaternion_orientation = self.euler_to_quaternion(roll,pitch,yaw)
        
        attitude_msg.roll = roll
        attitude_msg.pitch = pitch
        attitude_msg.yaw = yaw
        attitude_msg.quaternion.x = quaternion_orientation["x"]
        attitude_msg.quaternion.y = quaternion_orientation["y"]
        attitude_msg.quaternion.z = quaternion_orientation["z"]
        attitude_msg.quaternion.w = quaternion_orientation["w"]
        
        
        
      def euler_to_quaternion(self,roll: float, pitch: float, yaw: float) -> dict:
        """
        Converts Euler angles (Roll, Pitch, Yaw) in radians to a Quaternion.
        
        Assumes a standard Aerospace sequence (Yaw -> Pitch -> Roll).
        
        :param roll: Rotation around X axis (radians)
        :param pitch: Rotation around Y axis (radians)
        :param yaw: Rotation around Z axis (radians)
        :return: Dictionary containing 'w', 'x', 'y', 'z' components
        """
        # Calculate half angles
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        # Compute quaternion components
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return {"w": qw, "x": qx, "y": qy, "z": qz}
        
      def get_time_seconds(self):
         return (time.time_ns() // 1000000) / 1000.0
        
        
        
        

def main():
    rclpy.init()
    node = IMUComplementaryFilterNode("imu_complementary_filter_node")
    rclpy.spin(node)
    rclpy.shutdown()
    
     