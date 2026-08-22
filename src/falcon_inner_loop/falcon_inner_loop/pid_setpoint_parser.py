import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from airlink_interfaces.msg import RecieverPacket
from falcon_interfaces.msg import PIDSetpoint
import math

class PIDSetpointParser(Node):
    
    def __init__(self,node_name):
        super().__init__(node_name)
        best_effort_qos_profile = QoSProfile(
              history = HistoryPolicy.KEEP_LAST,
              depth=10,
              reliability=ReliabilityPolicy.BEST_EFFORT
          )
    
        self.pid_setpoints = {"roll":0,"pitch":0,"yaw_rate":0,"throttle":0}
        self.max_yaw_rate = self.deg_to_rad(180)
        self.channel_val_max = 2000
        self.channel_val_min = 1000
        
        self.reciever_sub_ = self.create_subscription(RecieverPacket,"/airlink/reciever",self.airlink_reciever_callback,best_effort_qos_profile)
        self.pid_setpoint_pub_ = self.create_publisher(PIDSetpoint,"/pid_setpoint",best_effort_qos_profile);
      
        
        self.get_logger().info("Falcon Inner Loop PID Setpoint Parser node has started...")
        
    def airlink_reciever_callback(self, message):
        if not ((message.channel_5 == 2000) and (message.channel_6==1000)):
            pid_setpoint_message = PIDSetpoint()
            self.pid_setpoint_pub_.publish(pid_setpoint_message)
            return
        
        self.pid_setpoints["roll"] = int(self.map_value(message.channel_4,self.channel_val_min,self.channel_val_max,-90,90))
        self.pid_setpoints["pitch"] = -1*int(self.map_value(message.channel_3,self.channel_val_min,self.channel_val_max,-90,90))
        self.pid_setpoints["yaw_rate"] = int(self.map_value(message.channel_2,self.channel_val_min,self.channel_val_max,-self.max_yaw_rate,self.max_yaw_rate))
        self.pid_setpoints["throttle"] = int(self.map_value(message.channel_1,self.channel_val_min,self.channel_val_max,0,100))
        
        pid_setpoint_message = PIDSetpoint()
        
        pid_setpoint_message.roll = self.pid_setpoints["roll"] 
        pid_setpoint_message.pitch = self.pid_setpoints["pitch"] 
        pid_setpoint_message.yaw_rate = self.pid_setpoints["yaw_rate"] 
        pid_setpoint_message.throttle = self.pid_setpoints["throttle"] 
        
        self.pid_setpoint_pub_.publish(pid_setpoint_message)
    
    def deg_to_rad(self,deg_per_sec):
     return deg_per_sec * (math.pi / 180)
        
        
    
    def map_value(self,val, min1, max1, min2, max2):
     return min2 + (val - min1) * (max2 - min2) / (max1 - min1)
 
def main():
    rclpy.init()
    node = PIDSetpointParser("pid_serpoint_parser")
    rclpy.spin(node)
    rclpy.shutdown()
    