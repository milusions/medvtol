#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from airlink_interfaces.msg import RecieverPacket

import asyncio
import threading
import websockets
import json


class AirLinkWebSocketServer(Node):

    def __init__(self):
        super().__init__('airlink_websocket_server')
        best_effort_qos_profile = QoSProfile(
              history = HistoryPolicy.KEEP_LAST,
              depth=10,
              reliability=ReliabilityPolicy.BEST_EFFORT
          )
        
        self.airlink_reciever_pub_ = self.create_publisher(RecieverPacket, '/airlink/reciever', best_effort_qos_profile)
 

        self.host = "0.0.0.0"
        self.port = 8888
        
        self.loop = asyncio.new_event_loop()
        self.ws_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.ws_thread.start()

        asyncio.run_coroutine_threadsafe(self.start_ws_server(), self.loop)
        
        self.get_logger().info("AirLink Server Node has started...")
       

    def _run_async_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def start_ws_server(self):
        self.get_logger().info(f"Starting WebSocket server on ws://{self.host}:{self.port}")
        
        try:
            
            async with websockets.serve(self.handler, self.host, self.port):
                await asyncio.Future()  
                
        except Exception as e:
            self.get_logger().error(f"WebSocket server error: {e}")

    async def handler(self, websocket):
 
        client_address = websocket.remote_address
        
        self.get_logger().info(f"Client connected from: {client_address}")
        
        try:
            async for message in websocket:
                self.publish_transmitted_packet(message)
                
        except websockets.ConnectionClosedOK:
            self.get_logger().info(f"Client disconnected gracefully: {client_address}")
        except websockets.ConnectionClosedError as e:
            self.get_logger().warn(f"Client disconnected with error: {client_address} -> {e}")

    def publish_transmitted_packet(self, raw_message):
        msg = RecieverPacket()
        parsed_message = json.loads(raw_message)
        if parsed_message["type"]=="rc_command":
            
            msg.channel_1 = parsed_message["channels"]["channel_1"]
            msg.channel_2 = parsed_message["channels"]["channel_2"]
            msg.channel_3 = parsed_message["channels"]["channel_3"]
            msg.channel_4 = parsed_message["channels"]["channel_4"]
            msg.channel_5 = parsed_message["channels"]["channel_5"]
            msg.channel_6 = parsed_message["channels"]["channel_6"]
            msg.channel_7 = parsed_message["channels"]["channel_7"]
            msg.channel_8 = parsed_message["channels"]["channel_8"]

        
        self.airlink_reciever_pub_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = AirLinkWebSocketServer()
    rclpy.spin(node)
    rclpy.try_shutdown()

