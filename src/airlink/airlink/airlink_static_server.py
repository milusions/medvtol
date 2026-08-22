import rclpy
from rclpy.node import Node
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
import os

class StaticFileHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        # Only serve on the root (home) page
        if self.path == '/':
            file_path = "template/index.html" 
            
            # Check if the HTML file exists
            if os.path.exists(file_path):
                with open(file_path, 'rb') as file:
                    content = file.read()
                    
                self.send_response(200)
                self.send_header("Content-type", "text/html")
                self.end_headers()
                self.wfile.write(content)
            else:
                self.send_response(404)
                self.send_header("Content-type", "text/html")
                self.end_headers()
                self.wfile.write(b"<h1>404 Not Found</h1><p>Ensure index.html exists in the run directory.</p>")
        else:
           
            self.send_response(404)
            self.end_headers()


class AirLinkStaticServer(Node):
    def __init__(self):
        super().__init__('airlink_static_server')
        self.host = "0.0.0.0"
        self.port = 8889
        
        # Initialize the HTTP server
        self.http_server = HTTPServer((self.host, self.port), StaticFileHandler)
        
        # Run it in a daemon thread so it doesn't block rclpy.spin()
        self.server_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.server_thread.start()
        
        self.get_logger().info(f"AirLink Static Server at http://{self.host}:{self.port}")

    def destroy_node(self):
        self.get_logger().info("Shutting down HTTP server...")
        self.http_server.shutdown()
        self.http_server.server_close()
        self.server_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AirLinkStaticServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

