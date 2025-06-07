import rclpy
from rclpy.node import Node
import socket
import json


class TCPJsonClientNode(Node):
    def __init__(self):
        super().__init__("tcp_json_client_node")

        self.host = "127.0.0.1"
        self.port = 9005

        self.timer = self.create_timer(2.0, self.send_json_data)

        # data = {
        #     "odometry": {
        #         "traslation_x": 1.1,
        #         "traslation_y": 0.0,
        #         "traslation_z": 0.0,
        #         "yaw": 0.4,
        #     },
        # }

        test = {
            "robot_status": 2,
            "card_id": "location_1c12",
            "navigation_req": True,
            "odometry": {
                "traslation_x": 1.1,
                "traslation_y": 0.0,
                "traslation_z": 0.0,
                "yaw": 0.4,
            },
        }
        self.json_data = json.dumps(test)

    def send_json_data(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.host, self.port))
                s.sendall(self.json_data.encode())

                # receive response
                response = s.recv(1024)
                print("Client received:", json.loads(response.decode()))
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TCPJsonClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
