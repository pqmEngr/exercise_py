import socket
import json
import time


class TCPClient:
    def __init__(self, host="localhost", port=9005):
        self.host = host
        self.port = port
        self.socket = None

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            print(f"[CLIENT] Đã kết nối tới {self.host}:{self.port}")
        except Exception as e:
            print("[CLIENT] Lỗi khi kết nối:", e)
            self.socket = None

    def send_json_data(self, data: dict):
        if self.socket is None:
            print("[CLIENT] Chưa kết nối.")
            return

        try:
            json_data = json.dumps(data)
            self.socket.sendall(json_data.encode())

            # Nhận phản hồi (nếu có)
            response = self.socket.recv(1024)
            print("Client received:", json.loads(response.decode()))
        except Exception as e:
            print("[CLIENT] Lỗi khi gửi/nhận:", e)

    def close(self):
        if self.socket:
            self.socket.close()
            print("[CLIENT] Đã đóng kết nối")


# Ví dụ sử dụng
if __name__ == "__main__":
    client = TCPClient()
    client.connect()
    try:
        while True:
            data_to_send = {
                "robot_status": 2,
                "card_id": "2301",
                "navigation_req": True,
                "odometry": {
                    "traslation_x": 1.1,
                    "traslation_y": 0.0,
                    "traslation_z": 0.0,
                    "yaw": 0.4,
                },
            }

            client.send_json_data(data_to_send)
            time.sleep(1)  # Gửi mỗi 1 giây

    except KeyboardInterrupt:
        print("\n[CLIENT] Dừng gửi dữ liệu.")
