import socket
# import json
import threading
# import time


class PiServer:
    def __init__(self, host="192.168.50.50", port=5000):
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False

        # Client variables for transfer
        self.slider_val = 0
        self.slider_update = False
        # self.Kp = 0
        # self.Ki = 0
        # self.Kd = 0
        # self.Kp2 = 0
        # self.Ki2 = 0
        # self.Kd2 = 0

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        self.running = True
        print(f"Server started on {self.host}:{self.port}")
        while self.running:
            try:
                conn, addr = self.server_socket.accept()
                client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                client_thread.start()
                print("handle_client thread created.")
            except socket.timeout:
                continue

    def stop(self):
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        print("Server stopped")

    def handle_client(self, conn, addr):
        print(f"Connected by {addr}")
        while self.running:
            try:
                # data = {"Kp": self.Kp, "Ki": self.Ki, "Kd": self.Kd, "Kp2": self.Kp2, "Ki2": self.Ki2, "Kd2": self.Kd2}
                # # Send data as JSON
                # json_data = json.dumps(data)
                # conn.send(json_data.encode())  # Send K numbers to client
                # # print(f"handle_client:{json_data}")
                # time.sleep(1)

                data = conn.recv(1024)  # Get silder position value
                # print(data)
                if not data:
                    break
                s = data.decode()
                self.slider_val = int(s)
                self.slider_update = True
                # print(s)

            except KeyboardInterrupt:
                break
        conn.close()


# Usage
# if __name__ == "__main__":
#     server = PiServer()
#     server_thread = threading.Thread(target=server.start)
#     server_thread.start()
#     time.sleep(1)

#     # To stop the server
#     server.stop()
#     server_thread.join()
