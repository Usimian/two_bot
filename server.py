import socket
import json
import threading

SERVER_IP = "192.168.50.50"  # Host
PORT = 5000  # The same port as the server


class PiServer:
    def __init__(self, host=SERVER_IP, port=PORT):
        self.host = host
        self.port = port
        self.server_socket = None
        self.running = False

        # Client variables for transfer
        self.slider_val = 0
        self.Rp = 0
        self.Ri = 0
        self.Rd = 0
        self.v_batt = 0
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.Kp2 = 0
        self.Ki2 = 0
        self.Kd2 = 0
        self.Pos = 0

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
                data = {
                    "Rp": self.Rp,
                    "Ri": self.Ri,
                    "Rd": self.Rd,
                    "Vb": self.v_batt,
                    "Kp": self.Kp,
                    "Ki": self.Ki,
                    "Kd": self.Kd,
                    "Kp2": self.Kp2,
                    "Ki2": self.Ki2,
                    "Kd2": self.Kd2,
                    "Pos": self.Pos,
                }
                # Send data as JSON
                json_data = json.dumps(data)
                conn.send(json_data.encode())  # Send K numbers to client

                data = conn.recv(1024)  # Get slider position value
                if not data:
                    break
                s = data.decode()
                self.slider_val = int(s)
                self.slider_update = True

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
