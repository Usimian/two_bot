# client.py
import socket
import time

# Define server IP address and port
SERVER_IP = '192.168.50.50'  # Replace with your Raspberry Pi's IP address
PORT = 5000  # The same port as the server

# Create a TCP/IP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
client_socket.connect((SERVER_IP, PORT))

try:
    while True:
        message = 'Hello, Raspberry Pi!'
        client_socket.sendall(message.encode())
        data = client_socket.recv(1024)
        print('Received from server:', data.decode())
        time.sleep(1)
finally:
    client_socket.close()
