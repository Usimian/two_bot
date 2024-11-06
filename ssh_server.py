# Raspberry Pi socket server

import socket
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
ads = ADS.ADS1015(i2c)

# Create single-ended input on channels 0 - 3
chan0 = AnalogIn(ads, ADS.P0)
chan1 = AnalogIn(ads, ADS.P1)
chan2 = AnalogIn(ads, ADS.P2)
chan3 = AnalogIn(ads, ADS.P3)

# Define host and port
HOST = ''  # Listen on all available interfaces
PORT = 5000  # Port to listen on

# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_socket.bind((HOST, PORT))

# Enable the server to accept connections
server_socket.listen(1)
print('Server listening on port', PORT)

# Wait for a connection
conn, addr = server_socket.accept()
print('Connected by', addr)

while True:
    data = conn.recv(1024)
    if not data:
        break
    print('Received:', data.decode())
    msg = f"Channel 2 = {chan2.voltage}V"
    conn.sendall(msg.encode('utf-8'))  # Echo back the received data

# Clean up the connection
conn.close()
