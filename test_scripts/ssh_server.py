# Raspberry Pi socket server

import socket
import json
import time
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

SERVER_IP = '192.168.50.50'  # Replace with your Raspberry Pi's IP address
PORT = 5000  # The same port as the server

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
# HOST = ''  # Listen on all available interfaces
SERVER_IP = '192.168.50.50'  # I am the host
PORT = 5000  # Port to listen on


def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((SERVER_IP, PORT))
    server_socket.listen(1)

    print(f"Server listening on {SERVER_IP}:{PORT}")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr}")

        try:
            while True:
                # Generate sample data (replace with your actual sensor data)
                data = {
                    'Kp': chan0.voltage,
                    'Ki': chan1.voltage,
                    'Kd': chan2.voltage
                }

                # Send data as JSON
                json_data = json.dumps(data)
                client_socket.send(json_data.encode())

                time.sleep(0.1)  # Send data every second
        except Exception as e:
            print(f"Error: {e}")
        finally:
            client_socket.close()
            print("Close connection")


if __name__ == '__main__':
    start_server()
