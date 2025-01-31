import paho.mqtt.client as mqtt
import json
import threading
import time

MQTT_BROKER = "localhost"  # MQTT broker address
MQTT_PORT = 1883  # Default MQTT port
TOPIC_SEND = "two_bot/data"  # Topic for sending data
TOPIC_RECEIVE = "two_bot/control"  # Topic for receiving control commands


class PiServer:
    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT):
        self.broker = broker
        self.port = port
        self.client = None
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
        
        # MQTT setup
        self.setup_mqtt()

    def setup_mqtt(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        # Subscribe to control topic on connect/reconnect
        self.client.subscribe(TOPIC_RECEIVE)

    def on_message(self, client, userdata, msg):
        try:
            # Handle incoming slider value
            if msg.topic == TOPIC_RECEIVE:
                self.slider_val = int(msg.payload.decode())
                self.slider_update = True
        except Exception as e:
            print(f"Error processing message: {e}")

    def start(self):
        try:
            self.client.connect(self.broker, self.port, 60)
            self.running = True
            
            # Start the MQTT loop in a background thread
            self.client.loop_start()
            
            # Start the publishing loop in a separate thread
            self.publish_thread = threading.Thread(target=self.publish_data)
            self.publish_thread.start()
            
            print(f"MQTT Server started on {self.broker}:{self.port}")
        except Exception as e:
            print(f"Failed to start server: {e}")
            self.stop()

    def stop(self):
        self.running = False
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
        print("Server stopped")

    def publish_data(self):
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
                # Publish data as JSON
                json_data = json.dumps(data)
                self.client.publish(TOPIC_SEND, json_data)
                time.sleep(0.1)  # Publish at 10Hz
            except Exception as e:
                print(f"Error publishing data: {e}")
                if not self.running:
                    break


# Usage example:
if __name__ == "__main__":
    server = PiServer()
    server.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        server.stop()
