import paho.mqtt.client as mqtt
import json
import threading
import time

MQTT_BROKER = "localhost"  # MQTT broker address
MQTT_PORT = 1883  # Default MQTT port
TOPIC_CONTROL = "two_bot/control_request"  # Topic for receiving control commands
TOPIC_STATUS = "two_bot/status_request"  # Topic for receiving status requests
TOPIC_RESPONSE = "two_bot/status_response"  # Topic for sending status responses


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
        self.Vb = 0
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
        # Subscribe to control and status topics on connect/reconnect
        self.client.subscribe([(TOPIC_CONTROL, 0), (TOPIC_STATUS, 0)])
        print(f"Subscribed to topics: {TOPIC_CONTROL}, {TOPIC_STATUS}")

    def on_message(self, client, userdata, msg):
        try:
            print(f"\n--- Received Message: {msg.topic} ---")
            if msg.topic == TOPIC_STATUS:
                # Send current status with 2 significant digits
                response = {
                    "Vb": float(f"{self.Vb:.2f}"),
                    "Rp": float(f"{self.Rp:.2f}"),
                    "Ri": float(f"{self.Ri:.2f}"),
                    "Rd": float(f"{self.Rd:.2f}")
                }
                self.client.publish(TOPIC_RESPONSE, json.dumps(response))
                print(f"Published status response: {response}")
            elif msg.topic == TOPIC_CONTROL:
                # Handle control messages
                data = json.loads(msg.payload.decode())
                for key, value in data.items():
                    print(f"{key}: {value}")
                    # Update corresponding values based on control message
                    if hasattr(self, key):
                        setattr(self, key, value)
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
                # Only publish on status request now
                time.sleep(0.1)  # Sleep to prevent busy loop
            except Exception as e:
                print(f"Error in publish loop: {e}")
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
