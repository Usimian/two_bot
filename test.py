import paho.mqtt.client as mqtt
import json
import time

# MQTT Configuration
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TOPIC_DATA = "two_bot/control_topic"

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code: {rc}")
    client.subscribe(TOPIC_DATA)
    print(f"Subscribed to topic: {TOPIC_DATA}")

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        print("\n--- Received Control Message ---")
        for key, value in data.items():
            print(f"{key}: {value}")
    except Exception as e:
        print(f"Error processing message: {e}")

def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        print(f"Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nExiting...")
        client.disconnect()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
