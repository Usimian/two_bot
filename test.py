import paho.mqtt.client as mqtt
import json
import time
import random

# MQTT Configuration
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TOPIC_CONTROL = "two_bot/control_request"
TOPIC_STATUS = "two_bot/status_request"
TOPIC_RESPONSE = "two_bot/status_response"

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code: {rc}")
    client.subscribe([(TOPIC_CONTROL, 0), (TOPIC_STATUS, 0)])
    print(f"Subscribed to topics: {TOPIC_CONTROL}, {TOPIC_STATUS}")

def on_message(client, userdata, msg):
    try:
        print(f"\n--- Received Message: {msg.topic} ---")
        if msg.topic == TOPIC_STATUS:
            voltage = round(random.uniform(9, 13), 2)
            response = {
                "Vb": voltage,
                "Rp": round(random.uniform(0, 10), 2),
                "Ri": round(random.uniform(0, 10), 2),
                "Rd": round(random.uniform(0, 10), 2)
            }
            client.publish(TOPIC_RESPONSE, json.dumps(response))
            print(f"Published status response: {response}")
        else:
            data = json.loads(msg.payload.decode())
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
