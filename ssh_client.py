from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.clock import Clock
from kivy.uix.slider import Slider
import socket
import json
import threading
from kivy.core.window import Window

Window.size = (720, 480)

SERVER_IP = "192.168.50.50"  # Host
PORT = 5000  # The same port as the server


class DataDisplay(BoxLayout):
    def __init__(self, **kwargs):
        super(DataDisplay, self).__init__(**kwargs)
        self.orientation = "vertical"

        self.pos_slider = Slider(min=-100, max=100, value=0)
        self.add_widget(Label(text="POSITION"))
        self.add_widget(self.pos_slider)
        self.pos_value = Label(text="0")
        self.add_widget(self.pos_value)
        self.pos_slider.bind(value=self.on_slider)

        self.Kp = Label(text="Kp")
        self.Ki = Label(text="Ki")
        self.Kd = Label(text="Kd")

        self.Kp2 = Label(text="Kp2")
        self.Ki2 = Label(text="Ki2")
        self.Kd2 = Label(text="Kd2")

        self.add_widget(self.Kp)
        self.add_widget(self.Ki)
        self.add_widget(self.Kd)

        self.add_widget(self.Kp2)
        self.add_widget(self.Ki2)
        self.add_widget(self.Kd2)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_to_server()

    def connect_to_server(self):
        try:
            self.socket.connect((SERVER_IP, PORT))
            print(f"Connected to server {SERVER_IP}:{PORT}")
            threading.Thread(target=self.receive_data, daemon=True).start()
        except Exception as e:
            print(f"Connection failed: {e}")

    def receive_data(self):     # Receive data from RP
        while True:
            try:
                data = self.socket.recv(1024).decode()
                if not data:
                    break
                json_data = json.loads(data)
                print(json_data)
                Clock.schedule_once(lambda dt: self.update_labels(json_data))
            except Exception as e:
                print(f"Error receiving data: {e}")
                break
        self.socket.close()

    def update_labels(self, data):
        self.Kp.text = f"Proportional: {data['Kp']:.2f}"
        self.Ki.text = f"Integral: {data['Ki']:.2f}"
        self.Kd.text = f"Derivative: {data['Kd']:.2f}"
        self.Kp2.text = f"Proportional 2: {data['Kp2']:.2f}"
        self.Ki2.text = f"Integral 2: {data['Ki2']:.2f}"
        self.Kd2.text = f"Derivative 2: {data['Kd2']:.2f}"

    def on_slider(self, instance, val):
        self.pos_value.val = int(val)
        self.pos_value.text = f"{self.pos_value.val}"
        response = self.pos_value.text
        self.socket.sendall(response.encode())      # Send position value to RP


class TwoBotApp(App):
    def build(self):
        return DataDisplay()


if __name__ == "__main__":
    TwoBotApp().run()
