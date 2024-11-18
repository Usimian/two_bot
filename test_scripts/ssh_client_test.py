from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.clock import Clock
import socket
import json
import threading

SERVER_IP = '192.168.50.50'  # Host
PORT = 5000  # The same port as the server


class DataDisplay(BoxLayout):
    def __init__(self, **kwargs):
        super(DataDisplay, self).__init__(**kwargs)
        self.orientation = 'horizontal'

        self.temp_label = Label(text='Temperature: N/A')
        self.humid_label = Label(text='Humidity: N/A')
        self.pressure_label = Label(text='Pressure: N/A')

        self.add_widget(self.temp_label)
        self.add_widget(self.humid_label)
        self.add_widget(self.pressure_label)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_to_server()

    def connect_to_server(self):
        try:
            self.socket.connect((SERVER_IP, PORT))
            print("Connected to server")
            threading.Thread(target=self.receive_data, daemon=True).start()
        except Exception as e:
            print(f"Connection failed: {e}")

    def receive_data(self):
        while True:
            try:
                data = self.socket.recv(1024).decode()
                if not data:
                    break
                json_data = json.loads(data)
                Clock.schedule_once(lambda dt: self.update_labels(json_data))
            except Exception as e:
                print(f"Error receiving data: {e}")
                break
        self.socket.close()

    def update_labels(self, data):
        self.temp_label.text = f"Proportional: {data['Kp']:.2f}"
        self.humid_label.text = f"Integral: {data['Ki']:.2f}"
        self.pressure_label.text = f"Derivative: {data['Kd']:.2f}"


class DataDisplayApp(App):
    def build(self):
        return DataDisplay()


if __name__ == '__main__':
    DataDisplayApp().run()
