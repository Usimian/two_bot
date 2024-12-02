import time
from simple_pid import PID
import qwiic_icm20948  # IMU
from DRV8825 import DRV8825
import busio
from board import SCL, SDA

# import threading
import math
import numpy as np
import adafruit_ads1x15.ads1015 as ADS  # 4 chan ADC
from adafruit_ads1x15.analog_in import AnalogIn

Motor1 = DRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
Motor2 = DRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))

Motor1.Stop()
Motor2.Stop()

i2c = busio.I2C(SCL, SDA)
myIMU = qwiic_icm20948.QwiicIcm20948(address=0x68)
myADC = ADS.ADS1015(i2c)

# Create single-ended input on channels 0 - 3
chan0 = AnalogIn(myADC, ADS.P0)
chan1 = AnalogIn(myADC, ADS.P1)
chan2 = AnalogIn(myADC, ADS.P2)
chan3 = AnalogIn(myADC, ADS.P3)

Rp = chan0.voltage * 5
Ri = chan1.voltage * 5
Rd = chan2.voltage

Kp = Rp
Ki = Ri
Kd = Rd

angle_corr = 0.0


def init_mpu():
    myIMU.begin()  # Initialize IMU

    if myIMU.connected is False:
        print("ICM20948 IMU not connected.")
        return

    print("IMU initialized.")


class KalmanFilter:
    def __init__(self, dt, process_variance, measurement_variance):
        self.dt = dt
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.angle = 0
        self.bias = 0
        self.P = np.eye(2)

    def predict(self, rate):
        # Predict the state
        self.angle += self.dt * (rate - self.bias)

        # Update the error covariance matrix
        self.P[0][0] += self.dt * (self.dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.process_variance)
        self.P[0][1] -= self.dt * self.P[1][1]
        self.P[1][0] -= self.dt * self.P[1][1]
        self.P[1][1] += self.process_variance * self.dt

    def update(self, measurement):
        # Calculate the Kalman gain
        S = self.P[0][0] + self.measurement_variance
        K = [self.P[0][0] / S, self.P[1][0] / S]

        # Update the estimate
        y = measurement - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y

        # Update the error covariance matrix
        self.P[0][0] -= K[0] * self.P[0][0]
        self.P[0][1] -= K[0] * self.P[0][1]
        self.P[1][0] -= K[1] * self.P[0][0]
        self.P[1][1] -= K[1] * self.P[0][1]

    def get_angle(self):
        return self.angle


def read_IMU(angle):
    # Read accelerometer data
    if myIMU.dataReady():
        myIMU.getAgmt()  # read all axis and temp from sensor, note this also updates all instance variables

        gyro_rate = myIMU.gxRaw / 131.0  # Gyro sensitivity is 131 LSB/degrees/sec
        accel_angle = math.atan2(myIMU.ayRaw, myIMU.azRaw) * 180 / math.pi - angle_corr  # Degrees, Correct for IMU angle when balanced

        kf.predict(gyro_rate)
        kf.update(accel_angle)

        angle = kf.get_angle()

        # Calculate pitch angle
        # accel_angle = math.atan2(myIMU.ayRaw, myIMU.azRaw) * 180 / math.pi - angle_corr  # Degrees, Correct for IMU angle when balanced
        # gyro_angle = myIMU.gxRaw / 131.0  # Gyro sensitivity is 131 LSB/degrees/sec

        # Complementary filter to combine accelerometer and gyroscope data
        # angle = 0.99 * (angle + gyro_angle * 0.02) + 0.01 * accel_angle  # Degrees
        # print(f"accel_angle: {accel_angle:>.1f}\tgyro_angle: {gyro_angle:>.1f}\tangle: {angle:>.1f}\tcorr: {angle_corr:>.1f}")
        return angle


# # Usage example
dt = 0.01  # Time step
process_variance = 0.001  # Adjust based on your system
measurement_variance = 0.1  # Adjust based on your sensors

kf = KalmanFilter(dt, process_variance, measurement_variance)

# Initialize PID controller
pid = PID(Kp, Ki, Kd, setpoint=0)
pid.output_limits = (-20, 20)
pid.sample_time = 0.002


def control_loop():
    prev_angle = 0
    step_delay = 0.002
    old_loop_time = time.time()  # sec

    while True:
        prev_angle = read_IMU(prev_angle)
        control = pid(prev_angle)
        print(prev_angle, control)
        if control > 0:
            Motor1.TurnStep(Dir="backward", steps=abs(int(control)), stepdelay=step_delay)
            Motor2.TurnStep(Dir="forward", steps=abs(int(control)), stepdelay=step_delay)
        else:
            Motor1.TurnStep(Dir="forward", steps=abs(int(control)), stepdelay=step_delay)
            Motor2.TurnStep(Dir="backward", steps=abs(int(control)), stepdelay=step_delay)

        new_time = time.time()
        if new_time > old_loop_time + 1.0:  # Display update loop
            Rp = chan0.voltage  # 0 - 16.5
            Ri = chan1.voltage  # 0 - 16.5
            Rd = chan2.voltage  # 0 - 3.3
            # v_batt = chan3.voltage * 152.6 / 13.9
            pid.tunings = (Rp, Ri, Rd)  # Balance PID tuning
            old_loop_time = new_time


try:
    # client_thread = threading.Thread(target=self.handle_client, args=(conn, addr))
    # client_thread.start()
    # print("handle_client thread created.")
    init_mpu()
    control_loop()

except KeyboardInterrupt:
    Motor1.Stop()
    Motor2.Stop()
    print("Motors OFF")
