import time

# import math
# from simple_pid import PID
# import qwiic_dual_encoder_reader  # Motor encoder reader
import qwiic_scmd  # Motor control
import qwiic_icm20948  # IMU
import numpy as np
from scipy import linalg

myMotor = qwiic_scmd.QwiicScmd()
IMU = qwiic_icm20948.QwiicIcm20948()

# PID controller
# pid = PID(Kp=10, Ki=0.1, Kd=0.05, setpoint=0)
# Kp = 20
# Ki = 0.5
# Kd = 0.1
# pid = PID(Kp, Ki, Kd, setpoint=0)
# pid.output_limits = (-200, 200)
# angle = 0

R_MTR = 0
L_MTR = 1
FWD = 0
REV = 1


def initialize_system():
    if myMotor.connected is False:
        print("Motor Driver not connected.")
        return

    myMotor.begin()
    print("Motor initialized.")
    time.sleep(0.250)

    # Zero Motor Speeds
    myMotor.set_drive(L_MTR, FWD, 0)
    myMotor.set_drive(R_MTR, FWD, 0)

    myMotor.enable()
    print("Motor enabled")
    time.sleep(0.250)

    IMU.begin()

    if IMU.connected is False:
        print("The Qwiic ICM20948 device isn't connected to the system.")
        return

    print("IMU initialized.")


# def read_IMU(angle):
#     # Read accelerometer data
#     if IMU.dataReady():
#         IMU.getAgmt()  # read all axis and temp from sensor, note this also updates all instance variables

#         # Calculate pitch angle
#         accel_angle = math.atan2(IMU.ayRaw, IMU.azRaw) * 180 / math.pi
#         gyro_angle = IMU.gxRaw / 131.0  # Gyro sensitivity is 131 LSB/degrees/sec

#         # Complementary filter to combine accelerometer and gyroscope data
#         angle = 0.98 * (angle + gyro_angle * 0.02) + 0.02 * accel_angle

#         return angle


def get_imu_data():  # Read accelerometer data
    if IMU.dataReady():
        IMU.getAgmt()  # read all axis and temp from sensor, note this also updates all instance variables
        acc_x = IMU.axRaw / 16384.0
        acc_y = IMU.ayRaw / 16384.0
        acc_z = IMU.azRaw / 16384.0
        gyro_x = IMU.gxRaw / 131.0
        gyro_y = IMU.gyRaw / 131.0
        gyro_z = IMU.gzRaw / 131.0

        return np.array([acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])


def set_motor_speed(left_speed, right_speed):
    if left_speed >= 0:
        myMotor.set_drive(L_MTR, REV, left_speed)
    else:
        myMotor.set_drive(L_MTR, FWD, abs(left_speed))

    if right_speed >= 0:
        myMotor.set_drive(R_MTR, FWD, right_speed)
    else:
        myMotor.set_drive(R_MTR, REV, abs(right_speed))


def lqr(A, B, Q, R):
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = np.dot(np.linalg.inv(R), np.dot(B.T, P))
    return K


# System matrices (example for a simple 2D system)
A = np.array([[0, 1], [0, 0]])
B = np.array([[0], [1]])

# LQR weights
Q = np.eye(2)
R = np.array([[1]])

# Calculate LQR gain
K = lqr(A, B, Q, R)

initialize_system()

# Main control loop
try:
    while True:
        # Get IMU data
        imu_data = get_imu_data()

        # print(f"{imu_data[0]},   {imu_data[1]},   {imu_data[2]}")
        # Extract relevant state information (example: using pitch and pitch rate)
        pitch = np.arctan2(imu_data[1], np.sqrt(imu_data[0] ** 2 + imu_data[2] ** 2))
        pitch_rate = imu_data[3]

        # Current state
        x = np.array([[pitch], [pitch_rate]])

        # Calculate control input
        u = -np.dot(K, x)
        # Apply control input to your system here
        # For example, if controlling a motor:
        # set_motor_speed(u[0][0])
        speed = int(u[0][0])
        set_motor_speed(speed, speed)

        # print(f"Pitch: {pitch:.2f}, Pitch Rate: {pitch_rate:.2f}, Control Input: {u[0][0]:.2f}")

        time.sleep(0.01)  # 100 Hz control loop

except KeyboardInterrupt:
    myMotor.disable()
