import time
import math
from simple_pid import PID
import numpy as np

# import qwiic_dual_encoder_reader  # Motor encoder reader
import qwiic_scmd  # Motor control
import qwiic_icm20948  # IMU
import qwiic_dual_encoder_reader  # Motor encoder reader

myMotor = qwiic_scmd.QwiicScmd()
myEncoders = qwiic_dual_encoder_reader.QwiicDualEncoderReader()
myIMU = qwiic_icm20948.QwiicIcm20948(address=0x68)

# PID balance controller
Kp = 10
Ki = 0.1
Kd = 0
pid = PID(Kp, Ki, Kd, setpoint=0)
pid.output_limits = (-200, 200)

# PID position controller
Kp2 = 0.1
Ki2 = 0
Kd2 = 0
pid_pos = PID(Kp2, Ki2, Kd2, setpoint=0)
pid_pos.output_limits = (-200, 200)
old_pos = 0
x_vel = 0
ticksPerMm = 0.833
oldTickTime = 0  # uSec

R_MTR = 0
L_MTR = 1
FWD = 0
REV = 1


def initialize_system():
    # Initialize motor drive
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

    # Initialize encoders
    if myEncoders.connected is False:
        print("Dual Encoder Reader not connected.")
        return

    myEncoders.begin()
    myEncoders.count1 = 0
    myEncoders.count2 = 0
    print("Encoders enabled")

    # Initialize IMU
    myIMU.begin()

    if myIMU.connected is False:
        print("ICM20948 IMU not connected.")
        return

    print("IMU initialized.")


def read_IMU(angle):
    # Read accelerometer data
    if myIMU.dataReady():
        myIMU.getAgmt()  # read all axis and temp from sensor, note this also updates all instance variables

        # Calculate pitch angle
        accel_angle = math.atan2(myIMU.ayRaw, myIMU.azRaw) * 180 / math.pi
        gyro_angle = myIMU.gxRaw / 131.0  # Gyro sensitivity is 131 LSB/degrees/sec

        # Complementary filter to combine accelerometer and gyroscope data
        angle = 0.99 * (angle + gyro_angle * 0.02) + 0.01 * accel_angle
        # print(f"accel_angle: {accel_angle:.0f}  gyro_angle: {gyro_angle:.0f}")
        return angle


def calibrate_gyro(samples=1000):
    print("Calibrating gyroscope. Keep the sensor still.")
    gyro_data = []
    for _ in range(samples):
        if myIMU.dataReady():
            myIMU.getAgmt()  # read all axis and temp from sensor
            ax = myIMU.axRaw
            ay = myIMU.ayRaw
            az = myIMU.azRaw
            # gx = myIMU.gxRaw
            # gy = myIMU.gyRaw
            # gz = myIMU.gzRaw
            print(f"{ax}, {ay}, {az}")
            # print(f"{gx}, {gy}, {gz}")
            # gyro_data.append([gx, gy, gz])
            time.sleep(0.1)

    gyro_offset = np.mean(gyro_data, axis=0)
    print(f"Gyro offsets: {gyro_offset}")
    return gyro_offset


def set_motor_speed(left_speed, right_speed):
    if left_speed >= 0:
        myMotor.set_drive(L_MTR, REV, left_speed)
    else:
        myMotor.set_drive(L_MTR, FWD, abs(left_speed))

    if right_speed >= 0:
        myMotor.set_drive(R_MTR, FWD, right_speed)
    else:
        myMotor.set_drive(R_MTR, REV, abs(right_speed))


def move_to_position(target_position):
    global oldTickTime, x_vel, old_pos

    current_position = (myEncoders.count1 - myEncoders.count2) / 2
    # position_error = target_position - current_position

    tickTime = time.time()  # sec

    dTickTime = tickTime - oldTickTime  # usec
    x_vel = (old_pos - current_position) / dTickTime  # ticks per uSec
    x_vel = x_vel / ticksPerMm  # mm/sec

    oldTickTime = tickTime
    old_pos = current_position

    # return pid_pos(position_error)
    return pid_pos(x_vel)


try:
    initialize_system()
    pid.proportional_on_measurement = True
    idx = 0
    prevAngle = 0
    speed = 0
    oldTickTime = 0
    # calibrate_gyro()
    while True:
        pos_adj = move_to_position(0)
        prevAngle = read_IMU(prevAngle)
        control = pid(prevAngle)
        speed = control - pos_adj
        # speed = control

        if speed > 0:
            speed += 20
        elif speed < 0:
            speed -= 20

        print(f"x_vel: {x_vel:.2f}  angle: {prevAngle:.0f}  pos_adj: {pos_adj:.0f}  control: {control:.0f}  speed: {speed:.0f}")

        left_speed = speed
        right_speed = speed

        set_motor_speed(left_speed, right_speed)

        time.sleep(0.02)  # 10ms loop time

except KeyboardInterrupt:
    myMotor.disable()
