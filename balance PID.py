import time
import math
from simple_pid import PID
import numpy as np
from PioLED import PioLED
import busio
from board import SCL, SDA

# import qwiic_dual_encoder_reader  # Motor encoder reader
import qwiic_scmd  # Motor control
import qwiic_icm20948  # IMU
import qwiic_dual_encoder_reader  # Motor encoder reader
import adafruit_ads1x15.ads1015 as ADS  # 4 chan ADC
from adafruit_ads1x15.analog_in import AnalogIn

myMotor = qwiic_scmd.QwiicScmd()
myEncoders = qwiic_dual_encoder_reader.QwiicDualEncoderReader()
myIMU = qwiic_icm20948.QwiicIcm20948(address=0x68)

i2c = busio.I2C(SCL, SDA)
myADC = ADS.ADS1015(i2c)

# Create single-ended input on channels 0 - 3
chan0 = AnalogIn(myADC, ADS.P0)
chan1 = AnalogIn(myADC, ADS.P1)
chan2 = AnalogIn(myADC, ADS.P2)
chan3 = AnalogIn(myADC, ADS.P3)

# PID balance controller
# kP = 4
# kI = 0
# kD = 0
# pid = PID(kP, kI, kD, setpoint=0)
pid = PID(4.0, 0, 0, setpoint=0)
pid.output_limits = (-200, 200)
pid.sample_time = 0.01

# PID position controller
pid_pos = PID(0, 0, 0, setpoint=0)
pid_pos.output_limits = (-200, 200)
pid_pos.sample_time = 0.01

old_pos = 0
x_vel = 0
ticksPerMm = 937.0/300.0  # ticks per millimeter
oldTickTime = 0  # uSec

angle_corr = 0.0

R_MTR = 0
L_MTR = 1
FWD = 0
REV = 1

oled = PioLED()


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

    oled.clear()
    oled.display_text(f"kP = {pid.Kp:.1f} kP2 = {pid_pos.Kp:.1f}", 0, 0)
    oled.display_text(f"kI = {pid.Ki:.1f} kI2 = {pid_pos.Ki:.1f}", 0, 8)
    oled.display_text(f"kD = {pid.Kd:.1f} kD2 = {pid_pos.Kd:.1f}", 0, 16)
    # oled.draw_rectangle(10, 10, 30, 20, fill=True)
    # oled.draw_line(0, 0, 127, 31)


def read_IMU(angle):
    # Read accelerometer data
    if myIMU.dataReady():
        myIMU.getAgmt()  # read all axis and temp from sensor, note this also updates all instance variables

        # Calculate pitch angle
        accel_angle = math.atan2(myIMU.ayRaw, myIMU.azRaw) * 180 / math.pi - angle_corr  # Correct for IMU angle when balanced
        gyro_angle = myIMU.gxRaw / 131.0  # Gyro sensitivity is 131 LSB/degrees/sec

        # Complementary filter to combine accelerometer and gyroscope data
        angle = 0.99 * (angle + gyro_angle * 0.02) + 0.01 * accel_angle
        # print(f"accel_angle: {accel_angle:>.1f}\tgyro_angle: {gyro_angle:>.1f}\tangle: {angle:>.1f}\tcorr: {angle_corr:>.1f}")
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

    current_position = ((myEncoders.count1 - myEncoders.count2) / 2) / ticksPerMm  # position in mm
    position_error = target_position - current_position

    tickTime = time.time()  # sec

    dTickTime = tickTime - oldTickTime  # sec
    oldTickTime = tickTime  # sec

    x_vel = (old_pos - current_position) / dTickTime  # mm per sec
    old_pos = current_position  # mm

    return pid_pos(position_error)
    # return pid_pos(x_vel)


try:
    initialize_system()
    pid.proportional_on_measurement = True
    pid_pos.proportional_on_measurement = True
    prevAngle = 0
    speed = 0
    oldTickTime = 0
    # calibrate_gyro()
    old_loop_time = time.time()  # sec
    while True:
        pos_adj = move_to_position(0)
        prevAngle = read_IMU(prevAngle)
        control = pid(prevAngle)
        # speed = control - pos_adj
        speed = control

        # if speed > 0:
        #     speed += 10
        # elif speed < 0:
        #     speed -= 10

        # print(f"x_vel: {x_vel:>.2f}\tangle: {prevAngle:>.0f}\tpos_adj: {pos_adj:>.0f}\tcontrol: {control:>.0f}\tspeed: {speed:>.0f}")
        left_speed = speed
        right_speed = speed

        set_motor_speed(left_speed, right_speed)
        new_time = time.time()
        if new_time > old_loop_time + 1.0:  # Display update loop
            # pid.tunings = (chan0.voltage * 10, chan1.voltage * 10, chan2.voltage * 10)
            pid_pos.tunings = (chan0.voltage * 10, chan1.voltage * 10, chan2.voltage * 10)
            angle_corr = chan3.voltage * 6.0 - 10.0
            print(f"angle: {prevAngle:>.1f}\tangle_corr: {angle_corr:>.1f}\told_pos: {old_pos:>.0f}\tcontrol: {control:>.1f}\tpos_adj: {pos_adj:>.1f}")
            # print(f"{pid.Kp:>6.3f}\t{pid.Ki:>6.3f}\t{pid.Kd:>6.3f}")
            print(f"{pid_pos.Kp:>6.3f}\t{pid_pos.Ki:>6.3f}\t{pid_pos.Kd:>6.3f}")
            # oled.clear()
            # oled.display_text(f"kP = {pid.Kp}", 0, 0)
            # oled.display_text(f"kI = {pid.Ki}", 0, 10)
            # oled.display_text(f"kD = {pid.Kd}", 0, 20)
            old_loop_time = new_time

except KeyboardInterrupt:
    myMotor.disable()
