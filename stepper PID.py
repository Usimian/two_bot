import math
from simple_pid import PID
import numpy as np
from PioLED import PioLED
import busio
from board import SCL, SDA
import random
import time
import json
import threading
import qwiic_icm20948  # IMU
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from server import PiServer
from mock_devices import MockIMU, MockADC, MockAnalogIn, MockOLED, MockI2C, MockGPIO, MockDRV8825

def initialize_motors():
    global Motor1, Motor2, GPIO
    try:
        # Try to import real GPIO
        import RPi.GPIO as GPIO
        # Test GPIO access
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        print("GPIO hardware detected, using real stepper motors")
        from DRV8825 import DRV8825
        Motor1 = DRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
        Motor2 = DRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))
    except Exception as e:
        print(f"GPIO initialization failed: {e}")
        print("Using mock stepper motors")
        GPIO = MockGPIO
        Motor1 = MockDRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
        Motor2 = MockDRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))

# Initialize motors
initialize_motors()

# I2C error handling wrapper class
class I2CDevice:
    def __init__(self, device, name):
        self.device = device
        self.name = name
        self.error_count = 0
        self.max_retries = 3
        self.retry_delay = 0.1  # seconds

    def execute(self, operation, *args, **kwargs):
        for attempt in range(self.max_retries):
            try:
                return operation(*args, **kwargs)
            except Exception as e:
                self.error_count += 1
                print(f"I2C error on {self.name}: {str(e)}")
                if attempt < self.max_retries - 1:
                    print(f"Retrying {self.name} operation in {self.retry_delay}s (attempt {attempt + 2}/{self.max_retries})")
                    time.sleep(self.retry_delay)
                else:
                    print(f"Failed to execute {self.name} operation after {self.max_retries} attempts")
                    raise

# Initialize devices with hardware detection
def initialize_i2c_devices():
    global i2c, myADC, myIMU, oled, AnalogIn
    try:
        # Try to initialize real I2C bus
        i2c = busio.I2C(SCL, SDA)
        if not i2c.try_lock():
            raise RuntimeError("Unable to lock I2C bus")
        i2c.unlock()
        
        print("I2C hardware detected, using real devices")
        try:
            myADC = I2CDevice(ADS.ADS1015(i2c), "ADC")
        except Exception as e:
            print(f"ADC initialization failed: {e}, using mock ADC")
            myADC = I2CDevice(MockADC(), "MockADC")
            AnalogIn = MockAnalogIn
            
        try:
            myIMU = I2CDevice(qwiic_icm20948.QwiicIcm20948(address=0x68), "IMU")
        except Exception as e:
            print(f"IMU initialization failed: {e}, using mock IMU")
            myIMU = I2CDevice(MockIMU(), "MockIMU")
            
        try:
            oled = I2CDevice(PioLED(), "OLED")
        except Exception as e:
            print(f"OLED initialization failed: {e}, using mock OLED")
            oled = I2CDevice(MockOLED(), "MockOLED")
            
    except Exception as e:
        print(f"No I2C hardware detected: {e}")
        print("Initializing all mock devices")
        i2c = MockI2C()
        myADC = I2CDevice(MockADC(), "MockADC")
        myIMU = I2CDevice(MockIMU(), "MockIMU")
        oled = I2CDevice(MockOLED(), "MockOLED")
        AnalogIn = MockAnalogIn

# Initialize devices
initialize_i2c_devices()

def is_mock_device(device):
    """Check if a device is a mock implementation"""
    return device.__class__.__name__.startswith('Mock')

# PID balance controller
Kp = 6.0
Ki = 33.0
Kd = 0.1
pid = PID(Kp, Ki, Kd, setpoint=0)
pid.output_limits = (-200, 200)
pid.sample_time = 0.002

# PID position controller
Kp2 = 0.44
Ki2 = 0
Kd2 = 0.3
pid_pos = PID(Kp2, Ki2, Kd2, setpoint=0)
pid_pos.output_limits = (-7, 7)
pid_pos.sample_time = 0.02

battery_cal_factor = 152.6 / 13.9

def get_simulated_voltage():
    """Simulate battery voltage between 9V and 12V"""
    # Start with base voltage of 12V
    base_voltage = 12.0
    
    # Add some random noise (-0.1V to +0.1V)
    noise = random.uniform(-0.1, 0.1)
    
    # If motors are enabled, simulate voltage drop
    if hasattr(Motor1, 'is_enabled') and Motor1.is_enabled:
        # Simulate larger voltage drop when motors are active
        voltage_drop = random.uniform(0.5, 1.5)
    else:
        # Small voltage drop when idle
        voltage_drop = random.uniform(0.1, 0.3)
        
    # Calculate final voltage
    voltage = base_voltage + noise - voltage_drop
    
    # Ensure voltage stays within 9V to 12V range
    return max(9.0, min(12.0, voltage))


# Initialize ADC channels with error handling
try:
    # Force simulation if using mock ADC
    if is_mock_device(myADC.device):
        raise Exception("Using mock ADC")
        
    # Create single-ended input on channels 0 - 3
    chan0 = AnalogIn(myADC.device, ADS.P0)
    chan1 = AnalogIn(myADC.device, ADS.P1)
    chan2 = AnalogIn(myADC.device, ADS.P2)
    chan3 = AnalogIn(myADC.device, ADS.P3)
    Vb = chan3.voltage * battery_cal_factor
except Exception as e:
    print(f"Failed to initialize ADC channels: {e}")
    chan0 = chan1 = chan2 = chan3 = None
    Vb = get_simulated_voltage()

def read_adc_values():
    """Read ADC values with error handling"""
    try:
        # Check if myADC and its device attribute are properly initialized
        if myADC is None:
            raise Exception("ADC not initialized")

        # Check if myADC.device is not None
        if not hasattr(myADC, 'device') or myADC.device is None:
            raise Exception("ADC not initialized")

        # Use mock ADC values if using mock device
        if is_mock_device(myADC.device):
            # Get simulated values from MockADC
            chan0 = AnalogIn(myADC.device, 0)  # Rp channel
            chan1 = AnalogIn(myADC.device, 1)  # Ri channel
            chan2 = AnalogIn(myADC.device, 2)  # Rd channel
            chan3 = AnalogIn(myADC.device, 3)  # Battery voltage channel
        else:
            # Read real ADC values
            chan0 = AnalogIn(myADC.device, ADS.P0)
            chan1 = AnalogIn(myADC.device, ADS.P1)
            chan2 = AnalogIn(myADC.device, ADS.P2)
            chan3 = AnalogIn(myADC.device, ADS.P3) * battery_cal_factor


        Rp = chan0.voltage * 5
        Ri = chan1.voltage * 5
        Rd = chan2.voltage
        Vb = chan3.voltage
        return Rp, Ri, Rd, Vb

    except Exception as e:
        print(f"Failed to read ADC values: {e}")
        # Return simulated values as fallback
        Vb = get_simulated_voltage()
        Rp = random.uniform(-5, 5)
        Ri = random.uniform(-10, 10)
        Rd = random.uniform(-2, 2)
        return Rp, Ri, Rd, Vb

read_adc_values()

# oled display
oled.device.clear()

oled.device.draw_rectangle(0, 25, Vb * 90 / 17.2, 6, fill=True)
oled.device.display_text(f"{Vb:.2f} ", 92, 23)

Rp = 0
Ri = 0
Rd = 0

oled.device.display_text(f"{Rp:.2f} {Ri:.2f} {Rd:.2f} {Vb:.2f}", 0, -1)
oled.device.display_text(f"{pid.Kp:>5.1f}{pid.Ki:>5.1f}{pid.Kd:>5.1f}", 0, 7)
oled.device.display_text(f"{pid_pos.Kp:>5.1f}{pid_pos.Ki:>5.1f}{pid_pos.Kd:>5.1f}", 0, 15)

old_pos = 0
x_vel = 0
ticksPerMm = 937.0 / 300.0  # ticks per millimeter
oldTickTime = 0.0  # Sec
dTickTime = 0.0  # Sec

angle_corr = 2.0

R_MTR = 0
L_MTR = 1
FWD = 0
REV = 1

Vb = 0

SERVER_IP = "192.168.1.167"  # I am the host
PORT = 1883  # Port to listen on

def initialize_system():
    try:
        if not myIMU.execute(lambda: myIMU.device.begin()):
            print("ICM20948 IMU not connected.")
            return False
        print("IMU initialized.")
    except Exception as e:
        print(f"Failed to initialize IMU: {str(e)}")
        return False

    try:
        Motor1.Stop()
        Motor2.Stop()

        # Read ADC values with simulation fallback
        Rp, Ri, Rd, Vb = read_adc_values()

        # Update OLED display with error handling
        oled.execute(lambda: oled.device.clear())
        oled.execute(lambda: oled.device.draw_rectangle(0, 25, Vb * 90 / 17.2, 6, fill=True))
        oled.execute(lambda: oled.device.display_text(f"{Vb:.2f} ", 92, 23))
        oled.execute(lambda: oled.device.display_text(f"{Rp:.2f} {Ri:.2f} {Rd:.2f} {Vb:.2f}", 0, -1))
        oled.execute(lambda: oled.device.display_text(f"{pid.Kp:>5.1f}{pid.Ki:>5.1f}{pid.Kd:>5.1f}", 0, 7))
        oled.execute(lambda: oled.device.display_text(f"{pid_pos.Kp:>5.1f}{pid_pos.Ki:>5.1f}{pid_pos.Kd:>5.1f}", 0, 15))

        print(f"Rp:  {Rp:>5.2f}\tRi:  {Ri:>5.2f}\tRd:  {Rd:>5.2f}\tBattery: {Vb:>5.2f}")
        print(f"Kp:  {pid.Kp:>5.2f}\tKi:  {pid.Ki:>5.2f}\tKd:  {pid.Kd:>5.2f}")
        print(f"Kp2: {pid_pos.Kp:>5.2f}\tKi2: {pid_pos.Ki:>5.2f}\tKd2: {pid_pos.Kd:>5.2f}")
        return True
    except Exception as e:
        print(f"Error in system initialization: {str(e)}")
        return False


def read_IMU(angle):
    # Read accelerometer data
    try:
        if myIMU.execute(lambda: myIMU.device.dataReady()):
            myIMU.execute(lambda: myIMU.device.getAgmt())
            ax = myIMU.device.axRaw
            ay = myIMU.device.ayRaw
            az = myIMU.device.azRaw
            gx = myIMU.device.gxRaw
            gy = myIMU.device.gyRaw
            gz = myIMU.device.gzRaw

            # Calculate pitch angle from accelerometer
            pitch = math.atan2(ay, math.sqrt(ax * ax + az * az)) * 180.0 / math.pi
            return pitch + angle_corr
    except Exception as e:
        print(f"Error reading IMU data: {str(e)}")
        return angle  # Return previous angle on error
    return angle


def calibrate_gyro(samples=1000):
    print("Calibrating gyroscope. Keep the sensor still.")
    gyro_data = []
    for _ in range(samples):
        if myIMU.execute(lambda: myIMU.device.dataReady()):
            myIMU.execute(lambda: myIMU.device.getAgmt())  # read all axis and temp from sensor
            ax = myIMU.device.axRaw
            ay = myIMU.device.ayRaw
            az = myIMU.device.azRaw
            # gx = myIMU.device.gxRaw
            # gy = myIMU.device.gyRaw
            # gz = myIMU.device.gzRaw
            print(f"{ax}, {ay}, {az}")
            # print(f"{gx}, {gy}, {gz}")
            # gyro_data.append([gx, gy, gz])
            time.sleep(0.1)

    gyro_offset = np.mean(gyro_data, axis=0)
    print(f"Gyro offsets: {gyro_offset}")
    return gyro_offset


def set_motor_speed(left_speed, right_speed):
    # if left_speed >= 0:
    #     myMotor.set_drive(L_MTR, REV, left_speed)
    # else:
    #     myMotor.set_drive(L_MTR, FWD, abs(left_speed))

    # if right_speed >= 0:
    #     myMotor.set_drive(R_MTR, FWD, right_speed)
    # else:
    #     myMotor.set_drive(R_MTR, REV, abs(right_speed))
    return


def move_to_position(target_position):
    global oldTickTime, dTickTime, x_vel, old_pos

    # current_position = ((myEncoders.count1 - myEncoders.count2) / 2.0) / ticksPerMm  # position in mm
    current_position = 0.0
    position_error = (target_position - current_position) / 10.0

    tickTime = time.time()  # Current time in sec
    dTickTime = tickTime - oldTickTime  # dt sec

    if dTickTime > 0.1:
        x_vel = (old_pos - current_position) / dTickTime  # mm per sec
        oldTickTime = tickTime  # sec
        old_pos = current_position  # mm

    return pid_pos(position_error)


try:
    initialize_system()
    pos_setpoint = 0
    server = PiServer(broker=SERVER_IP, port=PORT)  # Create server class
    server_thread = threading.Thread(target=server.start)
    server_thread.start()
    server.Rp = Rp
    server.Ri = Ri
    server.Rd = Rd
    server.Vb = Vb
    server.Kp = pid.Kp
    server.Ki = pid.Ki
    server.Kd = pid.Kd
    server.Kp2 = pid_pos.Kp
    server.Ki2 = pid_pos.Ki
    server.Kd2 = pid_pos.Kd
    server.Pos = old_pos

    pid.proportional_on_measurement = True
    pid_pos.proportional_on_measurement = True
    prevAngle = 0
    speed = 0
    oldTickTime = 0
    # calibrate_gyro()
    old_loop_time = time.time()  # sec
    while True:
        pos_err = move_to_position(pos_setpoint)  # Use pos_setpoint instead of server.slider_val
        prevAngle = read_IMU(prevAngle)
        control = pid(prevAngle + pos_err)
        speed = control

        left_speed = speed
        right_speed = speed

        set_motor_speed(left_speed, right_speed)
        new_time = time.time()
        if new_time > old_loop_time + 1.0:  # Display update loop
            # Read ADC values with simulation fallback
            Rp, Ri, Rd, Vb = read_adc_values()

            server.Rp = Rp
            server.Ri = Ri
            server.Rd = Rd
            server.Vb = Vb  # Updated from v_batt to Vb
            # pid.tunings = (Rp, Ri, Rd)  # Balance PID tuning
            # pid_pos.tunings = (Rp, Ri, Rd)  # Position PID tuning
            # print(f"{pid.Kp:>5.1f}{pid.Ki:>5.1f}{pid.Kd:>5.2f}\told_pos: {old_pos:>5.2f}\t{pos_err:5.2f}")
            # print(f"{pid_pos.Kp:>5.1f}{pid_pos.Ki:>5.1f}{pid_pos.Kd:>5.2f}")
            # Update K values for server
            server.Kp = pid.Kp
            server.Ki = pid.Ki
            server.Kd = pid.Kd
            server.Kp2 = pid_pos.Kp
            server.Ki2 = pid_pos.Ki
            server.Kd2 = pid_pos.Kd
            server.Pos = old_pos

            # print(f"{old_pos:>5.2f} mm\tprevAngle: {prevAngle:>5.2f} deg\t{pos_err:5.2f} mm")
            # print(f"L: {myEncoders.count1:>5.2f}\tR: {myEncoders.count2:>5.2f} deg\t{pos_err:5.2f} mm")
            # print(f"{server.Kp:>6.3f}\t{server.Ki:>6.3f}\t{server.Kd:>6.3f}")
            # print(f"{server.Kp2:>6.3f}\t{server.Ki2:>6.3f}\t{server.Kd2:>6.3f}")
            old_loop_time = new_time

except KeyboardInterrupt:
    Motor1.Stop()
    Motor2.Stop()
    print("Motors OFF")
    print(f"Kp:  {pid.Kp:>5.2f}\tKi:  {pid.Ki:>5.2f}\tKd:  {pid.Kd:>5.2f}")
    print(f"Kp2: {pid_pos.Kp:>5.2f}\tKi2: {pid_pos.Ki:>5.2f}\tKd2: {pid_pos.Kd:>5.2f}")

server.stop()
server_thread.join()
