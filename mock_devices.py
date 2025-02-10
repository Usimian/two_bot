"""Mock implementations of hardware devices for testing without physical hardware"""
import random
import time
from board import SCL, SDA

class MockI2C:
    """Mock I2C bus for testing"""
    def __init__(self):
        self.scl = SCL
        self.sda = SDA
        print("Mock I2C bus initialized")
        
    def try_lock(self):
        return True
        
    def unlock(self):
        pass

class MockIMU:
    """Mock IMU device for testing"""
    def __init__(self):
        self.connected = True
        self.axRaw = 0
        self.ayRaw = 0
        self.azRaw = 0
        self.gxRaw = 0
        self.gyRaw = 0
        self.gzRaw = 0
        
    def begin(self):
        print("Mock IMU initialized")
        return True
        
    def dataReady(self):
        return True
        
    def getAgmt(self):
        # Simulate small random noise
        self.axRaw = random.uniform(-0.1, 0.1)
        self.ayRaw = random.uniform(-0.1, 0.1)
        self.azRaw = 1.0  # Simulated gravity
        self.gxRaw = random.uniform(-0.1, 0.1)
        self.gyRaw = random.uniform(-0.1, 0.1)
        self.gzRaw = random.uniform(-0.1, 0.1)

class MockADC:
    """Mock ADC for simulating voltage readings"""
    def __init__(self):
        self.base_voltage = 12.0
        self.last_update = time.time()
        # Initialize response values
        self.rp = 0.0
        self.ri = 0.0
        self.rd = 0.0
        self._update_interval = 0.1  # Update values every 100ms
        
    def get_voltage(self, channel):
        """Simulate voltage readings for different channels"""
        now = time.time()
        dt = now - self.last_update
        
        # Update response values periodically
        if dt >= self._update_interval:
            self._simulate_response_values()
            self.last_update = now
        
        # Add some random noise
        noise = random.uniform(-0.1, 0.1)
        
        # Different behavior for different channels
        if channel == 3:  # Battery voltage
            voltage = self.simulate_battery_voltage()
        elif channel == 0:  # Rp channel
            voltage = self.rp
        elif channel == 1:  # Ri channel
            voltage = self.ri
        elif channel == 2:  # Rd channel
            voltage = self.rd
        else:  # Other channels
            voltage = random.uniform(0, 5)
            
        return voltage + noise
        
    def _simulate_response_values(self):
        """Simulate PID response values with realistic behavior"""
        # Proportional response (-5 to 5)
        self.rp = random.uniform(-5, 5)
        
        # Integral response (slowly changing)
        self.ri = max(-10, min(10, self.ri + random.uniform(-0.2, 0.2)))
        
        # Derivative response (quick changes)
        self.rd = random.uniform(-2, 2)
        
    def simulate_battery_voltage(self):
        """Simulate battery voltage with realistic behavior"""
        # Base voltage with noise
        voltage = self.base_voltage + random.uniform(-0.1, 0.1)
        
        # Simulate voltage drop under load
        if hasattr(self, 'is_enabled') and self.is_enabled:
            voltage_drop = random.uniform(0.5, 1.5)
        else:
            voltage_drop = random.uniform(0.1, 0.3)
            
        voltage -= voltage_drop
        
        # Keep voltage in realistic range
        return max(9.0, min(12.0, voltage))

class MockAnalogIn:
    """Mock AnalogIn for ADC readings"""
    def __init__(self, adc, pin):
        self.adc = adc
        self.pin = pin
        self._voltage = 0
        
    @property
    def voltage(self):
        if isinstance(self.adc, MockADC):
            return self.adc.get_voltage(self.pin)
        return random.uniform(0, 5)  # Fallback random value

class MockOLED:
    """Mock OLED display for testing"""
    def __init__(self, width=128, height=32):
        self.width = width
        self.height = height
        self.buffer = []
        
    def clear(self):
        print("Mock OLED: Clear display")
        self.buffer = []
        
    def draw_rectangle(self, x, y, width, height, fill=False):
        print(f"Mock OLED: Draw rectangle at ({x}, {y}) size {width}x{height} {'filled' if fill else 'outline'}")
        
    def display_text(self, text, x=0, y=0):
        print(f"Mock OLED: {text} at ({x}, {y})")  # Print to console instead

class MockGPIO:
    """Mock GPIO for testing without hardware"""
    BCM = "BCM"
    OUT = "OUT"
    
    @staticmethod
    def setmode(mode):
        pass
        
    @staticmethod
    def setwarnings(flag):
        pass
        
    @staticmethod
    def setup(pin, mode):
        pass
        
    @staticmethod
    def output(pin, value):
        pass

class MockDRV8825:
    """Mock DRV8825 stepper driver for testing"""
    def __init__(self, dir_pin, step_pin, enable_pin, mode_pins):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.enable_pin = enable_pin
        self.mode_pins = mode_pins
        self.current_position = 0
        self.is_enabled = False
        self.direction = "forward"
        print(f"Mock stepper initialized (pins: dir={dir_pin}, step={step_pin}, enable={enable_pin})")
    
    def digital_write(self, pin, value):
        if pin == self.enable_pin:
            self.is_enabled = bool(value)
            if value:
                print(f"Mock stepper enabled")
            else:
                print(f"Mock stepper disabled")
        elif pin == self.dir_pin:
            self.direction = "forward" if value == 0 else "backward"
            print(f"Mock stepper direction: {self.direction}")
    
    def Stop(self):
        self.is_enabled = False
        print("Mock stepper stopped")
    
    def SetMicroStep(self, mode, stepformat):
        print(f"Mock stepper microstepping set to: {stepformat}")
    
    def TurnStep(self, Dir, steps, stepdelay=0.005):
        if not self.is_enabled:
            print("Mock stepper is disabled")
            return
            
        if Dir not in ["forward", "backward"]:
            print("Invalid direction")
            return
            
        step_change = steps if Dir == "forward" else -steps
        self.current_position += step_change
        print(f"Mock stepper moved {steps} steps {Dir}. Current position: {self.current_position}")
        # Simulate the time it would take
        time.sleep(steps * stepdelay * 2)
