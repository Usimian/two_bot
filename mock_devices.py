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
        
    def get_voltage(self, channel):
        """Simulate voltage readings for different channels"""
        now = time.time()
        dt = now - self.last_update
        
        # Add some random noise
        noise = random.uniform(-0.1, 0.1)
        
        # Different behavior for different channels
        if channel == 3:  # Battery voltage
            voltage = self.simulate_battery_voltage()
        else:  # Potentiometer channels
            voltage = random.uniform(0, 5)
            
        self.last_update = now
        return voltage
        
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
