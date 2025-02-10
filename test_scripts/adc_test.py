import time
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADC object using the I2C bus
ads = ADS.ADS1015(i2c)

# Create single-ended input on channels 0 - 3
chan0 = AnalogIn(ads, ADS.P0)
chan1 = AnalogIn(ads, ADS.P1)
chan2 = AnalogIn(ads, ADS.P2)
chan3 = AnalogIn(ads, ADS.P3)

# Create differential input between channel 0 and 1
# chan = AnalogIn(ads, ADS.P0, ADS.P1)

# Main loop
while True:
    print(f"{chan0.voltage:>6.3f}\t{chan1.voltage:>6.3f}\t{chan2.voltage:>6.3f}\t{chan3.voltage:>6.3f}\t")
    # print("{:>5}\t{:>6.3f}".format(chan0.value, chan0.voltage))
    time.sleep(0.5)
