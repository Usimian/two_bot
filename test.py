# import RPi.GPIO as GPIO
import time
from DRV8825 import DRV8825


try:
    Motor1 = DRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
    Motor2 = DRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))

    Motor1.Stop()
    Motor2.Stop()

    # 1.8 degree: nema23, nema14
    # software Control :
    # 'fullstep': A cycle = 200 steps
    # 'halfstep': A cycle = 200 * 2 steps
    # '1/4step': A cycle = 200 * 4 steps
    # '1/8step': A cycle = 200 * 8 steps
    # '1/16step': A cycle = 200 * 16 steps
    # '1/32step': A cycle = 200 * 32 steps

    Motor2.SetMicroStep("hardware", "fullstep")
    Motor2.TurnStep(Dir="forward", steps=200, stepdelay=0.005)
    time.sleep(0.5)
    Motor2.TurnStep(Dir="backward", steps=200, stepdelay=0.005)
    Motor2.Stop()

    Motor1.SetMicroStep("hardware", "fullstep")
    Motor1.TurnStep(Dir="forward", steps=200, stepdelay=0.005)
    time.sleep(0.5)
    Motor1.TurnStep(Dir="backward", steps=200, stepdelay=0.005)
    Motor1.Stop()

except KeyboardInterrupt:
    # GPIO.cleanup()
    print("\nMotor stop")
    Motor1.Stop()
    Motor2.Stop()
    exit()
