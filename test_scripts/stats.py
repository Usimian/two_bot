# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
# SPDX-FileCopyrightText: 2017 James DeVito for Adafruit Industries
# SPDX-License-Identifier: MIT

# This example is for use on (Linux) computers that are using CPython with
# Adafruit Blinka to support CircuitPython libraries. CircuitPython does
# not support PIL/pillow (python imaging library)!

import time
import subprocess

from board import SCL, SDA
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import gpiozero as gz
# from Rosmaster_Lib import Rosmaster

# bot = Rosmaster()
# bot.create_receive_threading()

# Create the I2C interface.
i2c = busio.I2C(SCL, SDA)

# Create the SSD1306 OLED class.
# The first two parameters are the pixel width and pixel height.  Change these
# to the right size for your display!
disp = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)

# Clear display.
disp.fill(0)
disp.show()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width  # (128)
height = disp.height  # (32)
image = Image.new("1", (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0, 0, width, height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height - padding

# Load default font.
font = ImageFont.load_default()

# set progress bar width and height relative to board's display
BAR_WIDTH = disp.width - 40
BAR_HEIGHT = 16

# Alternatively load a TTF font.  Make sure the .ttf font file is in the
# same directory as the python script!
# Some other nice fonts to try: http://www.dafont.com/bitmap.php
# font = ImageFont.truetype('/usr/share/fonts/truetype/fonts-deva-extra/samanata.ttf', index=0, size=9)


# progress is 0 to 100
def drawProgressbar(x, y, width, height, progress):
    bar = (width / 100) * max(min(progress, 100), 0)
    draw.rectangle((x, y, x + width - 2, y + height), outline=255, fill=0)
    draw.rectangle((x, y, x + bar, y + height), outline=None, fill=1)


while True:
    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, width, height), outline=0, fill=0)

    # Shell scripts for system monitoring from here:
    # https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
    cmd = "hostname -I | cut -d' ' -f1"
    IP = subprocess.check_output(cmd, shell=True).decode("utf-8")
    cmd = 'cut -f 1 -d " " /proc/loadavg'
    CPU = subprocess.check_output(cmd, shell=True).decode("utf-8")
    # cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%s MB  %.2f%%\", $3,$2,$3*100/$2 }'"
    # MemUsage = subprocess.check_output(cmd, shell=True).decode("utf-8")
    # cmd = 'df -h | awk \'$NF=="/"{printf "Disk: %d/%d GB  %s", $3,$2,$5}\''
    # Disk = subprocess.check_output(cmd, shell=True).decode("utf-8")

    draw.text((0, top + 0), "IP: " + IP, font=font, fill=255)
    draw.text((0, top + 8), f"CPU load: {CPU} ", font=font, fill=255)
    draw.text((0, top + 16), "CPU", font=font, fill=255)
    # voltage = bot.get_battery_voltage()
    cpu_temp = int(gz.CPUTemperature().temperature)
    draw.text((0, top + 24), f"{cpu_temp} °C", font=font, fill=255)

    drawProgressbar(40, top + 19, width - 40, 14, cpu_temp)

    disp.image(image)
    disp.show()
    time.sleep(1)
