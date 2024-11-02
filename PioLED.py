from board import SCL, SDA
import busio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont


class PioLED:
    def __init__(self, width=128, height=32):
        self.width = width
        self.height = height

        # Initialize I2C
        i2c = busio.I2C(SCL, SDA)

        # Create the SSD1306 OLED display
        self.oled = adafruit_ssd1306.SSD1306_I2C(width, height, i2c)

        # Create blank image for drawing
        self.image = Image.new("1", (width, height))
        self.draw = ImageDraw.Draw(self.image)

        # Load default font
        self.font = ImageFont.load_default()

    # Alternatively load a TTF font.  Make sure the .ttf font file is in the
    # same directory as the python script!
    # Some other nice fonts to try: http://www.dafont.com/bitmap.php
    # font = ImageFont.truetype('/usr/share/fonts/truetype/fonts-deva-extra/samanata.ttf', index=0, size=9)

    def clear(self):
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        self.oled.fill(0)
        self.oled.show()

    def display_text(self, text, x=0, y=0):
        self.draw.text((x, y), text, font=self.font, fill=255)
        self.oled.image(self.image)
        self.oled.show()

    def draw_rectangle(self, x, y, width, height, fill=False):
        if fill:
            self.draw.rectangle((x, y, x + width, y + height), outline=255, fill=255)
        else:
            self.draw.rectangle((x, y, x + width, y + height), outline=255, fill=0)
        self.oled.image(self.image)
        self.oled.show()

    def draw_line(self, x0, y0, x1, y1):
        self.draw.line((x0, y0, x1, y1), fill=255)
        self.oled.image(self.image)
        self.oled.show()

    def set_font(self, font_path, font_size):
        self.font = ImageFont.truetype(font_path, font_size)

    def display_image(self, image_path):
        image = (
            Image.open(image_path)
            .resize((self.width, self.height), Image.ANTIALIAS)
            .convert("1")
        )
        self.oled.image(image)
        self.oled.show()
