import time
import RPi.GPIO as GPIO
import Adafruit_CharLCD as LCD
import Adafruit_DHT

LedPin = 17
RedLedPin = 12
PompPin = 4

LCD_RS = 7
LCD_E = 8
LCD_D4 = 25
LCD_D5 = 24
LCD_D6 = 23
LCD_D7 = 18

LCD_WIDTH = 24
LCD_CHR = True
LCD_CMD = False

LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0

E_PULSE = 0.00005
E_DELAY = 0.00005

lcd_columns = 16
lcd_rows = 2

sensor = Adafruit_DHT.DHT11
sensorPin = 5

lcd = LCD.Adafruit_CharLCD(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7, lcd_columns, lcd_rows)


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(22, GPIO.IN)
    GPIO.setup(LedPin, GPIO.OUT)
    GPIO.setup(RedLedPin, GPIO.OUT)
    GPIO.setup(PompPin, GPIO.OUT)
    GPIO.output(LedPin, GPIO.LOW)
    GPIO.output(RedLedPin, GPIO.LOW)
    GPIO.output(PompPin, GPIO.LOW)
    GPIO.setup(LCD_E, GPIO.OUT)
    GPIO.setup(LCD_RS, GPIO.OUT)
    GPIO.setup(LCD_D4, GPIO.OUT)
    GPIO.setup(LCD_D5, GPIO.OUT)
    GPIO.setup(LCD_D6, GPIO.OUT)
    GPIO.setup(LCD_D7, GPIO.OUT)
    lcd_init()
    try:
        while True:
            if (GPIO.input(22)) == 0:
                GPIO.output(LedPin, GPIO.HIGH)
                GPIO.output(RedLedPin, GPIO.LOW)
                lcd_byte(LCD_LINE_1, LCD_CMD)
                lcd_string("     KWIAT      ")
                lcd_byte(LCD_LINE_2, LCD_CMD)
                lcd_string("    PODLANY     ")
                time.sleep(3.0)
                lcd.clear()
                #GPIO.output(PompPin, GPIO.LOW)
                humidity, temperature = Adafruit_DHT.read_retry(sensor, sensorPin)
                lcd_byte(LCD_LINE_1, LCD_CMD)
                lcd_string("Temp: %d C" % temperature)
                lcd_byte(LCD_LINE_2, LCD_CMD)
                lcd_string("Humidity: %d %%" % humidity)
                time.sleep(3.0)
                lcd.clear()
                time.sleep(0.25)
            elif (GPIO.input(22)) == 1:
                GPIO.output(LedPin, GPIO.LOW)
                GPIO.output(RedLedPin, GPIO.HIGH)
                lcd_byte(LCD_LINE_1, LCD_CMD)
                lcd_string("    PODLEJ!    ")
                time.sleep(3.0)
                lcd.clear()
                #GPIO.output(PompPin, GPIO.HIGH)
                humidity, temperature = Adafruit_DHT.read_retry(sensor, sensorPin)
                lcd_byte(LCD_LINE_1, LCD_CMD)
                lcd_string("Temp: %d C" % temperature)
                lcd_byte(LCD_LINE_2, LCD_CMD)
                lcd_string("Humidity: %d %%" % humidity)
                time.sleep(3.0)
                lcd.clear()
                time.sleep(0.25)

        # time.sleep(0.25)
    finally:
        GPIO.cleanup()

def lcd_init():
        lcd_byte(0x33, LCD_CMD)
        lcd_byte(0x32, LCD_CMD)
        lcd_byte(0x28, LCD_CMD)
        lcd_byte(0x0C, LCD_CMD)
        lcd_byte(0x06, LCD_CMD)
        lcd_byte(0x01, LCD_CMD)

def lcd_string(message):
    message = message.ljust(LCD_WIDTH, " ")

    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]), LCD_CHR)

def lcd_byte(bits, mode):
        GPIO.output(LCD_RS, mode)

        # High bits
        GPIO.output(LCD_D4, False)
        GPIO.output(LCD_D5, False)
        GPIO.output(LCD_D6, False)
        GPIO.output(LCD_D7, False)
        if bits & 0x10 == 0x10:
            GPIO.output(LCD_D4, True)
        if bits & 0x20 == 0x20:
            GPIO.output(LCD_D5, True)
        if bits & 0x40 == 0x40:
            GPIO.output(LCD_D6, True)
        if bits & 0x80 == 0x80:
            GPIO.output(LCD_D7, True)

        time.sleep(E_DELAY)
        GPIO.output(LCD_E, True)
        time.sleep(E_PULSE)
        GPIO.output(LCD_E, False)
        time.sleep(E_DELAY)

        GPIO.output(LCD_D4, False)
        GPIO.output(LCD_D5, False)
        GPIO.output(LCD_D6, False)
        GPIO.output(LCD_D7, False)
        if bits&0x01==0x01:
                GPIO.output(LCD_D4, True)
        if bits&0x02==0x02:
                GPIO.output(LCD_D5, True)
        if bits&0x04==0x04:
                GPIO.output(LCD_D6, True)
        if bits&0x08==0x08:
                GPIO.output(LCD_D7, True)

        time.sleep(E_DELAY)
        GPIO.output(LCD_E, True)
        time.sleep(E_PULSE)
        GPIO.output(LCD_E, False)
        time.sleep(E_DELAY)

if __name__ == '__main__':
        main()


