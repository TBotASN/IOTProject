import RPi.GPIO as GPIO
import time

PIN = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.IN)

try:
    while True:
        if GPIO.input(PIN) == 0:
            print("Object detected")
        else:
            print("No object")

        time.sleep(0.5)

except KeyboardInterrupt:
    GPIO.cleanup()