import RPi.GPIO as GPIO
import time

PIN_PIR = 17  # BCM pin for PIR motion sensor

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_PIR, GPIO.IN)

print("PIR sensor test started. Press Ctrl+C to stop.")
print("Waiting for motion...")

try:
    while True:
        if GPIO.input(PIN_PIR) == GPIO.HIGH:
            print("Motion detected!")
        else:
            print("No motion.")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nTest stopped.")
    GPIO.cleanup()
