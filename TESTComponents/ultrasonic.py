import RPi.GPIO as GPIO
import time

TRIG = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

empty_distance = 40   # cm
full_distance = 5     # cm

def calcDistance():
    GPIO.output(TRIG, False)
    time.sleep(0.2)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    #distance = round(distance, 2)
    
    return round(distance, 2)
    #print("Distance:", distance, "cm")

    #time.sleep(1)
def fillPercentage(distance):
    fillPercent = ((empty_distance - distance) / (empty_distance - full_distance)) * 100
    fillPercent = max(0, min(100, fillPercent)) #make sure the fill percent is not negative or above 100
    return round(fillPercent, 1) #one decimal point
def getStatus(fillPercent): #seperate function, together harder to print fill and status individually
    if fillPercent < 30:
        return "Empty"
    elif fillPercent < 70:
        return "Half Full"
    else:
        return "Full"
    
try:
    while True:
        distance = calcDistance()
        
        if distance is not None: #distance != 0:
            fillPercent = fillPercentage(distance)
            status = getStatus(fillPercent)
            
            print(f"Distance: {distance} cm")
            print(f"Fill Level: {fillPercent}%")
            print(f"Status: {status}")
            print("----------------------")
        else:
            print("Sensor reading failed")

        time.sleep(2)
            
except KeyboardInterrupt:
    GPIO.cleanup()