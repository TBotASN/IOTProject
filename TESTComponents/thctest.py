import adafruit_dht
import board
import time

dhtDevice = adafruit_dht.DHT22(board.D17)

while True:
    try:
        temperature = dhtDevice.temperature
        humidity = dhtDevice.humidity

        print("Temperature:", temperature, "C")
        print("Humidity:", humidity, "%")
        print("-------------------")

    except RuntimeError as error:
        print("Retrying:", error)

    time.sleep(2)