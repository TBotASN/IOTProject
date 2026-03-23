#!/usr/bin/env python3
"""
test_sensors.py — Individual sensor test script for the Smart Waste Management System.

Run this script to verify each sensor is wired correctly and returning valid data.
Each test can be run independently from the menu.

Hardware (BCM pin numbering):
  HC-SR04   TRIG→GPIO23 (Pin 16), ECHO→GPIO24 (Pin 18) via voltage divider 5V→3.3V
  PIR        GPIO17 (Pin 11)
  IR prox    GPIO22 (Pin 15)
  DHT22      GPIO4  (Pin 7)
  SenseHAT   Jumper cables — see pin table below (I2C, SPI, power, EEPROM)
  LCD 16×2   I2C SDA GPIO2 (Pin 3), SCL GPIO3 (Pin 5), 1602I2C at 0x3e
  Camera     CSI ribbon connector (not GPIO)
"""

import time
import sys

# ── Pin constants (mirrors config.py) ─────────────────────────────────────────
PIN_TRIG = 23
PIN_ECHO = 24
PIN_PIR  = 17
PIN_IR   = 22
PIN_DHT  = 4
BIN_DEPTH_CM = 30

# ── Colours ────────────────────────────────────────────────────────────────────
RED    = (200, 0,   0)
GREEN  = (0,   200, 0)
BLUE   = (0,   0,   200)
YELLOW = (200, 200, 0)
OFF    = (0,   0,   0)


# ─────────────────────────────────────────────────────────────────────────────
#  Pin reference table
# ─────────────────────────────────────────────────────────────────────────────

PIN_TABLE = """
── Pi 40-pin GPIO Header (physical pin layout) ──────────────────────────────
  Label                    Pin        Pin   Label
  ──────────────────────────────────────────────────────────────────────────
  3.3V  (SenseHAT pwr)  ● [ 1]──[ 2] ●   5V   (SenseHAT / PIR / IR / LCD)
  SDA   (I2C: LCD+HAT)  ● [ 3]──[ 4] ●   5V
  SCL   (I2C: LCD+HAT)  ● [ 5]──[ 6] ●   GND  (SenseHAT)
  DHT22 DATA            ● [ 7]──[ 8] ·
  GND                   · [ 9]──[10] ·
  PIR OUT               ● [11]──[12] ·
  —                     · [13]──[14] ·   GND
  IR DOUT               ● [15]──[16] ●   HC-SR04 TRIG
  3.3V                  · [17]──[18] ●   HC-SR04 ECHO  ← 5V→3.3V divider
  SenseHAT MOSI         ● [19]──[20] ·   GND
  SenseHAT MISO         ● [21]──[22] ·
  SenseHAT SCLK         ● [23]──[24] ●   SenseHAT CE0
  GND                   · [25]──[26] ●   SenseHAT CE1
  SenseHAT EEPROM SDA   ● [27]──[28] ●   SenseHAT EEPROM SCL
  —                     · [29]──[30] ·   GND
  —                     · [31]──[32] ·
  —                     · [33]──[34] ·   GND
  —                     · [35]──[36] ·
  —                     · [37]──[38] ·
  GND                   · [39]──[40] ·
  ──────────────────────────────────────────────────────────────────────────
  ● = connected to this system    · = unused pin

╔══════════════════╦═══════════════╦════════════╦══════════════════════════════════════╗
║ Sensor/Device    ║ Physical Pin  ║  BCM GPIO  ║ Notes                                ║
╠══════════════════╬═══════════════╬════════════╬══════════════════════════════════════╣
║ HC-SR04 TRIG     ║  Pin 16       ║  GPIO23    ║ Output — send 10µs pulse             ║
║ HC-SR04 ECHO     ║  Pin 18       ║  GPIO24    ║ Input  — NEEDS voltage divider!      ║
║                  ║               ║            ║ (5V→3.3V: 1kΩ + 2kΩ resistors)      ║
╠══════════════════╬═══════════════╬════════════╬══════════════════════════════════════╣
║ PIR motion       ║  Pin 11       ║  GPIO17    ║ Input, RISING edge trigger           ║
╠══════════════════╬═══════════════╬════════════╬══════════════════════════════════════╣
║ IR proximity     ║  Pin 15       ║  GPIO22    ║ Input, active-low (0=detected)       ║
╠══════════════════╬═══════════════╬════════════╬══════════════════════════════════════╣
║ DHT22            ║  Pin 7        ║  GPIO4     ║ 1-wire data (adafruit-circuitpython) ║
╠══════════════════╬═══════════════╬════════════╬══════════════════════════════════════╣
║ SenseHAT — 3.3V  ║  Pin 1        ║  —         ║ 3.3V power to SenseHAT               ║
║ SenseHAT — 5V    ║  Pin 2        ║  —         ║ 5V power to SenseHAT                 ║
║ SenseHAT — GND   ║  Pin 6        ║  —         ║ Ground                               ║
║ SenseHAT — I2C   ║  Pin 3/5      ║  GPIO2/3   ║ SDA/SCL — all sensors + ATtiny LED   ║
║ SenseHAT — MOSI  ║  Pin 19       ║  GPIO10    ║ SPI — LED matrix framebuffer         ║
║ SenseHAT — MISO  ║  Pin 21       ║  GPIO9     ║ SPI — LED matrix framebuffer         ║
║ SenseHAT — SCLK  ║  Pin 23       ║  GPIO11    ║ SPI — LED matrix framebuffer         ║
║ SenseHAT — CE0   ║  Pin 24       ║  GPIO8     ║ SPI Chip Enable 0                    ║
║ SenseHAT — CE1   ║  Pin 26       ║  GPIO7     ║ SPI Chip Enable 1                    ║
║ SenseHAT — EEPROM║  Pin 27/28    ║  GPIO0/1   ║ HAT ID EEPROM (ID_SD / ID_SC)        ║
╠══════════════════╬═══════════════╬════════════╬══════════════════════════════════════╣
║ LCD 16×2 (I2C)   ║  Pin 3/5      ║  GPIO2/3   ║ 1602I2C at 0x3e                      ║
╠══════════════════╬═══════════════╬════════════╬══════════════════════════════════════╣
║ Camera           ║  CSI ribbon   ║  (CSI)     ║ Not GPIO — uses dedicated CSI port   ║
╠══════════════════╬═══════════════╬════════════╬══════════════════════════════════════╣
║ 3.3V power       ║  Pin 1, 17    ║  —         ║ For sensors requiring 3.3V           ║
║ 5V power         ║  Pin 2, 4     ║  —         ║ For HC-SR04, PIR, IR, LCD backlight  ║
║ Ground           ║  Pin 6,9,14,  ║  —         ║ Common ground for all sensors        ║
║                  ║  20,25,30,    ║            ║                                      ║
║                  ║  34,39        ║            ║                                      ║
╚══════════════════╩═══════════════╩════════════╩══════════════════════════════════════╝

SenseHAT jumper cable wiring (sensors-only vs full):
  MINIMUM (sensors only — temp, humidity, pressure, IMU):
    Pin 1  → 3.3V       Pin 2  → 5V        Pin 6  → GND
    Pin 3  → SDA        Pin 5  → SCL

  FULL (adds LED matrix and joystick via framebuffer):
    All above, plus:
    Pin 19 → MOSI       Pin 21 → MISO      Pin 23 → SCLK
    Pin 24 → CE0        Pin 26 → CE1

  EEPROM (HAT identification — needed for sense-hat library auto-detect):
    Pin 27 → ID_SD (GPIO0)    Pin 28 → ID_SC (GPIO1)

  I2C device addresses on the SenseHAT:
    0x5F — HTS221  (humidity + temperature)
    0x5C — LPS25H  (pressure + temperature)
    0x1C — LSM9DS1 accelerometer/magnetometer
    0x6A — LSM9DS1 gyroscope
    0x46 — ATtiny88 (joystick controller)

  NOTE: GPIO4 (Pin 7) is NOT used by the SenseHAT when wired via jumpers,
  so DHT22 on GPIO4 is safe to use alongside it.
"""


# ─────────────────────────────────────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────────────────────────────────────

def section(title: str):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print('='*60)


def ok(msg: str):
    print(f"  [OK]  {msg}")


def warn(msg: str):
    print(f"  [WARN] {msg}")


def err(msg: str):
    print(f"  [FAIL] {msg}")


# ─────────────────────────────────────────────────────────────────────────────
#  Test functions
# ─────────────────────────────────────────────────────────────────────────────

def test_ultrasonic():
    """HC-SR04 ultrasonic distance sensor — BCM GPIO23 (TRIG) / GPIO24 (ECHO)."""
    section("HC-SR04 Ultrasonic Sensor  [TRIG=GPIO23/Pin16  ECHO=GPIO24/Pin18]")
    print("  NOTE: ECHO pin requires a voltage divider (5V→3.3V).")
    print("  Taking 5 readings...\n")
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_TRIG, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_ECHO, GPIO.IN)
        time.sleep(0.5)  # settle

        for i in range(5):
            GPIO.output(PIN_TRIG, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(PIN_TRIG, GPIO.LOW)

            timeout = time.time() + 0.05
            while GPIO.input(PIN_ECHO) == GPIO.LOW:
                if time.time() > timeout:
                    warn(f"Reading {i+1}: echo LOW timeout — check wiring on GPIO24")
                    break
            else:
                pulse_start = time.time()
                timeout = time.time() + 0.05
                while GPIO.input(PIN_ECHO) == GPIO.HIGH:
                    if time.time() > timeout:
                        warn(f"Reading {i+1}: echo HIGH timeout — object too close or wiring issue")
                        break
                else:
                    pulse_end = time.time()
                    distance_cm = round((pulse_end - pulse_start) * 17150, 2)
                    fill = max(0.0, min(100.0, round(((BIN_DEPTH_CM - distance_cm) / BIN_DEPTH_CM) * 100, 1)))
                    ok(f"Reading {i+1}: {distance_cm} cm  →  fill {fill}%")
            time.sleep(1)

        GPIO.cleanup([PIN_TRIG, PIN_ECHO])
        print("\n  Done. Expected range: 2–400 cm. Readings of -1 or timeout = wiring problem.")
    except ImportError:
        err("RPi.GPIO not installed. Run: pip install RPi.GPIO")
    except Exception as exc:
        err(f"Unexpected error: {exc}")


def test_pir():
    """PIR motion sensor — BCM GPIO17 (Pin 11)."""
    section("PIR Motion Sensor  [GPIO17 / Pin 11]")
    print("  Wiring: VCC→5V (Pin2), GND→GND (Pin6), OUT→GPIO17 (Pin11)")
    print("  Listening for 15 seconds — wave your hand in front of the sensor...\n")
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_PIR, GPIO.IN)

        count = 0
        deadline = time.time() + 15
        last_state = None

        while time.time() < deadline:
            state = GPIO.input(PIN_PIR)
            if state != last_state:
                if state == GPIO.HIGH:
                    count += 1
                    print(f"  [OK]  Motion detected! (event #{count})")
                else:
                    print(f"  [--]  No motion")
                last_state = state
            time.sleep(0.5)

        GPIO.cleanup([PIN_PIR])

        if count == 0:
            warn("No motion detected. Check wiring or sensor sensitivity adjustment.")
        else:
            ok(f"Total events detected: {count}")
    except ImportError:
        err("RPi.GPIO not installed. Run: pip install RPi.GPIO")
    except Exception as exc:
        err(f"Unexpected error: {exc}")


def test_ir():
    """IR proximity sensor — BCM GPIO22 (Pin 15). Active-low: pin reads 0 when object detected."""
    section("IR Proximity Sensor  [GPIO22 / Pin 15]")
    print("  Wiring: VCC→5V (Pin2), GND→GND (Pin6), OUT→GPIO22 (Pin15)")
    print("  Active-low sensor: pin LOW (0) = object detected, HIGH (1) = no object")
    print("  Polling for 15 seconds — pass an object in front of the IR sensor...\n")
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_IR, GPIO.IN)

        count = 0
        deadline = time.time() + 15
        last_state = None

        while time.time() < deadline:
            state = GPIO.input(PIN_IR)
            if state != last_state:
                if state == 0:
                    count += 1
                    print(f"  [OK]  Object detected! (event #{count})")
                else:
                    print(f"  [--]  No object")
                last_state = state
            time.sleep(0.1)

        GPIO.cleanup([PIN_IR])

        if count == 0:
            warn("No IR events. Check wiring or adjust sensitivity potentiometer on sensor.")
        else:
            ok(f"Total detections: {count}")
    except ImportError:
        err("RPi.GPIO not installed. Run: pip install RPi.GPIO")
    except Exception as exc:
        err(f"Unexpected error: {exc}")


def test_dht22():
    """DHT22 temperature/humidity sensor — BCM GPIO4 (Pin 7)."""
    section("DHT22 Temperature & Humidity Sensor  [GPIO4 / Pin 7]")
    print("  Wiring: VCC→3.3V (Pin1), GND→GND (Pin6), DATA→GPIO4 (Pin7)")
    print("  NOTE: Needs a 10kΩ pull-up resistor between DATA and VCC.")
    print("  NOTE: If SenseHAT is on the 40-pin header, GPIO4 may conflict — safe when using jumpers.")
    print("  Taking 3 readings (DHT22 needs ~2s between reads)...\n")
    try:
        import adafruit_dht
        import board

        sensor = adafruit_dht.DHT22(board.D4)
        success = 0
        for i in range(3):
            for attempt in range(3):
                try:
                    temp = sensor.temperature
                    hum  = sensor.humidity
                    if temp is not None and hum is not None:
                        ok(f"Reading {i+1}: {temp:.1f}°C  {hum:.1f}% RH")
                        success += 1
                        break
                except RuntimeError as e:
                    if attempt < 2:
                        time.sleep(0.5)
                    else:
                        warn(f"Reading {i+1}: failed after 3 attempts — {e}")
            time.sleep(2)

        sensor.exit()
        if success == 0:
            err("All readings failed. Check wiring, pull-up resistor, and GPIO4 conflicts.")
        else:
            ok(f"{success}/3 readings successful.")
    except ImportError:
        err("adafruit-circuitpython-dht not installed. Run: pip install adafruit-circuitpython-dht")
    except Exception as exc:
        err(f"Unexpected error: {exc}")


def test_sensehat():
    """SenseHAT — connected via jumper cables (I2C GPIO2/3, SPI GPIO7-11, power)."""
    section("SenseHAT  [jumper cables]")
    print("  Jumper wiring required:")
    print("  • Power:  Pin1→3.3V  Pin2→5V  Pin6→GND")
    print("  • I2C:    Pin3→SDA (GPIO2)  Pin5→SCL (GPIO3)  [sensors + ATtiny]")
    print("  • SPI:    Pin19→MOSI  Pin21→MISO  Pin23→SCLK  Pin24→CE0  Pin26→CE1  [LED matrix]")
    print("  • EEPROM: Pin27→ID_SD (GPIO0)  Pin28→ID_SC (GPIO1)\n")

    try:
        from sense_hat import SenseHat
        sense = SenseHat()

        # ── Environmental sensors ─────────────────────────────────────────────
        print("  --- Environmental Sensors ---")
        temp     = round(sense.get_temperature(), 1)
        temp_hum = round(sense.get_temperature_from_humidity(), 1)
        temp_prs = round(sense.get_temperature_from_pressure(), 1)
        humidity = round(sense.get_humidity(), 1)
        pressure = round(sense.get_pressure(), 1)

        ok(f"Temperature (combined):      {temp}°C")
        ok(f"Temperature (from humidity): {temp_hum}°C")
        ok(f"Temperature (from pressure): {temp_prs}°C")
        ok(f"Humidity:                    {humidity}% RH")
        ok(f"Pressure:                    {pressure} hPa")

        # ── IMU ───────────────────────────────────────────────────────────────
        print("\n  --- IMU (LSM9DS1 — via I2C) ---")
        accel       = sense.get_accelerometer_raw()
        gyro        = sense.get_gyroscope_raw()
        mag         = sense.get_compass_raw()
        orientation = sense.get_orientation_degrees()

        ok(f"Accelerometer (g):  x={accel['x']:.3f}  y={accel['y']:.3f}  z={accel['z']:.3f}")
        ok(f"Gyroscope (rad/s):  x={gyro['x']:.3f}   y={gyro['y']:.3f}   z={gyro['z']:.3f}")
        ok(f"Magnetometer (µT):  x={mag['x']:.1f}    y={mag['y']:.1f}    z={mag['z']:.1f}")
        ok(f"Orientation (°):    roll={orientation['roll']:.1f}  pitch={orientation['pitch']:.1f}  yaw={orientation['yaw']:.1f}")

        # ── Joystick ──────────────────────────────────────────────────────────
        print("\n  --- Joystick ---")
        print("  Press the joystick within 5 seconds to test it...")
        event = sense.stick.wait_for_event(emptybuffer=True)
        if event:
            ok(f"Joystick event: direction={event.direction}  action={event.action}")
        else:
            warn("No joystick event within timeout.")

        # ── LED matrix ────────────────────────────────────────────────────────
        print("\n  --- 8×8 LED Matrix ---")
        print("  Cycling colours: RED → GREEN → BLUE → YELLOW → OFF")
        for label, colour in [("RED", RED), ("GREEN", GREEN), ("BLUE", BLUE), ("YELLOW", YELLOW), ("OFF", OFF)]:
            sense.clear(colour)
            print(f"    {label}...", end=" ", flush=True)
            time.sleep(0.8)
            print("done")

        sense.clear()
        ok("LED matrix test complete.")

    except ImportError:
        err("sense-hat not installed. Run: pip install sense-hat")
    except Exception as exc:
        err(f"Unexpected error: {exc}")


def test_lcd():
    """I2C LCD 16×2 display — SDA GPIO2 (Pin 3), SCL GPIO3 (Pin 5), address 0x3e."""
    section("I2C LCD 16×2 Display  [SDA=GPIO2/Pin3  SCL=GPIO3/Pin5  addr=0x3e]")
    print("  Wiring: VCC→5V (Pin2), GND→GND (Pin6), SDA→GPIO2 (Pin3), SCL→GPIO3 (Pin5)\n")

    try:
        import smbus

        bus  = smbus.SMBus(1)
        addr = 0x3e

        def command(cmd):
            bus.write_byte_data(addr, 0x00, cmd)
            time.sleep(0.002)

        def data(val):
            bus.write_byte_data(addr, 0x40, val)
            time.sleep(0.001)

        def init():
            time.sleep(0.05)
            command(0x38)
            command(0x39)
            command(0x14)
            command(0x70)
            command(0x56)
            command(0x6c)
            time.sleep(0.2)
            command(0x38)
            command(0x0C)
            command(0x01)
            time.sleep(0.01)

        def set_cursor(line, pos):
            if line == 0:
                command(0x80 + pos)
            elif line == 1:
                command(0xC0 + pos)

        def write(text):
            for c in text:
                data(ord(c))

        init()
        ok("LCD initialised at 0x3e")

        command(0x01)
        time.sleep(0.01)
        set_cursor(0, 0)
        write("Sensor Test OK")
        set_cursor(1, 0)
        write("GPIO2/3 I2C")
        ok("Written test message to LCD. You should see 'Sensor Test OK' on line 1.")
        time.sleep(3)

        command(0x01)
        time.sleep(0.01)
        ok("LCD cleared.")
    except ImportError:
        err("smbus not installed. Run: pip install smbus2")
    except Exception as exc:
        err(f"LCD error: {exc}  — check address and I2C wiring")


def test_camera():
    """Raspberry Pi Camera — CSI ribbon connector."""
    section("Camera (Picamera2)  [CSI ribbon connector]")
    print("  The camera uses the dedicated CSI (Camera Serial Interface) port,")
    print("  NOT GPIO pins. Connect the ribbon cable to the CAM port on the Pi.\n")
    print("  Make sure camera is enabled: sudo raspi-config → Interface Options → Camera\n")

    output_path = "/tmp/test_capture.jpg"
    try:
        from picamera2 import Picamera2
        cam = Picamera2()
        cfg = cam.create_still_configuration(main={"size": (1280, 720)})
        cam.configure(cfg)
        cam.start()
        time.sleep(2)  # warm-up
        cam.capture_file(output_path)
        cam.stop()

        import os
        size = os.path.getsize(output_path)
        ok(f"Image captured: {output_path}  ({size} bytes)")
        if size < 5000:
            warn("File is very small — camera may be covered or not functioning correctly.")
    except ImportError:
        err("picamera2 not installed. Run: pip install picamera2")
    except Exception as exc:
        err(f"Camera error: {exc}")


# ─────────────────────────────────────────────────────────────────────────────
#  Run all sensors
# ─────────────────────────────────────────────────────────────────────────────

def test_all():
    """Run every sensor test in sequence."""
    section("Running ALL sensor tests")
    print("  Each test runs independently. Failures in one do not stop the rest.\n")

    tests = [
        ("HC-SR04 Ultrasonic",  test_ultrasonic),
        ("PIR Motion Sensor",   test_pir),
        ("IR Proximity Sensor", test_ir),
        ("DHT22 Temp/Humidity", test_dht22),
        ("SenseHAT",            test_sensehat),
        ("I2C LCD Display",     test_lcd),
        ("Camera",              test_camera),
    ]

    results = {}
    for name, fn in tests:
        try:
            fn()
            results[name] = "PASS"
        except Exception as exc:
            err(f"{name} raised an exception: {exc}")
            results[name] = "ERROR"

    section("Summary")
    for name, result in results.items():
        status = "OK  " if result == "PASS" else "FAIL"
        print(f"  [{status}]  {name}")


# ─────────────────────────────────────────────────────────────────────────────
#  Menu
# ─────────────────────────────────────────────────────────────────────────────

MENU = """
╔══════════════════════════════════════════════════════════╗
║          Sensor Test Menu — Waste Management System      ║
╠══════════════════════════════════════════════════════════╣
║  0.  Show GPIO pin reference table                       ║
║  1.  Test HC-SR04 Ultrasonic  (GPIO23/24)                ║
║  2.  Test PIR Motion Sensor   (GPIO17)                   ║
║  3.  Test IR Proximity Sensor (GPIO22)                   ║
║  4.  Test DHT22 Temp/Humidity (GPIO4)                    ║
║  5.  Test SenseHAT            (jumper cables)            ║
║  6.  Test I2C LCD Display     (GPIO2/3, addr 0x3e)       ║
║  7.  Test Camera              (CSI ribbon)               ║
║  8.  Run ALL sensors                                     ║
║  9.  Exit                                                ║
╚══════════════════════════════════════════════════════════╝
"""

ACTIONS = {
    "0": ("Show pin reference table",  lambda: print(PIN_TABLE)),
    "1": ("HC-SR04 Ultrasonic",        test_ultrasonic),
    "2": ("PIR Motion Sensor",         test_pir),
    "3": ("IR Proximity Sensor",       test_ir),
    "4": ("DHT22 Temp/Humidity",       test_dht22),
    "5": ("SenseHAT",                  test_sensehat),
    "6": ("I2C LCD Display",           test_lcd),
    "7": ("Camera",                    test_camera),
    "8": ("Run ALL sensors",           test_all),
}


def main():
    print("\nSensor Test Script — Smart Office Waste Management System")
    print("Run as root or with GPIO permissions: sudo python3 test_sensors.py\n")
    print(PIN_TABLE)

    while True:
        print(MENU)
        try:
            choice = input("Enter option (0-9): ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting.")
            break

        if choice == "9":
            print("Goodbye.")
            break
        elif choice in ACTIONS:
            label, fn = ACTIONS[choice]
            try:
                fn()
            except KeyboardInterrupt:
                print("\n  [Interrupted]")
        else:
            print("  Invalid option. Enter 0-9.")


if __name__ == "__main__":
    main()
