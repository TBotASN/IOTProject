#!/usr/bin/env python3
"""
test_sensors.py — Individual sensor test script for the Smart Waste Management System.

Run this script to verify each sensor is wired correctly and returning valid data.
Each test can be run independently from the menu.

Hardware (BCM pin numbering):
  HC-SR04    TRIG→GPIO23 (Pin 16), ECHO→GPIO24 (Pin 18) via voltage divider 5V→3.3V
  IR wave    GPIO17 (Pin 11)  — hand-wave sensor (replaces PIR)
  IR prox    GPIO22 (Pin 15)  — lid open/closed detection (active-low)
  DHT22      GPIO4  (Pin 7)
  LED        GPIO27 (Pin 13)  — motor indicator
  SenseHAT   Jumper cables — see pin table below (I2C, SPI, power, EEPROM)
  LCD 16×2   I2C SDA GPIO2 (Pin 3), SCL GPIO3 (Pin 5), 1602I2C at 0x3e
  Camera     CSI ribbon connector (not GPIO)
"""

import time
import sys
import os

# ── Pin constants (mirrors config.py) ─────────────────────────────────────────
PIN_TRIG    = 23
PIN_ECHO    = 24
PIN_WAVE_IR = 17   # IR wave sensor — hand-wave to open lid
PIN_IR      = 22   # IR proximity — lid open/closed (active-low)
PIN_DHT     = 4
PIN_LED     = 27   # Motor indicator LED
BIN_DEPTH_CM = 30

# ── TFLite paths (mirrors config.py) ──────────────────────────────────────────
TFLITE_PATH          = "/home/group28/Documents/IOTProject/IOTProject/waste_system/trash_classifier.tflite"
LABELS_PATH          = "/home/group28/Documents/IOTProject/IOTProject/waste_system/labels.txt"
IMG_SIZE             = 224
CONFIDENCE_THRESHOLD = 0.6

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
  3.3V  (SenseHAT pwr)  ● [ 1]──[ 2] ●   5V   (SenseHAT / IR / LCD)
  SDA   (I2C: LCD+HAT)  ● [ 3]──[ 4] ●   5V
  SCL   (I2C: LCD+HAT)  ● [ 5]──[ 6] ●   GND  (SenseHAT)
  DHT22 DATA            ● [ 7]──[ 8] ·
  GND                   · [ 9]──[10] ·
  IR WAVE OUT           ● [11]──[12] ·
  LED (motor ind.)      ● [13]──[14] ·   GND
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
║ IR wave sensor   ║  Pin 11       ║  GPIO17    ║ Input, RISING = hand wave detected   ║
╠══════════════════╬═══════════════╬════════════╬══════════════════════════════════════╣
║ IR proximity     ║  Pin 15       ║  GPIO22    ║ Input, active-low (0=lid closed)     ║
╠══════════════════╬═══════════════╬════════════╬══════════════════════════════════════╣
║ LED indicator    ║  Pin 13       ║  GPIO27    ║ Output — on when lid should open     ║
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


def test_wave_ir():
    """IR wave sensor — BCM GPIO17 (Pin 11). Detects hand wave to open lid."""
    section("IR Wave Sensor  [GPIO17 / Pin 11]")
    print("  Wiring: VCC→5V (Pin2), GND→GND (Pin6), OUT→GPIO17 (Pin11)")
    print("  HIGH = hand detected, LOW = no hand")
    print("  Listening for 15 seconds — wave your hand in front of the sensor...\n")
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_WAVE_IR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        count = 0
        deadline = time.time() + 15
        last_state = None

        while time.time() < deadline:
            state = GPIO.input(PIN_WAVE_IR)
            if state != last_state:
                if state == GPIO.HIGH:
                    count += 1
                    print(f"  [OK]  Hand detected! (event #{count})")
                else:
                    print(f"  [--]  No hand")
                last_state = state
            time.sleep(0.1)

        GPIO.cleanup([PIN_WAVE_IR])

        if count == 0:
            warn("No detections. Check wiring or adjust sensitivity potentiometer on sensor.")
        else:
            ok(f"Total wave events: {count}")
    except ImportError:
        err("RPi.GPIO not installed. Run: pip install RPi.GPIO")
    except Exception as exc:
        err(f"Unexpected error: {exc}")


def test_led():
    """Motor indicator LED — BCM GPIO27 (Pin 13)."""
    section("Motor Indicator LED  [GPIO27 / Pin 13]")
    print("  Wiring: Anode→GPIO27 (Pin13) via resistor, Cathode→GND")
    print("  LED will blink 3 times then stay on for 2 s, then off.\n")
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_LED, GPIO.OUT, initial=GPIO.LOW)

        print("  Blinking 3 times...")
        for i in range(3):
            GPIO.output(PIN_LED, GPIO.HIGH)
            print(f"    ON  (blink {i+1})")
            time.sleep(0.4)
            GPIO.output(PIN_LED, GPIO.LOW)
            print(f"    OFF")
            time.sleep(0.4)

        print("  Holding ON for 2 seconds...")
        GPIO.output(PIN_LED, GPIO.HIGH)
        time.sleep(2)
        GPIO.output(PIN_LED, GPIO.LOW)
        GPIO.cleanup([PIN_LED])
        ok("LED test complete. Confirm you saw 3 blinks + 2 s solid.")
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


def test_sensehat_leds():
    """SenseHAT 8×8 LED matrix — standalone colour and pattern test."""
    section("SenseHAT 8×8 LED Matrix  [SPI GPIO7-11 + I2C]")
    print("  SPI wiring required: Pin19→MOSI  Pin21→MISO  Pin23→SCLK  Pin24→CE0  Pin26→CE1")
    print("  Power:  Pin1→3.3V  Pin2→5V  Pin6→GND\n")

    try:
        from sense_hat import SenseHat
        sense = SenseHat()

        # ── Solid colour cycle ────────────────────────────────────────────────
        print("  [1/3] Solid colour cycle: RED → GREEN → BLUE → YELLOW → OFF")
        for label, colour in [("RED", RED), ("GREEN", GREEN), ("BLUE", BLUE), ("YELLOW", YELLOW), ("OFF", OFF)]:
            sense.clear(colour)
            print(f"    {label}...", end=" ", flush=True)
            time.sleep(0.8)
            print("done")
        ok("Solid colour cycle complete.")

        # ── Fill level simulation ─────────────────────────────────────────────
        print("\n  [2/3] Fill-level simulation (green→red as fill increases)")
        for fill in range(0, 101, 10):
            colour = RED if fill >= 80 else GREEN
            sense.clear(colour)
            print(f"    fill={fill:3d}%  colour={'RED  ' if fill >= 80 else 'GREEN'}", end="\r", flush=True)
            time.sleep(0.3)
        print()
        sense.clear()
        ok("Fill-level simulation complete.")

        # ── Pixel-level walk ──────────────────────────────────────────────────
        print("\n  [3/3] Single-pixel walk across all 64 LEDs (BLUE)")
        sense.clear()
        for i in range(64):
            x, y = i % 8, i // 8
            sense.set_pixel(x, y, BLUE)
            time.sleep(0.03)
            sense.set_pixel(x, y, OFF)
        sense.clear()
        ok("Pixel walk complete.")

        ok("LED matrix test PASSED — all 64 pixels exercised.")

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
    cam = None
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
    finally:
        if cam is not None:
            cam.close()


def test_camera_live():
    """Real-time camera feed with TFLite inference overlay — proves camera works live."""
    section("Camera — Live Video  [CSI ribbon connector]")
    print("  Streams live video from the Pi Camera with TFLite classification overlay.")
    print("  Press 'q' in the OpenCV window (or Ctrl+C here) to stop.")
    print("  Runs for up to 60 seconds.\n")
    print("  NOTE: Requires a display (HDMI / VNC / X forwarding).")
    print("  Headless fallback: saves a labelled frame every 3 s to /tmp/\n")

    try:
        import cv2
        import numpy as np
        from picamera2 import Picamera2
    except ImportError as e:
        err(f"Missing dependency: {e}  —  run: pip install opencv-python picamera2")
        return

    # ── Try to load TFLite model ──────────────────────────────────────────────
    interp = inp_d = outp_d = class_names = None
    try:
        try:
            import tflite_runtime.interpreter as tflite
        except ImportError:
            import tensorflow.lite as tflite  # type: ignore
        with open(LABELS_PATH) as f:
            class_names = [line.strip() for line in f]
        interp = tflite.Interpreter(model_path=TFLITE_PATH)
        interp.allocate_tensors()
        inp_d  = interp.get_input_details()
        outp_d = interp.get_output_details()
        ok(f"TFLite model loaded — {len(class_names)} classes: {class_names}")
    except FileNotFoundError:
        warn(f"TFLite model not found at {TFLITE_PATH} — showing live feed without inference")
    except Exception as exc:
        warn(f"TFLite load failed ({exc}) — showing live feed without inference")

    # ── Detect headless mode ──────────────────────────────────────────────────
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if not has_display:
        warn("No DISPLAY detected — headless mode: saving frames to /tmp/live_frame_*.jpg")

    # ── Open camera ───────────────────────────────────────────────────────────
    try:
        cam = Picamera2()
        cfg = cam.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},
        )
        cam.configure(cfg)
        cam.start()
        time.sleep(1)
        ok("Camera started (640×480 RGB)")
    except Exception as exc:
        err(f"Camera failed to open: {exc}")
        return

    def _infer(frame_rgb):
        """Return (label, pct_str) or ('', '') if no model."""
        if interp is None:
            return "", ""
        img = cv2.resize(frame_rgb, (IMG_SIZE, IMG_SIZE))
        arr = np.expand_dims(img.astype(np.float32) / 255.0, axis=0)
        interp.set_tensor(inp_d[0]['index'], arr)
        interp.invoke()
        scores = interp.get_tensor(outp_d[0]['index'])[0]
        idx    = int(np.argmax(scores))
        conf   = float(scores[idx])
        label  = class_names[idx] if conf >= CONFIDENCE_THRESHOLD else "uncertain"
        return label, f"{conf:.0%}"

    print("\n  Streaming... (press 'q' in window or Ctrl+C to stop)\n")
    frame_count  = 0
    save_counter = 0
    deadline     = time.time() + 60

    try:
        while time.time() < deadline:
            frame = cam.capture_array()        # RGB888 numpy array (H×W×3)
            frame_count += 1

            label, pct = _infer(frame)

            # Build display frame (convert RGB→BGR for OpenCV)
            display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Overlay label
            if label:
                colour = (0, 200, 0) if label != "uncertain" else (0, 140, 255)
                cv2.putText(display, f"{label}  {pct}", (10, 36),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.1, colour, 2, cv2.LINE_AA)
            cv2.putText(display, f"frame {frame_count}", (10, display.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

            if has_display:
                cv2.imshow("Pi Camera — Live (press q to quit)", display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("  [q pressed — stopping]")
                    break
            else:
                # Headless: save every 3 s
                if frame_count % 90 == 1:
                    path = f"/tmp/live_frame_{save_counter:03d}.jpg"
                    cv2.imwrite(path, display)
                    save_counter += 1
                    size = os.path.getsize(path)
                    msg  = f"{label} {pct}" if label else "no inference"
                    ok(f"Saved {path}  ({size} bytes)  [{msg}]")

            if label:
                print(f"  frame {frame_count:4d}  →  {label:15s} {pct}", end="\r", flush=True)

    except KeyboardInterrupt:
        print("\n  [Ctrl+C — stopping]")
    finally:
        cam.stop()
        cam.close()
        if has_display:
            cv2.destroyAllWindows()

    print(f"\n\n  Total frames captured: {frame_count}")
    ok("Live camera test complete.")


# ─────────────────────────────────────────────────────────────────────────────
#  Run all sensors
# ─────────────────────────────────────────────────────────────────────────────

def test_all():
    """Run every sensor test in sequence."""
    section("Running ALL sensor tests")
    print("  Each test runs independently. Failures in one do not stop the rest.\n")

    tests = [
        ("HC-SR04 Ultrasonic",   test_ultrasonic),
        ("IR Wave Sensor",       test_wave_ir),
        ("IR Proximity Sensor",  test_ir),
        ("DHT22 Temp/Humidity",  test_dht22),
        ("LED Indicator",        test_led),
        ("SenseHAT",             test_sensehat),
        ("SenseHAT LEDs",        test_sensehat_leds),
        ("I2C LCD Display",      test_lcd),
        ("Camera (still)",       test_camera),
        ("Camera (live video)",  test_camera_live),
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
║  2.  Test IR Wave Sensor      (GPIO17)                   ║
║  3.  Test IR Proximity Sensor (GPIO22)                   ║
║  4.  Test DHT22 Temp/Humidity (GPIO4)                    ║
║  W.  Test LED Indicator       (GPIO27)                   ║
║  5.  Test SenseHAT            (jumper cables)            ║
║  S.  Test SenseHAT LEDs only  (8×8 matrix)               ║
║  6.  Test I2C LCD Display     (GPIO2/3, addr 0x3e)       ║
║  7.  Test Camera              (CSI ribbon — still)       ║
║  8.  Run ALL sensors                                     ║
║  9.  Exit                                                ║
║  L.  Live Camera + TFLite inference (real-time video)    ║
╚══════════════════════════════════════════════════════════╝
"""

ACTIONS = {
    "0": ("Show pin reference table",               lambda: print(PIN_TABLE)),
    "1": ("HC-SR04 Ultrasonic",                     test_ultrasonic),
    "2": ("IR Wave Sensor",                          test_wave_ir),
    "3": ("IR Proximity Sensor",                     test_ir),
    "4": ("DHT22 Temp/Humidity",                     test_dht22),
    "w": ("LED Indicator",                           test_led),
    "W": ("LED Indicator",                           test_led),
    "5": ("SenseHAT",                               test_sensehat),
    "s": ("SenseHAT LEDs only",                     test_sensehat_leds),
    "S": ("SenseHAT LEDs only",                     test_sensehat_leds),
    "6": ("I2C LCD Display",                        test_lcd),
    "7": ("Camera (still capture)",                 test_camera),
    "8": ("Run ALL sensors",                        test_all),
    "l": ("Live camera + TFLite (real-time video)", test_camera_live),
    "L": ("Live camera + TFLite (real-time video)", test_camera_live),
}


def main():
    print("\nSensor Test Script — Smart Office Waste Management System")
    print("Run as root or with GPIO permissions: sudo python3 test_sensors.py\n")
    print(PIN_TABLE)

    while True:
        print(MENU)
        try:
            choice = input("Enter option (0-9, W, S, L): ").strip()
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
