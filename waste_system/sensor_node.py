#!/usr/bin/env python3
"""
sensor_node.py — Smart Office Waste Management System (Local Mode)
Raspberry Pi 4B sensor loop — writes state to /tmp/waste_state.json for Flask.

Hardware (BCM pin numbering):
  HC-SR04   TRIG→GPIO23, ECHO→GPIO24 (via voltage divider)
  PIR        GPIO17  (hand-wave → LED on, simulates motor open)
  IR prox    GPIO22  (active-low: LOW=lid closed, HIGH=lid open)
  DHT22      GPIO4   (inside bin — temperature + humidity)
  LED        GPIO27  (motor actuation indicator)
  SenseHat   GPIO header (office ambient)
  LCD 16×2   I2C (SDA/SCL) — env display when closed, object when open
  Camera     CSI ribbon (classifies thrown object when lid open)
"""

import time
import json
import threading
import os
import sys
import logging
from datetime import datetime
import RPi.GPIO as GPIO
import adafruit_dht
import board
from sense_hat import SenseHat
import smbus

import config

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("/tmp/waste_system.log"),
    ],
)
log = logging.getLogger(__name__)

# ── GPIO setup ────────────────────────────────────────────────────────────────
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(config.PIN_TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(config.PIN_ECHO, GPIO.IN)
GPIO.setup(config.PIN_PIR,  GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(config.PIN_IR,   GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(config.PIN_LED,  GPIO.OUT, initial=GPIO.LOW)

# ── Sensor objects ────────────────────────────────────────────────────────────
dht_sensor = adafruit_dht.DHT22(board.D4)  # GPIO4
sense      = SenseHat()
sense.clear()

_lcd_bus  = smbus.SMBus(1)
_LCD_ADDR = 0x3e

def _lcd_command(cmd):
    _lcd_bus.write_byte_data(_LCD_ADDR, 0x00, cmd)
    time.sleep(0.002)

def _lcd_data(val):
    _lcd_bus.write_byte_data(_LCD_ADDR, 0x40, val)
    time.sleep(0.001)

def _lcd_init():
    time.sleep(0.05)
    _lcd_command(0x38)
    _lcd_command(0x39)
    _lcd_command(0x14)
    _lcd_command(0x70)
    _lcd_command(0x56)
    _lcd_command(0x6c)
    time.sleep(0.2)
    _lcd_command(0x38)
    _lcd_command(0x0C)
    _lcd_command(0x01)
    time.sleep(0.01)

def _lcd_set_cursor(line, pos):
    if line == 0:
        _lcd_command(0x80 + pos)
    elif line == 1:
        _lcd_command(0xC0 + pos)

def _lcd_write(text):
    for c in text:
        _lcd_data(ord(c))

_lcd_init()


# ── Shared state ──────────────────────────────────────────────────────────────
state_lock      = threading.Lock()
pir_count       = 0
deposit_count   = 0      # incremented each time the lid opens
lid_open        = False  # True when IR detects lid is open
led_state       = False  # True when LED is on (motor actuation)
collected_flag  = False  # set True by Flask /bin/collected endpoint
last_material   = "---"  # most recent deposit label (shown on LCD)
last_confidence = 0.0
_lcd_lock       = threading.Lock()  # serialise I2C LCD writes across threads

# Last known sensor readings — used by interrupt callbacks for immediate LCD update
_last_fill_pct  = 0.0
_last_bin_temp  = None
_last_bin_hum   = None

# Colours for SenseHat LED matrix
GREEN = (0, 200, 0)
RED   = (200, 0, 0)
OFF   = (0, 0, 0)


# ─────────────────────────────────────────────────────────────────────────────
#  Sensor reading helpers
# ─────────────────────────────────────────────────────────────────────────────

def read_ultrasonic() -> float:
    """Return distance in cm from HC-SR04 (BCM 23/24)."""
    GPIO.output(config.PIN_TRIG, GPIO.HIGH)
    time.sleep(0.00001)          # 10 µs pulse
    GPIO.output(config.PIN_TRIG, GPIO.LOW)

    timeout = time.time() + 0.05  # 50 ms safety timeout
    while GPIO.input(config.PIN_ECHO) == GPIO.LOW:
        if time.time() > timeout:
            log.warning("HC-SR04: echo LOW timeout")
            return -1.0
    pulse_start = time.time()

    timeout = time.time() + 0.05
    while GPIO.input(config.PIN_ECHO) == GPIO.HIGH:
        if time.time() > timeout:
            log.warning("HC-SR04: echo HIGH timeout")
            return -1.0
    pulse_end = time.time()

    distance_cm = (pulse_end - pulse_start) * 17150  # speed of sound / 2
    distance_cm = round(distance_cm, 2)
    return distance_cm


def compute_fill(distance_cm: float) -> float:
    """Convert raw distance to fill percentage (0–100)."""
    if distance_cm <= 0:
        return 0.0
    fill = ((config.BIN_DEPTH_CM - distance_cm) / config.BIN_DEPTH_CM) * 100
    return max(0.0, min(100.0, round(fill, 1)))


def read_dht22() -> tuple[float, float]:
    """Return (temperature_C, humidity_%) from DHT22 on GPIO4."""
    for _ in range(3):  # retry up to 3 times
        try:
            temp = dht_sensor.temperature
            hum  = dht_sensor.humidity
            if temp is not None and hum is not None:
                return round(temp, 1), round(hum, 1)
        except RuntimeError as e:
            log.debug("DHT22 read error (will retry): %s", e)
            time.sleep(0.5)
    log.warning("DHT22: failed after 3 attempts")
    return None, None


def read_sensehat() -> dict:
    """Return ambient office readings from SenseHat."""
    return {
        "office_temp":     round(sense.get_temperature(), 1),
        "office_humidity": round(sense.get_humidity(), 1),
        "office_pressure": round(sense.get_pressure(), 1),
    }


# ─────────────────────────────────────────────────────────────────────────────
#  Actuator helpers
# ─────────────────────────────────────────────────────────────────────────────

def update_led_matrix(fill_pct: float):
    """Set SenseHat 8×8 LED matrix green or red depending on fill."""
    colour = RED if fill_pct >= config.FILL_ALERT_THRESHOLD else GREEN
    sense.clear(colour)


def update_lcd(fill_pct: float, bin_temp, bin_humidity):
    """Cache latest sensor readings and refresh the LCD display."""
    global _last_fill_pct, _last_bin_temp, _last_bin_hum
    with state_lock:
        _last_fill_pct = fill_pct
        _last_bin_temp = bin_temp
        _last_bin_hum  = bin_humidity
    _refresh_lcd()


def _refresh_lcd():
    """Render current state to the LCD. Safe to call from any thread."""
    with state_lock:
        is_open      = lid_open
        fill_pct     = _last_fill_pct
        bin_temp     = _last_bin_temp
        bin_humidity = _last_bin_hum

    if is_open:
        line1 = "Lid Open        "
        line2 = "                "
    else:
        t_str = f"{bin_temp:.1f}C" if bin_temp is not None else "---C"
        h_str = f"{bin_humidity:.0f}%" if bin_humidity is not None else "---%"
        line1 = f"Fill:{fill_pct:4.0f}% {t_str}"[:16].ljust(16)
        line2 = f"Hum: {h_str}"[:16].ljust(16)

    with _lcd_lock:
        _lcd_command(0x01)
        time.sleep(0.01)
        _lcd_set_cursor(0, 0)
        _lcd_write(line1)
        _lcd_set_cursor(1, 0)
        _lcd_write(line2)


# ─────────────────────────────────────────────────────────────────────────────
#  GPIO interrupt callbacks
# ─────────────────────────────────────────────────────────────────────────────

def pir_callback(channel):
    """PIR fires when someone waves their hand — turn on LED (motor open signal)."""
    global pir_count, led_state
    with state_lock:
        pir_count += 1
        led_state = True
    GPIO.output(config.PIN_LED, GPIO.HIGH)
    log.info("PIR: motion detected → LED on (count=%d)", pir_count)


def ir_callback(channel):
    """Fires on both edges of the IR sensor to track lid state."""
    pin_state = GPIO.input(config.PIN_IR)
    if pin_state == GPIO.HIGH:
        _on_lid_open()
    else:
        _on_lid_close()


def _on_lid_open():
    """Called when IR goes HIGH (active-low: HIGH = lid open)."""
    global deposit_count, lid_open
    with state_lock:
        deposit_count += 1
        lid_open      = True
        lid_state_val = deposit_count
    log.info("Lid OPENED → deposit #%d", lid_state_val)
    _refresh_lcd()


def _on_lid_close():
    """Called when IR goes LOW (active-low: LOW = lid closed)."""
    global lid_open, led_state
    with state_lock:
        lid_open  = False
        led_state = False
    GPIO.output(config.PIN_LED, GPIO.LOW)
    log.info("Lid CLOSED → LED off")
    _refresh_lcd()


def _poll_interrupt_pins():
    """Poll PIR and IR pins in a thread — same approach as test_sensors.py."""
    pir_last = GPIO.input(config.PIN_PIR)
    ir_last  = GPIO.input(config.PIN_IR)
    while True:
        pir_now = GPIO.input(config.PIN_PIR)
        ir_now  = GPIO.input(config.PIN_IR)
        # PIR: trigger on RISING (LOW → HIGH)
        if pir_now != pir_last:
            if pir_now == GPIO.HIGH:
                pir_callback(config.PIN_PIR)
            pir_last = pir_now
        # IR: trigger on any change (BOTH edges)
        if ir_now != ir_last:
            ir_callback(config.PIN_IR)
            ir_last = ir_now
        time.sleep(0.1)

_poll_thread = threading.Thread(target=_poll_interrupt_pins, daemon=True)
_poll_thread.start()


# ─────────────────────────────────────────────────────────────────────────────
#  Shared state persistence (sensor_node → Flask)
# ─────────────────────────────────────────────────────────────────────────────

def save_state(state: dict):
    """Atomically write state dict to STATE_FILE for Flask to read."""
    tmp = config.STATE_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(state, f)
    os.replace(tmp, config.STATE_FILE)


def load_state() -> dict:
    """Read state from STATE_FILE (used on startup to restore counts after restart)."""
    try:
        with open(config.STATE_FILE) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


# ─────────────────────────────────────────────────────────────────────────────
#  Main loop
# ─────────────────────────────────────────────────────────────────────────────

LOOP_INTERVAL = 2  # seconds between sensor reads / state updates

def main():
    global pir_count, deposit_count, collected_flag

    # Restore counters across restarts
    saved = load_state()
    with state_lock:
        pir_count     = saved.get("pir_count",     0)
        deposit_count = saved.get("deposit_count", 0)

    log.info("Waste management sensor node started (local mode).")

    try:
        while True:
            loop_start = time.time()
            try:
                # ── Read sensors ─────────────────────────────────────────────────
                distance_cm = read_ultrasonic()
                fill_pct    = compute_fill(distance_cm)

                bin_temp, bin_hum = read_dht22()
                office = read_sensehat()

                # Check both in-memory flag and file-based flag written by Flask
                file_collected = False
                try:
                    with open(config.STATE_FILE) as _sf:
                        _fs = json.load(_sf)
                        file_collected = bool(_fs.get("collected_flag", False))
                except (FileNotFoundError, json.JSONDecodeError):
                    pass

                with state_lock:
                    snap_pir      = pir_count
                    snap_deposit  = deposit_count
                    snap_lid      = lid_open
                    snap_led      = led_state
                    was_collected = collected_flag or file_collected
                    if was_collected:
                        fill_pct       = 0.0
                        pir_count      = 0
                        deposit_count  = 0
                        snap_pir       = 0
                        snap_deposit   = 0
                        collected_flag = False
                        log.info("Bin marked as collected — counters reset.")

                # ── Actuators ────────────────────────────────────────────────────
                update_led_matrix(fill_pct)
                update_lcd(fill_pct, bin_temp, bin_hum)

                # ── Persist state for Flask ───────────────────────────────────────
                now_str = datetime.utcnow().isoformat() + "Z"
                with state_lock:
                    snap_material   = last_material
                    snap_confidence = last_confidence

                current_state = {
                    "timestamp":       now_str,
                    "fill_pct":        fill_pct,
                    "distance_cm":     distance_cm,
                    "bin_temp":        bin_temp,
                    "bin_humidity":    bin_hum,
                    "office_temp":     office["office_temp"],
                    "office_humidity": office["office_humidity"],
                    "office_pressure": office["office_pressure"],
                    "pir_count":       snap_pir,
                    "deposit_count":   snap_deposit,
                    "lid_open":        snap_lid,
                    "led_state":       snap_led,
                    "alert":           fill_pct >= config.FILL_ALERT_THRESHOLD,
                    "last_material":   snap_material,
                    "last_confidence": snap_confidence,
                }
                save_state(current_state)

                elapsed    = time.time() - loop_start
                sleep_time = max(0, LOOP_INTERVAL - elapsed)
                log.info(
                    "fill=%.1f%% dist=%.1fcm bin_T=%s°C office_T=%.1f°C "
                    "PIR=%d deposits=%d lid=%s led=%s  next_in=%.1fs",
                    fill_pct, distance_cm, bin_temp,
                    office["office_temp"], snap_pir, snap_deposit,
                    "open" if snap_lid else "closed",
                    "on"   if snap_led  else "off",
                    sleep_time,
                )
                time.sleep(sleep_time)

            except Exception as _loop_err:
                log.error("Main loop error (will retry): %s", _loop_err, exc_info=True)
                elapsed    = time.time() - loop_start
                sleep_time = max(0, LOOP_INTERVAL - elapsed)
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        log.info("Sensor node stopped by user.")
    finally:
        dht_sensor.exit()
        GPIO.output(config.PIN_LED, GPIO.LOW)
        _lcd_command(0x01)
        sense.clear()
        GPIO.cleanup()
        log.info("GPIO and peripherals cleaned up.")


if __name__ == "__main__":
    main()
