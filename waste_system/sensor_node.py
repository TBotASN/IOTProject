#!/usr/bin/env python3
"""
sensor_node.py — Smart Office Waste Management System
Raspberry Pi 4B sensor loop with MQTT + HTTP fallback publishing.

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
import ssl
import threading
import os
import sys
import logging
from datetime import datetime
from collections import deque
import http.client
import urllib.parse

import numpy as np
import cv2
import RPi.GPIO as GPIO
import adafruit_dht
import board
from sense_hat import SenseHat
import smbus
from picamera2 import Picamera2
import paho.mqtt.client as mqtt

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    try:
        import tensorflow.lite as tflite
    except ImportError:
        tflite = None

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
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(config.PIN_TRIG, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(config.PIN_ECHO, GPIO.IN)
GPIO.setup(config.PIN_PIR,  GPIO.IN)
GPIO.setup(config.PIN_IR,   GPIO.IN)
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

camera = Picamera2()
cam_config = camera.create_still_configuration(
    main={"size": (1280, 720)},
    lores={"size": (640, 360)},
    display="lores",
)
camera.configure(cam_config)
camera.start()
time.sleep(2)  # warm-up

# ── TFLite inference (optional — skipped if model files are missing) ───────────
_interp      = None
_inp         = None
_outp        = None
_class_names = []

def _load_inference_model():
    """Load TFLite model and labels. Returns True on success, False if files are missing."""
    global _interp, _inp, _outp, _class_names
    if tflite is None:
        log.warning("TFLite not available — install tflite-runtime or tensorflow")
        return False
    try:
        with open(config.LABELS_PATH) as f:
            _class_names = [line.strip() for line in f]
        _interp = tflite.Interpreter(model_path=config.TFLITE_PATH)
        _interp.allocate_tensors()
        _inp  = _interp.get_input_details()
        _outp = _interp.get_output_details()
        log.info("TFLite model loaded — classes: %s", _class_names)
        return True
    except FileNotFoundError as exc:
        log.warning("TFLite model/labels not found (%s) — classification disabled", exc)
        return False
    except Exception as exc:
        log.error("TFLite load error: %s — classification disabled", exc)
        return False

_inference_available = _load_inference_model()


def _classify_frame(frame_rgb: np.ndarray) -> np.ndarray:
    """Run a single RGB frame through the model; return raw score array."""
    img = cv2.resize(frame_rgb, (config.IMG_SIZE, config.IMG_SIZE))
    img = np.expand_dims(img.astype(np.float32) / 255.0, axis=0)
    _interp.set_tensor(_inp[0]['index'], img)
    _interp.invoke()
    return _interp.get_tensor(_outp[0]['index'])[0]


def _classify_deposit() -> tuple[str | None, float]:
    """
    Capture CAPTURE_FRAMES frames from the lores stream, average scores
    (temporal smoothing), and return (label, confidence).
    """
    score_buf = deque(maxlen=config.SMOOTH_WINDOW)
    for _ in range(config.CAPTURE_FRAMES):
        frame = camera.capture_array("lores")
        # picamera2 lores delivers YUV420 by default; main stream gives RGB
        # Use main stream for a reliable RGB array
        frame = camera.capture_array("main")
        scores = _classify_frame(frame[:, :, :3])  # drop alpha if present
        score_buf.append(scores)
        time.sleep(0.05)
    if not score_buf:
        return None, 0.0
    smoothed   = np.mean(score_buf, axis=0)
    idx        = int(np.argmax(smoothed))
    label      = _class_names[idx]
    confidence = float(smoothed[idx])
    return label, confidence


# ── Local MQTT (material classification) ──────────────────────────────────────
_local_mqtt: mqtt.Client | None = None
_local_mqtt_connected           = False


def _init_local_mqtt() -> mqtt.Client | None:
    """Connect to local MQTT broker for material classification publishing."""
    def _on_connect(client, userdata, flags, rc):
        global _local_mqtt_connected
        _local_mqtt_connected = (rc == 0)
        if rc == 0:
            log.info("Local MQTT: connected to %s:%d", config.LOCAL_MQTT_BROKER, config.LOCAL_MQTT_PORT)
        else:
            log.warning("Local MQTT: connection failed (rc=%d)", rc)

    def _on_disconnect(client, userdata, rc):
        global _local_mqtt_connected
        _local_mqtt_connected = False

    client = mqtt.Client()
    client.on_connect    = _on_connect
    client.on_disconnect = _on_disconnect
    try:
        client.connect(config.LOCAL_MQTT_BROKER, config.LOCAL_MQTT_PORT, keepalive=60)
        client.loop_start()
        return client
    except Exception as exc:
        log.warning("Local MQTT: could not connect (%s) — material publishing disabled", exc)
        return None

_local_mqtt = _init_local_mqtt()

# ── Shared state ──────────────────────────────────────────────────────────────
state_lock        = threading.Lock()
pir_count         = 0
deposit_count     = 0      # incremented each time the lid opens
lid_open          = False  # True when IR detects lid is open
collected_flag    = False  # set True by Flask /bin/collected endpoint
last_material     = "---"  # most recent classification label (shown on LCD)
last_confidence   = 0.0    # confidence of last classification
_inference_lock   = threading.Lock()  # prevent overlapping classify_deposit calls
_lcd_lock         = threading.Lock()  # serialise I2C LCD writes across threads

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
        "office_temp": round(sense.get_temperature(), 1),
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
        material     = last_material
        fill_pct     = _last_fill_pct
        bin_temp     = _last_bin_temp
        bin_humidity = _last_bin_hum

    if is_open:
        line1 = "Item:           "
        line2 = (material or "Detecting...")[:16].ljust(16)
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


def capture_image() -> str:
    """Capture a still image and return the saved path."""
    os.makedirs(os.path.dirname(config.LATEST_IMAGE_PATH), exist_ok=True)
    path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        config.LATEST_IMAGE_PATH,
    )
    camera.capture_file(path)
    log.info("Camera: image saved → %s", path)
    return path


# ─────────────────────────────────────────────────────────────────────────────
#  GPIO interrupt callbacks
# ─────────────────────────────────────────────────────────────────────────────

def pir_callback(channel):
    """PIR fires when someone waves their hand — turn on LED (motor open signal)."""
    global pir_count
    with state_lock:
        pir_count += 1
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
    global deposit_count, last_material
    with state_lock:
        deposit_count += 1
        last_material = "Detecting..."
        lid_state_val = deposit_count
    log.info("Lid OPENED → deposit #%d — starting classification", lid_state_val)
    _refresh_lcd()
    threading.Thread(target=_handle_lid_open, daemon=True).start()


def _on_lid_close():
    """Called when IR goes LOW (active-low: LOW = lid closed)."""
    global lid_open
    with state_lock:
        lid_open = False
    GPIO.output(config.PIN_LED, GPIO.LOW)
    log.info("Lid CLOSED → LED off")
    _refresh_lcd()


def _handle_lid_open():
    """Capture image and run TFLite classification while lid is open (daemon thread)."""
    global last_material, last_confidence, lid_open

    # Mark lid as open only inside this thread to avoid race with ir_callback
    with state_lock:
        lid_open = True

    capture_image()  # save still for Flask dashboard

    if not _inference_available:
        with state_lock:
            last_material = "unknown"
        return

    if not _inference_lock.acquire(blocking=False):
        log.debug("Inference already running — skipping this deposit")
        return

    try:
        label, confidence = _classify_deposit()
        if label is None:
            log.warning("Inference: no frames captured")
            with state_lock:
                last_material = "unknown"
            return

        log.info("Inference: %s  (%.0f%%)", label, confidence * 100)

        with state_lock:
            last_material   = label
            last_confidence = confidence
        _refresh_lcd()  # show result on LCD immediately

        if confidence >= config.CONFIDENCE_THRESHOLD and _local_mqtt and _local_mqtt_connected:
            _local_mqtt.publish(config.MQTT_TOPIC_MATERIAL,   label)
            _local_mqtt.publish(config.MQTT_TOPIC_CONFIDENCE, f"{confidence:.2f}")
            log.info("Local MQTT: published material=%s confidence=%.2f", label, confidence)
        elif confidence < config.CONFIDENCE_THRESHOLD:
            log.info("Inference: below threshold (%.0f%%) — not published", config.CONFIDENCE_THRESHOLD * 100)
    finally:
        _inference_lock.release()


GPIO.add_event_detect(config.PIN_PIR, GPIO.RISING, callback=pir_callback, bouncetime=500)
GPIO.add_event_detect(config.PIN_IR,  GPIO.BOTH,   callback=ir_callback,  bouncetime=200)


# ─────────────────────────────────────────────────────────────────────────────
#  MQTT (primary — Lab 5 pattern)
# ─────────────────────────────────────────────────────────────────────────────

mqtt_connected = False
mqtt_client    = None


def _on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        mqtt_connected = True
        log.info("MQTT: connected to %s:%d", config.MQTT_BROKER, config.MQTT_PORT)
    else:
        mqtt_connected = False
        log.warning("MQTT: connection failed (rc=%d)", rc)


def _on_disconnect(client, userdata, rc):
    global mqtt_connected
    mqtt_connected = False
    log.warning("MQTT: disconnected (rc=%d)", rc)


def init_mqtt() -> mqtt.Client:
    """Initialise and connect the MQTT client with SSL WebSocket transport."""
    client = mqtt.Client(
        client_id=config.MQTT_CLIENT_ID,
        transport="websockets",
        protocol=mqtt.MQTTv311,
    )
    client.username_pw_set(config.MQTT_USERNAME, config.MQTT_PASSWORD)
    ssl_ctx = ssl.create_default_context()
    client.tls_set_context(ssl_ctx)
    client.on_connect    = _on_connect
    client.on_disconnect = _on_disconnect
    client.ws_set_options(path="/mqtt")  # ThingSpeak WS path

    try:
        client.connect(config.MQTT_BROKER, config.MQTT_PORT, keepalive=60)
        client.loop_start()
    except Exception as exc:
        log.error("MQTT: initial connect failed: %s", exc)
    return client


def publish_mqtt(payload: dict) -> bool:
    """Publish a field payload dict to ThingSpeak via MQTT. Returns True on success."""
    global mqtt_connected, mqtt_client
    if not mqtt_connected:
        return False
    field_str = "&".join(f"field{k}={v}" for k, v in payload.items())
    result = mqtt_client.publish(config.MQTT_TOPIC, field_str, qos=0)
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        log.info("MQTT: published → %s", field_str)
        return True
    log.warning("MQTT: publish failed (rc=%d)", result.rc)
    return False


# ─────────────────────────────────────────────────────────────────────────────
#  HTTP fallback (Lab 4 pattern — urllib / http.client)
# ─────────────────────────────────────────────────────────────────────────────

def publish_http(payload: dict) -> bool:
    """POST sensor data to ThingSpeak REST API. Returns True on success."""
    params = {"api_key": config.THINGSPEAK_WRITE_KEY}
    params.update({f"field{k}": v for k, v in payload.items()})
    body = urllib.parse.urlencode(params)
    headers = {
        "Content-Type": "application/x-www-form-urlencoded",
        "Content-Length": str(len(body)),
    }
    try:
        conn = http.client.HTTPSConnection(config.THINGSPEAK_HTTP_HOST, timeout=10)
        conn.request("POST", config.THINGSPEAK_HTTP_PATH, body, headers)
        resp = conn.getresponse()
        data = resp.read().decode()
        conn.close()
        if resp.status == 200 and data != "0":
            log.info("HTTP fallback: posted entry_id=%s", data.strip())
            return True
        log.warning("HTTP fallback: server returned status=%d body=%s", resp.status, data)
        return False
    except Exception as exc:
        log.error("HTTP fallback: exception: %s", exc)
        return False


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

def main():
    global pir_count, deposit_count, collected_flag, mqtt_client

    # Restore counters across restarts
    saved = load_state()
    with state_lock:
        pir_count     = saved.get("pir_count",     0)
        deposit_count = saved.get("deposit_count", 0)

    mqtt_client = init_mqtt()
    time.sleep(2)  # allow MQTT connection to establish

    log.info("Waste management sensor node started.")

    try:
        while True:
            loop_start = time.time()

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
                snap_pir     = pir_count
                snap_deposit = deposit_count
                snap_lid     = lid_open
                was_collected = collected_flag or file_collected
                if was_collected:
                    # Reset fill representation: pretend bin is empty
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
                snap_material    = last_material
                snap_confidence  = last_confidence
            current_state = {
                "timestamp":        now_str,
                "fill_pct":         fill_pct,
                "distance_cm":      distance_cm,
                "bin_temp":         bin_temp,
                "bin_humidity":     bin_hum,
                "office_temp":      office["office_temp"],
                "office_humidity":  office["office_humidity"],
                "office_pressure":  office["office_pressure"],
                "pir_count":        snap_pir,
                "deposit_count":    snap_deposit,
                "lid_open":         snap_lid,
                "alert":            fill_pct >= config.FILL_ALERT_THRESHOLD,
                "last_material":    snap_material,
                "last_confidence":  snap_confidence,
            }
            save_state(current_state)

            # ── Build ThingSpeak payload ──────────────────────────────────────
            ts_payload = {
                1: fill_pct,
                2: bin_temp  if bin_temp  is not None else "",
                3: bin_hum   if bin_hum   is not None else "",
                4: office["office_temp"],
                5: office["office_humidity"],
                6: snap_pir,
                7: 1 if snap_lid else 0,
                8: snap_deposit,
            }

            # ── Publish: MQTT primary, HTTP fallback ──────────────────────────
            if not publish_mqtt(ts_payload):
                log.info("MQTT unavailable — using HTTP fallback.")
                publish_http(ts_payload)

            # ── Adaptive sampling interval ────────────────────────────────────
            interval = (
                config.SAMPLE_INTERVAL_FAST
                if fill_pct >= config.FILL_FAST_SAMPLE_THR
                else config.SAMPLE_INTERVAL_NORMAL
            )
            elapsed = time.time() - loop_start
            sleep_time = max(0, interval - elapsed)
            log.info(
                "fill=%.1f%% dist=%.1fcm bin_T=%s°C office_T=%.1f°C "
                "PIR=%d deposits=%d lid=%s  next_in=%.0fs",
                fill_pct, distance_cm, bin_temp,
                office["office_temp"], snap_pir, snap_deposit,
                "open" if snap_lid else "closed", sleep_time,
            )
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        log.info("Sensor node stopped by user.")
    finally:
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        if _local_mqtt:
            _local_mqtt.loop_stop()
            _local_mqtt.disconnect()
        camera.stop()
        dht_sensor.exit()
        GPIO.output(config.PIN_LED, GPIO.LOW)
        _lcd_command(0x01)
        sense.clear()
        GPIO.cleanup()
        log.info("GPIO and peripherals cleaned up.")


if __name__ == "__main__":
    main()
