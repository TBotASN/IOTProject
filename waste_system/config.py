# config.py — Sensitive credentials and system constants
# Replace placeholder values with your actual ThingSpeak credentials.

# ── ThingSpeak channel ────────────────────────────────────────────────────────
THINGSPEAK_CHANNEL_ID  = "3309226"
THINGSPEAK_WRITE_KEY   = "WAODEXRBBPBI6WAU"

# ── MQTT (ThingSpeak MQTT broker, SSL WebSockets on port 443) ─────────────────
MQTT_BROKER    = "mqtt3.thingspeak.com"
MQTT_PORT      = 443
MQTT_TOPIC     = f"channels/{THINGSPEAK_CHANNEL_ID}/publish"
MQTT_CLIENT_ID = "AisWOQcSLRoJAx4wNiIJNjU"
MQTT_USERNAME  = "AisWOQcSLRoJAx4wNiIJNjU"
MQTT_PASSWORD  = "4UtoS9ym+YQIxprt0yH2eXlV"

# ── HTTP fallback ─────────────────────────────────────────────────────────────
THINGSPEAK_HTTP_HOST = "api.thingspeak.com"
THINGSPEAK_HTTP_PATH = "/update"

# ── Hardware constants ────────────────────────────────────────────────────────
BIN_DEPTH_CM = 30          # physical depth of the bin in centimetres
FILL_ALERT_THRESHOLD  = 80 # % — LED turns red, LCD shows ALERT-COLLECT
FILL_FAST_SAMPLE_THR  = 70 # % — adaptive sampling kicks in
SAMPLE_INTERVAL_NORMAL = 60 # seconds
SAMPLE_INTERVAL_FAST   = 10 # seconds

# ── GPIO pins (BCM numbering) ─────────────────────────────────────────────────
PIN_TRIG    = 23   # HC-SR04 trigger
PIN_ECHO    = 24   # HC-SR04 echo (via voltage divider — 5 V → 3.3 V)
PIN_WAVE_IR = 17   # IR sensor (hand-wave → LED on, replaces PIR)
PIN_IR      = 22   # IR proximity sensor (active-low: LOW=lid closed, HIGH=lid open)
PIN_DHT     = 4    # DHT22 data pin (inside bin)
PIN_LED     = 27   # Motor indicator LED

# ── Flask ─────────────────────────────────────────────────────────────────────
FLASK_PORT  = 5000
FLASK_DEBUG = False

# ── Shared state file (sensor_node → Flask) ───────────────────────────────────
STATE_FILE        = "/tmp/waste_state.json"
LATEST_IMAGE_PATH = "static/images/latest_deposit.jpg"

# ── TFLite trash classifier ────────────────────────────────────────────────────
TFLITE_PATH          = "/home/group28/Documents/IOTProject/IOTProject/waste_system/trash_classifier.tflite"
LABELS_PATH          = "/home/group28/Documents/IOTProject/IOTProject/waste_system/labels.txt"
IMG_SIZE             = 224          # model input resolution (square)
CONFIDENCE_THRESHOLD = 0.6          # minimum confidence to publish
SMOOTH_WINDOW        = 8            # frames averaged per deposit event
CAPTURE_FRAMES       = 10           # frames captured per IR trigger

# ── Local MQTT broker (material classification results) ───────────────────────
LOCAL_MQTT_BROKER     = "localhost"
LOCAL_MQTT_PORT       = 1883
MQTT_TOPIC_MATERIAL   = "office/bin/material"
MQTT_TOPIC_CONFIDENCE = "office/bin/material/confidence"

# ── ThingSpeak field mapping (8 fields max on free tier) ─────────────────────
# Field 1 = fill_pct
# Field 2 = bin_temp      (DHT22 inside bin)
# Field 3 = bin_humidity  (DHT22 inside bin)
# Field 4 = office_temp   (SenseHat ambient)
# Field 5 = office_humidity (SenseHat ambient)
# Field 6 = pir_count     (motion / hand-wave events)
# Field 7 = lid_state     (0 = closed, 1 = open)
# Field 8 = deposit_count (total lid-open events)
