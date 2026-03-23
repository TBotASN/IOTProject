# config.py — Sensitive credentials and system constants
# Replace placeholder values with your actual ThingSpeak credentials.

# ── ThingSpeak channel ────────────────────────────────────────────────────────
THINGSPEAK_CHANNEL_ID  = "YOUR_CHANNEL_ID"          # numeric channel ID (string)
THINGSPEAK_WRITE_KEY   = "YOUR_WRITE_API_KEY"        # 16-char write key

# ── MQTT (ThingSpeak MQTT broker, SSL WebSockets on port 443) ─────────────────
MQTT_BROKER    = "mqtt3.thingspeak.com"
MQTT_PORT      = 443
MQTT_TOPIC     = f"channels/{THINGSPEAK_CHANNEL_ID}/publish"
MQTT_CLIENT_ID = "YOUR_MQTT_CLIENT_ID"              # from ThingSpeak Devices
MQTT_USERNAME  = "YOUR_MQTT_USERNAME"               # from ThingSpeak Devices
MQTT_PASSWORD  = "YOUR_MQTT_PASSWORD"               # from ThingSpeak Devices

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
PIN_TRIG = 23   # HC-SR04 trigger
PIN_ECHO = 24   # HC-SR04 echo (via voltage divider — 5 V → 3.3 V)
PIN_PIR  = 17   # PIR motion sensor
PIN_IR   = 22   # IR proximity sensor (deposit detection)
PIN_DHT  = 4    # DHT22 data pin (inside bin)

# ── Flask ─────────────────────────────────────────────────────────────────────
FLASK_PORT  = 5000
FLASK_DEBUG = False

# ── Shared state file (sensor_node → Flask) ───────────────────────────────────
STATE_FILE        = "/tmp/waste_state.json"
LATEST_IMAGE_PATH = "static/images/latest_deposit.jpg"
