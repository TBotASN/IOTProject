# Smart Office Waste Management System — Project Memory

## What This Project Does

A Raspberry Pi 4B IoT system that monitors a smart office waste bin and serves a live Flask web dashboard.

**Two processes run concurrently:**
- `app.py` — Flask server on port 5000, launches `sensor_node.py` as a subprocess on startup
- `sensor_node.py` — Sensor loop (every 2 seconds), writes live state to `/tmp/waste_state.json`

Flask reads that JSON file to serve the dashboard and API.

## How to Run

```bash
pkill -f sensor_node.py   # kill any leftover process first (important!)
python waste_system/app.py
```

Dashboard available at `http://10.113.171.243:5000` (or `http://127.0.0.1:5000`).

## Hardware (BCM Pin Numbering)

| Sensor / Actuator       | Pin (BCM) | Notes                                      |
|-------------------------|-----------|--------------------------------------------|
| HC-SR04 TRIG            | GPIO23    | Ultrasonic — bin fill level                |
| HC-SR04 ECHO            | GPIO24    | Via 5V→3.3V voltage divider (1kΩ + 2kΩ)  |
| PIR motion sensor       | GPIO17    | Hand-wave → LED on, simulates lid open     |
| IR proximity sensor     | GPIO22    | Active-low: HIGH=lid open, LOW=lid closed  |
| DHT22 temperature/hum   | GPIO4     | Inside bin — bin environment               |
| LED (motor indicator)   | GPIO27    | On when lid should open                    |
| SenseHat                | GPIO header (I2C/SPI) | Office ambient temp/hum/pressure |
| LCD 16×2 I2C            | GPIO2/3 (SDA/SCL), addr 0x3e | Shows fill % and bin temp |

## Key Files

| File                        | Purpose                                      |
|-----------------------------|----------------------------------------------|
| `waste_system/app.py`       | Flask dashboard, launches sensor_node subprocess |
| `waste_system/sensor_node.py` | Main sensor loop, GPIO, state file writer  |
| `waste_system/config.py`    | All pin numbers, thresholds, credentials     |
| `waste_system/test_sensors.py` | Standalone sensor test menu (always works) |
| `waste_system/pi_inference.py` | TFLite trash classification (optional)     |

## State File: `/tmp/waste_state.json`

Written every 2 seconds by `sensor_node.py`. Fields:
- `fill_pct` — bin fill percentage (0–100), based on HC-SR04 distance vs `BIN_DEPTH_CM=30`
- `distance_cm` — raw ultrasonic reading
- `bin_temp`, `bin_humidity` — DHT22 inside bin
- `office_temp`, `office_humidity`, `office_pressure` — SenseHat ambient
- `pir_count` — total hand-wave events
- `deposit_count` — total lid-open events
- `lid_open` — bool
- `led_state` — bool
- `alert` — bool (fill_pct >= 80%)
- `last_material`, `last_confidence` — TFLite classification result (if enabled)
- `collected_flag` — set by `POST /bin/collected` to reset counters

## Flask Routes

| Route | Method | Action |
|-------|--------|--------|
| `/` | GET | Jinja2 dashboard (colour-coded by fill level) |
| `/api/status` | GET | JSON state snapshot (polled every 3s by frontend) |
| `/bin/collected` | POST | Mark bin collected, resets fill/counters |

## Thresholds & Config (config.py)

- `BIN_DEPTH_CM = 30` — physical bin depth
- `FILL_ALERT_THRESHOLD = 80` — % at which SenseHat LED turns red, LCD shows ALERT
- ThingSpeak channel ID `3309226` — 8 fields mapped in config.py comments

## Known Fix: GPIO Edge Detection Failure

**Problem:** `RuntimeError: Failed to add edge detection` when running `app.py`.
**Root cause:** When `sensor_node.py` is killed without clean shutdown, the Linux kernel keeps GPIO17 and GPIO22 exported in `/sys/class/gpio/` with edge detection still active. `GPIO.cleanup()` in a new process doesn't clear another process's sysfs state. `GPIO.remove_event_detect()` also silently fails.

**Fix applied (in `sensor_node.py`):**
Replaced `GPIO.add_event_detect()` entirely with a polling thread — the same approach `test_sensors.py` uses and that always works:

```python
def _poll_interrupt_pins():
    pir_last = GPIO.input(config.PIN_PIR)
    ir_last  = GPIO.input(config.PIN_IR)
    while True:
        pir_now = GPIO.input(config.PIN_PIR)
        ir_now  = GPIO.input(config.PIN_IR)
        if pir_now != pir_last:
            if pir_now == GPIO.HIGH:
                pir_callback(config.PIN_PIR)
            pir_last = pir_now
        if ir_now != ir_last:
            ir_callback(config.PIN_IR)
            ir_last = ir_now
        time.sleep(0.1)

_poll_thread = threading.Thread(target=_poll_interrupt_pins, daemon=True)
_poll_thread.start()
```

Also: PIR and IR pins now set up with `pull_up_down=GPIO.PUD_DOWN` to prevent floating.

**Rule:** Never use `GPIO.add_event_detect()` in this project. Always poll in a daemon thread.
