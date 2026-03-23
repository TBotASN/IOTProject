"""
Trash Classifier — Raspberry Pi 4B Inference Script
Triggered by PIR sensor, publishes material label via MQTT

Dependencies:
    pip install tflite-runtime opencv-python-headless pillow numpy paho-mqtt RPi.GPIO

Hardware:
    - Pi Camera or USB webcam on /dev/video0
    - PIR sensor on GPIO pin 17
"""

import time
import threading
import numpy as np
import cv2
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from collections import deque
from pathlib import Path

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite

# ── Config ────────────────────────────────────────────────────────────────────
TFLITE_PATH          = "/home/pi/model/trash_classifier.tflite"
LABELS_PATH          = "/home/pi/model/labels.txt"
PIR_PIN              = 17
IMG_SIZE             = 224
CONFIDENCE_THRESHOLD = 0.6    # only publish if confident enough
SMOOTH_WINDOW        = 8      # frames to average per deposit event
CAPTURE_FRAMES       = 10     # how many frames to capture per PIR trigger
MQTT_BROKER          = "localhost"  # Raspberry Pi is the MQTT broker
MQTT_PORT            = 1883
MQTT_TOPIC_MATERIAL  = "office/bin/material"
MQTT_TOPIC_CONFIDENCE= "office/bin/material/confidence"

# ── Load model ────────────────────────────────────────────────────────────────
print("→ Loading TFLite model...")
with open(LABELS_PATH) as f:
    class_names = [line.strip() for line in f.readlines()]

interp = tflite.Interpreter(model_path=TFLITE_PATH)
interp.allocate_tensors()
inp  = interp.get_input_details()
outp = interp.get_output_details()
print(f"✓ Model loaded — classes: {class_names}")

# ── MQTT ──────────────────────────────────────────────────────────────────────
client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT)
client.loop_start()
print(f"✓ MQTT connected to {MQTT_BROKER}:{MQTT_PORT}")

# ── GPIO ──────────────────────────────────────────────────────────────────────
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIR_PIN, GPIO.IN)
print(f"✓ PIR sensor on GPIO {PIR_PIN}")

# ── Camera ────────────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    raise RuntimeError("Could not open camera on /dev/video0")
print("✓ Camera open")

# ── Inference ─────────────────────────────────────────────────────────────────
def classify_frame(frame):
    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = np.expand_dims(img.astype(np.float32) / 255.0, axis=0)
    interp.set_tensor(inp[0]['index'], img)
    interp.invoke()
    return interp.get_tensor(outp[0]['index'])[0]

def classify_deposit():
    """
    Capture CAPTURE_FRAMES frames, average scores (smoothing),
    return the most confident label.
    """
    score_buffer = deque(maxlen=SMOOTH_WINDOW)

    for _ in range(CAPTURE_FRAMES):
        ret, frame = cap.read()
        if not ret:
            continue
        scores = classify_frame(frame)
        score_buffer.append(scores)
        time.sleep(0.05)  # ~20fps capture rate

    if not score_buffer:
        return None, 0.0

    smoothed   = np.mean(score_buffer, axis=0)
    idx        = np.argmax(smoothed)
    label      = class_names[idx]
    confidence = float(smoothed[idx])
    return label, confidence

# ── PIR callback ──────────────────────────────────────────────────────────────
inference_lock = threading.Lock()  # prevent overlapping inference runs

def on_pir_triggered(channel):
    if not inference_lock.acquire(blocking=False):
        return  # already processing a deposit, skip

    try:
        print("\n→ PIR triggered — classifying deposit...")
        label, confidence = classify_deposit()

        if label is None:
            print("  ✗ No frames captured")
            return

        print(f"  Prediction: {label}  ({confidence:.0%})")

        if confidence >= CONFIDENCE_THRESHOLD:
            client.publish(MQTT_TOPIC_MATERIAL,   label)
            client.publish(MQTT_TOPIC_CONFIDENCE, f"{confidence:.2f}")
            print(f"  ✓ Published → {MQTT_TOPIC_MATERIAL}: {label}")
        else:
            print(f"  ✗ Below confidence threshold ({CONFIDENCE_THRESHOLD:.0%}) — not published")

    finally:
        inference_lock.release()

# Register PIR interrupt — triggers on rising edge (motion detected)
GPIO.add_event_detect(PIR_PIN, GPIO.RISING,
                      callback=lambda ch: threading.Thread(
                          target=on_pir_triggered, args=(ch,), daemon=True
                      ).start(),
                      bouncetime=3000)  # 3s debounce — one trigger per deposit

# ── Main loop ─────────────────────────────────────────────────────────────────
print("\n✓ System ready — waiting for deposits...\n")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    cap.release()
    client.loop_stop()
    client.disconnect()
    GPIO.cleanup()
    print("Done.")
