#!/usr/bin/env python3
"""
app.py — Smart Office Waste Management System — Flask Dashboard (Local Mode)
Runs concurrently with sensor_node.py on port 5000.

Routes:
  GET  /             — colour-coded dashboard (Jinja2 template)
  GET  /api/status   — JSON snapshot of current sensor state (polled every 3s)
  POST /bin/collected — mark bin as collected (resets fill state)
"""

import json
import os
import sys
import subprocess
import atexit
import threading
from datetime import datetime, timezone

from flask import Flask, jsonify, render_template, redirect, url_for

import config

# ─────────────────────────────────────────────────────────────────────────────
#  Camera inference globals
# ─────────────────────────────────────────────────────────────────────────────

_cam_lock        = threading.Lock()
_cam_material    = "---"
_cam_confidence  = 0.0


def _camera_thread():
    """Background thread: runs Picamera2 + TFLite inference continuously."""
    global _cam_material, _cam_confidence
    try:
        import cv2
        import numpy as np
        from picamera2 import Picamera2
    except ImportError as e:
        print(f"[camera] Missing dependency: {e} — material detection disabled")
        return

    # Load TFLite model (optional — falls back to no inference)
    interp = inp_d = outp_d = class_names = None
    try:
        try:
            import tflite_runtime.interpreter as tflite
        except ImportError:
            import tensorflow.lite as tflite  # type: ignore
        with open(config.LABELS_PATH) as f:
            class_names = [line.strip() for line in f]
        interp = tflite.Interpreter(model_path=config.TFLITE_PATH)
        interp.allocate_tensors()
        inp_d  = interp.get_input_details()
        outp_d = interp.get_output_details()
        print(f"[camera] TFLite model loaded — classes: {class_names}")
    except FileNotFoundError:
        print(f"[camera] Model not found at {config.TFLITE_PATH} — running without inference")
    except Exception as exc:
        print(f"[camera] TFLite load failed ({exc}) — running without inference")

    try:
        cam = Picamera2()
        cfg = cam.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},
        )
        cam.configure(cfg)
        cam.start()
        import time as _time
        _time.sleep(1)
        print("[camera] Camera started (640×480 RGB)")
    except Exception as exc:
        print(f"[camera] Camera failed to open: {exc}")
        return

    import time as _time
    try:
        while True:
            frame = cam.capture_array()   # RGB888 numpy array

            if interp is not None:
                img = cv2.resize(frame, (config.IMG_SIZE, config.IMG_SIZE))
                arr = np.expand_dims(img.astype(np.float32) / 255.0, axis=0)
                interp.set_tensor(inp_d[0]['index'], arr)
                interp.invoke()
                scores = interp.get_tensor(outp_d[0]['index'])[0]
                idx    = int(np.argmax(scores))
                conf   = float(scores[idx])
                label  = class_names[idx] if conf >= config.CONFIDENCE_THRESHOLD else "uncertain"
                with _cam_lock:
                    _cam_material   = label
                    _cam_confidence = conf

            _time.sleep(0.1)   # ~10 fps inference rate

    except Exception as exc:
        print(f"[camera] Thread error: {exc}")
    finally:
        cam.stop()
        cam.close()

app = Flask(__name__)

# ─────────────────────────────────────────────────────────────────────────────
#  Launch sensor_node.py as a background subprocess
# ─────────────────────────────────────────────────────────────────────────────

_sensor_proc = None

def _start_sensor_node():
    global _sensor_proc
    sensor_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_node.py")
    _sensor_proc = subprocess.Popen(
        [sys.executable, sensor_script],
        cwd=os.path.dirname(os.path.abspath(__file__)),
    )
    print(f"[app] sensor_node.py started (pid={_sensor_proc.pid})")

def _stop_sensor_node():
    if _sensor_proc and _sensor_proc.poll() is None:
        _sensor_proc.terminate()
        try:
            _sensor_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            _sensor_proc.kill()
        print("[app] sensor_node.py stopped.")

atexit.register(_stop_sensor_node)
_start_sensor_node()

_cam_thread = threading.Thread(target=_camera_thread, daemon=True)
_cam_thread.start()

# ─────────────────────────────────────────────────────────────────────────────
#  State helpers
# ─────────────────────────────────────────────────────────────────────────────

def _read_state() -> dict:
    """Read latest sensor state written by sensor_node.py."""
    try:
        with open(config.STATE_FILE) as f:
            state = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        state = {}

    defaults = {
        "timestamp":       "—",
        "fill_pct":        0.0,
        "distance_cm":     config.BIN_DEPTH_CM,
        "bin_temp":        None,
        "bin_humidity":    None,
        "office_temp":     None,
        "office_humidity": None,
        "office_pressure": None,
        "pir_count":       0,
        "deposit_count":   0,
        "lid_open":        False,
        "led_state":       False,
        "alert":           False,
        "last_material":   "---",
        "last_confidence": 0.0,
    }
    defaults.update(state)
    return defaults


def _signal_collected():
    """
    Write a 'collected' flag into the state file so sensor_node.py resets.
    sensor_node.py checks `collected_flag` key on every loop iteration.
    """
    try:
        with open(config.STATE_FILE) as f:
            state = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        state = {}
    state["collected_flag"] = True
    tmp = config.STATE_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(state, f)
    os.replace(tmp, config.STATE_FILE)


def _fill_colour(fill_pct: float) -> str:
    """Return a Bootstrap colour name for a given fill percentage."""
    if fill_pct >= 80:
        return "danger"
    if fill_pct >= 50:
        return "warning"
    return "success"


# ─────────────────────────────────────────────────────────────────────────────
#  Routes
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/")
def dashboard():
    state       = _read_state()
    fill_pct    = float(state["fill_pct"])
    fill_colour = _fill_colour(fill_pct)

    return render_template(
        "dashboard.html",
        state        = state,
        fill_pct     = fill_pct,
        fill_colour  = fill_colour,
        alert_thr    = config.FILL_ALERT_THRESHOLD,
        now          = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC"),
    )


@app.route("/api/status")
def api_status():
    """JSON endpoint — current sensor state, polled every 3 s by the dashboard."""
    state = _read_state()
    state["fill_colour"] = _fill_colour(float(state["fill_pct"]))
    with _cam_lock:
        state["last_material"]   = _cam_material
        state["last_confidence"] = _cam_confidence
    return jsonify(state)


@app.route("/bin/collected", methods=["POST"])
def bin_collected():
    """Mark the bin as collected — signals sensor_node.py to reset fill state."""
    _signal_collected()
    return redirect(url_for("dashboard"))


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=config.FLASK_PORT, debug=config.FLASK_DEBUG)
