#!/usr/bin/env python3
"""
app.py — Smart Office Waste Management System — Flask Dashboard (Local Mode)
Runs concurrently with sensor_node.py on port 5000.

Routes:
  GET  /              — colour-coded dashboard (Jinja2 template)
  GET  /api/status    — JSON snapshot of current sensor state (polled every 3s)
  GET  /api/health    — liveness check: sensor node alive, app uptime, PID
  POST /bin/collected — mark bin as collected (resets fill state)
"""

import json
import logging
import os
import sys
import subprocess
import atexit
import threading
import time
from datetime import datetime, timezone

from flask import Flask, jsonify, render_template, redirect, url_for

import config
import predictor

# ─────────────────────────────────────────────────────────────────────────────
#  Logging
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("/tmp/waste_app.log"),
    ],
)
log = logging.getLogger(__name__)

_app_start_time = time.time()

# ─────────────────────────────────────────────────────────────────────────────
#  Camera inference globals
# ─────────────────────────────────────────────────────────────────────────────

_cam_lock       = threading.Lock()
_cam_material   = "---"
_cam_confidence = 0.0


def _camera_thread():
    """Background thread: runs Picamera2 + TFLite inference continuously."""
    global _cam_material, _cam_confidence
    try:
        import cv2
        import numpy as np
        from picamera2 import Picamera2
    except ImportError as e:
        log.info("[camera] Missing dependency: %s — material detection disabled", e)
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
        log.info("[camera] TFLite model loaded — classes: %s", class_names)
    except FileNotFoundError:
        log.info("[camera] Model not found at %s — running without inference", config.TFLITE_PATH)
    except Exception as exc:
        log.warning("[camera] TFLite load failed (%s) — running without inference", exc)

    try:
        cam = Picamera2()
        cfg = cam.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},
        )
        cam.configure(cfg)
        cam.start()
        time.sleep(1)
        log.info("[camera] Camera started (640×480 RGB)")
    except Exception as exc:
        log.warning("[camera] Camera failed to open: %s", exc)
        return

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

            time.sleep(0.1)   # ~10 fps inference rate

    except Exception as exc:
        log.error("[camera] Thread error: %s", exc)
    finally:
        cam.stop()
        cam.close()


app = Flask(__name__)

# ─────────────────────────────────────────────────────────────────────────────
#  Launch sensor_node.py as a background subprocess + watchdog
# ─────────────────────────────────────────────────────────────────────────────

_sensor_proc      = None
_sensor_proc_lock = threading.Lock()


def _start_sensor_node():
    """(Re)start sensor_node.py as a subprocess. Thread-safe."""
    global _sensor_proc
    sensor_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor_node.py")
    with _sensor_proc_lock:
        _sensor_proc = subprocess.Popen(
            [sys.executable, sensor_script],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            stdout=sys.stdout,
            stderr=sys.stderr,
        )
    log.info("[app] sensor_node.py started (pid=%d)", _sensor_proc.pid)


def _stop_sensor_node():
    """Gracefully stop sensor_node.py (called on app exit)."""
    with _sensor_proc_lock:
        proc = _sensor_proc
    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.info("[app] sensor_node.py stopped.")


def _sensor_watchdog():
    """Background thread: restart sensor_node.py if it dies unexpectedly."""
    while True:
        time.sleep(10)
        with _sensor_proc_lock:
            proc = _sensor_proc
        if proc and proc.poll() is not None:
            exit_code = proc.returncode
            log.warning("[watchdog] sensor_node.py exited (code=%d) — restarting…", exit_code)
            _start_sensor_node()


atexit.register(_stop_sensor_node)
_start_sensor_node()

_watchdog_thread = threading.Thread(target=_sensor_watchdog, daemon=True, name="sensor-watchdog")
_watchdog_thread.start()

_cam_thread = threading.Thread(target=_camera_thread, daemon=True, name="camera-inference")
_cam_thread.start()

log.info(
    "[app] Flask dashboard starting on port %d | state_file=%s",
    config.FLASK_PORT, config.STATE_FILE,
)

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
        "started_at":      "—",
        "uptime_seconds":  0,
        "loop_count":      0,
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
    state["prediction"] = predictor.get_prediction()
    return jsonify(state)


@app.route("/api/health")
def api_health():
    """Liveness endpoint — reports whether sensor_node is running and app uptime."""
    with _sensor_proc_lock:
        proc = _sensor_proc
    alive    = bool(proc and proc.poll() is None)
    pid      = proc.pid if proc else None
    uptime   = round(time.time() - _app_start_time)
    return jsonify({
        "sensor_node_alive": alive,
        "sensor_node_pid":   pid,
        "app_uptime_seconds": uptime,
        "timestamp":          datetime.now(timezone.utc).isoformat(),
    })


@app.route("/bin/collected", methods=["POST"])
def bin_collected():
    """Mark the bin as collected — signals sensor_node.py to reset fill state."""
    _signal_collected()
    log.info("[app] Bin marked as collected via dashboard.")
    return redirect(url_for("dashboard"))


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=config.FLASK_PORT,
        debug=config.FLASK_DEBUG,
        threaded=True,
    )
