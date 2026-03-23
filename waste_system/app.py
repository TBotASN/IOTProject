#!/usr/bin/env python3
"""
app.py — Smart Office Waste Management System — Flask Dashboard
Runs concurrently with sensor_node.py on port 5000.

Routes:
  GET  /             — colour-coded dashboard (Jinja2 template)
  GET  /api/status   — JSON snapshot of current sensor state
  POST /bin/collected — mark bin as collected (resets fill state)
"""

import json
import os
import time
from datetime import datetime, timezone

from flask import Flask, jsonify, render_template, redirect, url_for

import config

app = Flask(__name__)

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

    # Provide safe defaults so templates never crash on missing keys
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
        "ir_count":        0,
        "alert":           False,
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
    """Return a Bootstrap / CSS colour name for a given fill percentage."""
    if fill_pct >= 80:
        return "danger"   # red
    if fill_pct >= 50:
        return "warning"  # amber
    return "success"      # green


def _thingspeak_chart_url(field: int, title: str) -> str:
    """Return the ThingSpeak chart iframe src URL for a given field."""
    return (
        f"https://thingspeak.com/channels/{config.THINGSPEAK_CHANNEL_ID}"
        f"/charts/{field}?bgcolor=%23ffffff&color=%23d62020"
        f"&dynamic=true&results=40&title={title}&type=line"
    )


# ─────────────────────────────────────────────────────────────────────────────
#  Routes
# ─────────────────────────────────────────────────────────────────────────────

@app.route("/")
def dashboard():
    state      = _read_state()
    fill_pct   = float(state["fill_pct"])
    fill_colour = _fill_colour(fill_pct)

    # Latest deposit image (relative path for url_for / static serving)
    image_filename = "images/latest_deposit.jpg"
    image_abs      = os.path.join(app.static_folder, image_filename)
    has_image      = os.path.isfile(image_abs)

    charts = [
        {"src": _thingspeak_chart_url(1, "Fill+%25"),          "title": "Fill Level (%)"},
        {"src": _thingspeak_chart_url(2, "Bin+Temp"),          "title": "Bin Temperature (°C)"},
        {"src": _thingspeak_chart_url(3, "Bin+Humidity"),      "title": "Bin Humidity (%)"},
        {"src": _thingspeak_chart_url(4, "Office+Temp"),       "title": "Office Temperature (°C)"},
        {"src": _thingspeak_chart_url(5, "Office+Humidity"),   "title": "Office Humidity (%)"},
        {"src": _thingspeak_chart_url(6, "PIR+Count"),         "title": "PIR Approach Count"},
        {"src": _thingspeak_chart_url(7, "IR+Deposits"),       "title": "IR Deposit Count"},
        {"src": _thingspeak_chart_url(8, "Urgency+Score"),     "title": "Urgency Score (MATLAB)"},
    ]

    return render_template(
        "dashboard.html",
        state        = state,
        fill_pct     = fill_pct,
        fill_colour  = fill_colour,
        has_image    = has_image,
        image_url    = url_for("static", filename=image_filename) if has_image else None,
        charts       = charts,
        channel_id   = config.THINGSPEAK_CHANNEL_ID,
        alert_thr    = config.FILL_ALERT_THRESHOLD,
        now          = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC"),
    )


@app.route("/api/status")
def api_status():
    """JSON endpoint — current sensor state for live polling / external tools."""
    state = _read_state()
    state["fill_colour"] = _fill_colour(float(state["fill_pct"]))
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
