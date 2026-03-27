#!/usr/bin/env python3
"""
predictor.py — Fill-level prediction for Smart Office Waste Management.

Records timestamped fill readings to a rolling JSON history file and uses
simple linear regression to estimate the fill rate and predict when the bin
will reach the alert threshold.
"""

import json
import os
import time
from datetime import datetime, timezone

HISTORY_FILE = "/tmp/fill_history.json"
MAX_HISTORY  = 150   # keep at most ~5 min of readings at 2 s intervals
MIN_POINTS   = 5     # minimum readings needed before producing a prediction
ALERT_PCT    = 80.0  # threshold to predict time-to (matches config.FILL_ALERT_THRESHOLD)


# ─────────────────────────────────────────────────────────────────────────────
#  Public API
# ─────────────────────────────────────────────────────────────────────────────

def record_fill(fill_pct: float) -> None:
    """Append a fill reading (with current Unix timestamp) to rolling history."""
    history = _load_history()
    history.append({"ts": time.time(), "fill": float(fill_pct)})
    if len(history) > MAX_HISTORY:
        history = history[-MAX_HISTORY:]
    _save_history(history)


def clear_history() -> None:
    """Reset fill history — call this when the bin is collected."""
    _save_history([])


def get_prediction() -> dict:
    """
    Compute fill rate and time-to-alert from recorded history.

    Returns a dict with:
      fill_rate_per_hour  float | None   — % per hour (positive = filling)
      hours_until_full    float | None   — hours until fill reaches ALERT_PCT
      predicted_full_at   str   | None   — human-readable UTC timestamp
      confidence          str            — 'low' | 'medium' | 'high'
      history_points      int            — number of readings available
    """
    history = _load_history()
    n = len(history)

    result = {
        "fill_rate_per_hour": None,
        "hours_until_full":   None,
        "predicted_full_at":  None,
        "confidence":         "low",
        "history_points":     n,
    }

    if n < MIN_POINTS:
        return result

    # Use the most recent 80 readings for the regression to keep it responsive
    points = history[-80:]
    xs = [p["ts"]   for p in points]
    ys = [p["fill"] for p in points]

    # Ordinary least-squares linear regression
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    num = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
    den = sum((x - x_mean) ** 2 for x in xs)

    if den == 0:
        return result

    slope = num / den                        # %/second
    fill_rate_per_hour = round(slope * 3600, 2)
    result["fill_rate_per_hour"] = fill_rate_per_hour

    current_fill = ys[-1]
    current_ts   = xs[-1]

    # Only predict if the bin is actually filling and not yet at threshold
    if fill_rate_per_hour > 0.05 and current_fill < ALERT_PCT:
        hours_until = (ALERT_PCT - current_fill) / fill_rate_per_hour
        result["hours_until_full"] = round(hours_until, 1)
        predicted_unix = current_ts + hours_until * 3600
        result["predicted_full_at"] = datetime.fromtimestamp(
            predicted_unix, tz=timezone.utc
        ).strftime("%Y-%m-%d %H:%M UTC")

    # Confidence: more points + longer time span = higher confidence
    time_span_hours = (xs[-1] - xs[0]) / 3600
    if n >= 40 and time_span_hours >= 1.0:
        result["confidence"] = "high"
    elif n >= 10 and time_span_hours >= 0.05:
        result["confidence"] = "medium"

    return result


# ─────────────────────────────────────────────────────────────────────────────
#  Internal helpers
# ─────────────────────────────────────────────────────────────────────────────

def _load_history() -> list:
    try:
        with open(HISTORY_FILE) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return []


def _save_history(history: list) -> None:
    tmp = HISTORY_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(history, f)
    os.replace(tmp, HISTORY_FILE)
