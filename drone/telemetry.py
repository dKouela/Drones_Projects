"""Telemetry snapshots and a background reporter thread."""

import logging
import threading
import time

log = logging.getLogger(__name__)


def snapshot(vehicle):
    """Grab the interesting vehicle state as a plain dict."""
    loc = vehicle.location.global_relative_frame
    return {
        "mode": vehicle.mode.name,
        "armed": vehicle.armed,
        "lat": loc.lat,
        "lon": loc.lon,
        "alt_m": loc.alt,
        "groundspeed": vehicle.groundspeed,
        "heading": vehicle.heading,
        "battery_v": getattr(vehicle.battery, "voltage", None),
        "battery_pct": getattr(vehicle.battery, "level", None),
        "gps_fix": vehicle.gps_0.fix_type,
        "satellites": vehicle.gps_0.satellites_visible,
        "ekf_ok": vehicle.ekf_ok,
    }


def format_line(state):
    return (
        "{mode:<8} alt={alt_m:6.1f}m spd={groundspeed:4.1f}m/s "
        "hdg={heading:3d}deg sats={satellites} batt={battery_v:.1f}V"
    ).format(**{**state, "heading": int(state["heading"] or 0),
                "battery_v": state["battery_v"] or 0.0})


class Reporter(threading.Thread):
    """Prints a telemetry line every ``interval`` seconds until stopped."""

    def __init__(self, vehicle, interval=2.0):
        super().__init__(daemon=True, name="telemetry")
        self.vehicle = vehicle
        self.interval = interval
        self._stop_event = threading.Event()

    def run(self):
        while not self._stop_event.wait(self.interval):
            try:
                log.info("[telem] %s", format_line(snapshot(self.vehicle)))
            except Exception as exc:  # a dropped link shouldn't kill the mission
                log.debug("telemetry read failed: %s", exc)

    def stop(self):
        self._stop_event.set()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False
