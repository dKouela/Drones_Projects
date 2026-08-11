"""Flight track recording.

Samples vehicle state during a mission and writes it as JSON, so a flight can
be replayed and inspected after the fact rather than only read as scrolling log
lines. ``tools/flightview.py`` turns the result into a viewable page.

This is deliberately independent of the telemetry reporter: that prints for a
human watching live, this keeps a complete record for afterwards.
"""

import json
import logging
import threading
import time

log = logging.getLogger(__name__)

SCHEMA_VERSION = 1


class FlightRecorder(threading.Thread):
    """Samples vehicle state at a fixed rate into an in-memory track."""

    def __init__(self, vehicle, rate_hz=5.0, note=""):
        super().__init__(daemon=True, name="recorder")
        self.vehicle = vehicle
        self.rate_hz = rate_hz
        self.note = note
        self.samples = []
        self.events = []
        self._stop_event = threading.Event()
        self._started_at = None

    # -- recording --------------------------------------------------------

    def mark(self, label):
        """Record a named moment (mode change, waypoint reached, ...)."""
        if self._started_at is None:
            return
        self.events.append({"t": time.perf_counter() - self._started_at,
                            "label": label})

    def _sample(self):
        vehicle = self.vehicle
        location = vehicle.location.global_relative_frame
        if location.lat is None or location.lon is None:
            return None
        attitude = vehicle.attitude
        return {
            "t": round(time.perf_counter() - self._started_at, 3),
            "lat": location.lat,
            "lon": location.lon,
            "alt": round(location.alt or 0.0, 2),
            "mode": vehicle.mode.name,
            "armed": bool(vehicle.armed),
            "groundspeed": round(vehicle.groundspeed or 0.0, 2),
            "heading": vehicle.heading,
            "roll": round(attitude.roll, 3),
            "pitch": round(attitude.pitch, 3),
            "yaw": round(attitude.yaw, 3),
            "battery": getattr(vehicle.battery, "voltage", None),
        }

    def run(self):
        self._started_at = time.perf_counter()
        interval = 1.0 / self.rate_hz
        last_mode = None

        while not self._stop_event.wait(interval):
            try:
                sample = self._sample()
            except Exception as exc:
                log.debug("recorder sample failed: %s", exc)
                continue
            if sample is None:
                continue
            self.samples.append(sample)

            # Mode changes are the structure of a flight; capture them for free.
            if sample["mode"] != last_mode:
                if last_mode is not None:
                    self.mark(f"mode {sample['mode']}")
                last_mode = sample["mode"]

    def stop(self):
        self._stop_event.set()

    # -- output -----------------------------------------------------------

    def track(self, mission="", firmware=""):
        """Return the recorded flight as a serialisable dict."""
        altitudes = [s["alt"] for s in self.samples] or [0.0]
        speeds = [s["groundspeed"] for s in self.samples] or [0.0]
        return {
            "schema": SCHEMA_VERSION,
            "mission": mission,
            "firmware": firmware,
            "note": self.note,
            "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "duration_s": round(self.samples[-1]["t"], 1) if self.samples else 0.0,
            "sample_count": len(self.samples),
            "max_altitude_m": round(max(altitudes), 1),
            "max_groundspeed_ms": round(max(speeds), 1),
            "events": self.events,
            "samples": self.samples,
        }

    def save(self, path, mission="", firmware=""):
        payload = self.track(mission=mission, firmware=firmware)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle)
        log.info("Recorded %d samples over %.1fs -> %s",
                 payload["sample_count"], payload["duration_s"], path)
        return payload

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False
