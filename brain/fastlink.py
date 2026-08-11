"""High-rate control link to the autopilot.

DroneKit's ``simple_goto`` sends a ``MISSION_ITEM`` -- a "fly there eventually"
instruction that current firmware warns about on every call. It is far too slow
and too coarse to close a visual loop with.

This module bypasses it and streams ``SET_POSITION_TARGET_LOCAL_NED`` velocity
setpoints in the vehicle's own body frame, which is the standard way to fly a
copter from a companion computer.

Two properties matter and both are handled by the streaming thread:

* ArduCopter treats guided velocity setpoints as perishable and stops the
  vehicle if they go stale (``GUID_TIMEOUT``, 3 s by default). Setpoints must be
  repeated whether or not the decision has changed.
* The rate the vehicle is commanded at should not be tied to how fast decisions
  are made. The streamer re-sends the latest setpoint at a fixed rate; the
  controller updates it whenever it has something new.

The watchdog is the safety half of the same mechanism: if nothing refreshes the
setpoint within ``setpoint_timeout``, the streamer commands zero velocity
rather than continuing to fly on a stale intention.
"""

import logging
import threading
import time

from pymavlink import mavutil

from brain.latency import LatencyTracker

log = logging.getLogger(__name__)

# SET_POSITION_TARGET_LOCAL_NED type_mask: ignore position and acceleration,
# use velocity and yaw rate.
#   bits 0-2  position    ignored
#   bits 3-5  velocity    used
#   bits 6-8  acceleration ignored
#   bit  10   yaw          ignored
#   bit  11   yaw rate     used
VELOCITY_AND_YAW_RATE = 0b0000010111000111  # 1479

# Velocities are expressed relative to the vehicle's nose, not to north.
BODY_FRAME = mavutil.mavlink.MAV_FRAME_BODY_NED


class Setpoint:
    """A body-frame velocity command. Metres per second, radians per second."""

    __slots__ = ("forward", "right", "down", "yaw_rate", "issued_at")

    def __init__(self, forward=0.0, right=0.0, down=0.0, yaw_rate=0.0, issued_at=None):
        self.forward = forward
        self.right = right
        self.down = down
        self.yaw_rate = yaw_rate
        self.issued_at = issued_at if issued_at is not None else time.perf_counter()

    def is_zero(self):
        return not any((self.forward, self.right, self.down, self.yaw_rate))

    def __repr__(self):
        return (f"Setpoint(fwd={self.forward:+.2f} right={self.right:+.2f} "
                f"down={self.down:+.2f} yaw_rate={self.yaw_rate:+.2f})")


class FastLink:
    """Streams the latest velocity setpoint to the vehicle at a fixed rate."""

    def __init__(self, vehicle, rate_hz=50.0, setpoint_timeout=0.5):
        self.vehicle = vehicle
        self.rate_hz = rate_hz
        self.setpoint_timeout = setpoint_timeout

        self._master = vehicle._master
        self._setpoint = Setpoint()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

        self.sent = 0
        self.stale_stops = 0
        self.jitter = LatencyTracker("setpoint interval")

    # -- commanding -------------------------------------------------------

    def command_velocity(self, forward=0.0, right=0.0, down=0.0, yaw_rate=0.0):
        """Replace the active setpoint. Returns immediately."""
        with self._lock:
            self._setpoint = Setpoint(forward, right, down, yaw_rate)

    def hold(self):
        """Command zero velocity -- hover in place."""
        self.command_velocity()

    @property
    def setpoint(self):
        with self._lock:
            return self._setpoint

    # -- stream rates -----------------------------------------------------

    def request_message_interval(self, message_id, frequency_hz):
        """Ask the autopilot to send a message faster than its default rate.

        The brain is only as current as its picture of the vehicle, so the
        attitude and position messages the controller depends on are requested
        explicitly instead of relying on the default stream rates.
        """
        interval_us = 0 if frequency_hz <= 0 else int(1e6 / frequency_hz)
        self._master.mav.command_long_send(
            self._master.target_system, self._master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, interval_us, 0, 0, 0, 0, 0,
        )

    def request_fast_telemetry(self, frequency_hz=20):
        """Raise the rate of the messages the control loop reads."""
        for message_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
        ):
            self.request_message_interval(message_id, frequency_hz)
        log.info("Requested %d Hz telemetry for attitude and position", frequency_hz)

    # -- lifecycle --------------------------------------------------------

    def start(self):
        if self._thread is not None:
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="fastlink", daemon=True)
        self._thread.start()
        log.info("FastLink streaming setpoints at %.0f Hz", self.rate_hz)
        return self

    def _run(self):
        interval = 1.0 / self.rate_hz
        next_send = time.perf_counter()
        last_send = None

        while not self._stop.is_set():
            now = time.perf_counter()
            if now < next_send:
                time.sleep(min(next_send - now, interval))
                continue

            with self._lock:
                setpoint = self._setpoint

            # Watchdog: never keep flying on an intention nobody refreshed.
            if now - setpoint.issued_at > self.setpoint_timeout:
                if not setpoint.is_zero():
                    log.warning("Setpoint stale after %.2fs -- commanding hold",
                                now - setpoint.issued_at)
                    self.stale_stops += 1
                    with self._lock:
                        self._setpoint = Setpoint()
                setpoint = Setpoint()

            self._send(setpoint)
            self.sent += 1

            if last_send is not None:
                self.jitter.record((now - last_send) * 1000.0)
            last_send = now

            next_send += interval
            # If we fell badly behind, resynchronise rather than burst-sending.
            if next_send < now:
                next_send = now + interval

    def _send(self, setpoint):
        self._master.mav.set_position_target_local_ned_send(
            0,                                    # time_boot_ms
            self._master.target_system,
            self._master.target_component,
            BODY_FRAME,
            VELOCITY_AND_YAW_RATE,
            0, 0, 0,                              # position (ignored)
            setpoint.forward, setpoint.right, setpoint.down,
            0, 0, 0,                              # acceleration (ignored)
            0,                                    # yaw (ignored)
            setpoint.yaw_rate,
        )

    def stop(self):
        if self._thread is None:
            return
        # Command a stop before dropping the link, so the vehicle does not
        # coast on the last velocity until GUID_TIMEOUT expires.
        self.hold()
        self._send(Setpoint())
        self._stop.set()
        self._thread.join(timeout=2)
        self._thread = None
        log.info("FastLink stopped after %d setpoints (%s)",
                 self.sent, self.jitter.summary())

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False
