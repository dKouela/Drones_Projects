"""Visual follow behaviour.

Three independent proportional loops, each driven by one property of the
bounding box:

    horizontal offset  ->  yaw rate      (turn to face the target)
    box height         ->  forward speed (hold a standoff distance)
    vertical offset    ->  climb rate    (keep it level in frame)

Proportional control is deliberate. A derivative term on a noisy detector
amplifies box jitter into throttle noise, and an integral term winds up during
the seconds when the target is not visible at all. If this is later driven by a
tracker that produces smooth, gap-free estimates, a D term becomes worth adding.

Nothing here knows what a "face" or a "blob" is -- only where the box sits and
how big it looks.
"""

import logging
import time
from dataclasses import dataclass

from brain.fastlink import Setpoint
from brain.latency import LatencyReport

log = logging.getLogger(__name__)


@dataclass
class FollowConfig:
    """Gains, limits and safety envelope for the follow loop."""

    # Proportional gains, in output units per unit of normalised error.
    yaw_gain: float = 1.6            # rad/s per unit horizontal offset
    forward_gain: float = 6.0        # m/s per unit relative size error
    vertical_gain: float = 1.2       # m/s per unit vertical offset

    # Output limits.
    max_yaw_rate: float = 1.2        # rad/s  (~69 deg/s)
    max_forward: float = 4.0         # m/s
    max_vertical: float = 1.0        # m/s

    # Ignore errors smaller than these; stops the vehicle twitching at noise.
    offset_deadband: float = 0.06
    size_deadband: float = 0.05

    #: Desired box height as a fraction of frame height. Bigger = closer.
    target_size: float = 0.35

    # Safety envelope.
    lost_after_s: float = 0.6        # hover once the target is this stale
    abort_after_s: float = 6.0       # give up entirely after this long
    min_altitude_m: float = 3.0
    max_altitude_m: float = 40.0
    max_radius_m: float = 60.0       # never chase further than this from home

    control_rate_hz: float = 20.0


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def _deadband(value, width):
    return 0.0 if abs(value) < width else value


class FollowController:
    """Converts a detection into a body-frame velocity setpoint."""

    def __init__(self, config=None):
        self.config = config or FollowConfig()

    def compute(self, detection):
        """Return the :class:`Setpoint` for one detection."""
        cfg = self.config

        # Target right of centre -> positive offset -> yaw right to face it.
        offset_x = _deadband(detection.offset_x, cfg.offset_deadband)
        yaw_rate = _clamp(cfg.yaw_gain * offset_x, cfg.max_yaw_rate)

        # Box smaller than wanted -> target is far -> close the distance.
        size_error = (cfg.target_size - detection.size_ratio) / cfg.target_size
        size_error = _deadband(size_error, cfg.size_deadband)
        forward = _clamp(cfg.forward_gain * size_error, cfg.max_forward)

        # Target low in frame -> positive offset -> descend (NED: down is +).
        offset_y = _deadband(detection.offset_y, cfg.offset_deadband)
        down = _clamp(cfg.vertical_gain * offset_y, cfg.max_vertical)

        return Setpoint(forward=forward, right=0.0, down=down, yaw_rate=yaw_rate)

    def apply_envelope(self, setpoint, altitude_m, radius_m):
        """Clip a setpoint so it cannot leave the safety envelope.

        Returns ``(setpoint, reason)`` where ``reason`` is ``None`` when
        nothing was limited.
        """
        cfg = self.config
        reason = None

        if altitude_m <= cfg.min_altitude_m and setpoint.down > 0:
            setpoint.down = 0.0
            reason = f"at minimum altitude ({cfg.min_altitude_m:.0f} m)"
        elif altitude_m >= cfg.max_altitude_m and setpoint.down < 0:
            setpoint.down = 0.0
            reason = f"at maximum altitude ({cfg.max_altitude_m:.0f} m)"

        if radius_m >= cfg.max_radius_m and setpoint.forward > 0:
            setpoint.forward = 0.0
            reason = f"at maximum radius ({cfg.max_radius_m:.0f} m from home)"

        return setpoint, reason


def follow_target(vehicle, perception, link, config=None, duration_s=60.0):
    """Run the closed loop until the target is lost or ``duration_s`` elapses.

    Returns a summary dict; raises nothing on target loss, which is a normal
    outcome rather than an error.
    """
    from drone.geo import distance_m

    cfg = config or FollowConfig()
    controller = FollowController(cfg)
    latency = LatencyReport("capture->command")

    home = vehicle.location.global_relative_frame
    interval = 1.0 / cfg.control_rate_hz
    deadline = time.perf_counter() + duration_s

    tracked_s = 0.0
    last_seen = time.perf_counter()
    last_report = 0.0
    outcome = "duration reached"
    iterations = 0

    log.info("Following: holding target at %.0f%% of frame height, "
             "%.0f Hz control, %.0f s budget",
             cfg.target_size * 100, cfg.control_rate_hz, duration_s)

    while time.perf_counter() < deadline:
        cycle_start = time.perf_counter()
        iterations += 1

        detection = perception.latest
        now = time.perf_counter()

        # A detection is only useful while it is fresh; an old box describes
        # where the target *was*.
        fresh = detection is not None and (now - detection.captured_at) <= cfg.lost_after_s

        if fresh:
            last_seen = now
            tracked_s += interval

            setpoint = controller.compute(detection)

            location = vehicle.location.global_relative_frame
            altitude = location.alt
            radius = distance_m(home, location)
            setpoint, limited = controller.apply_envelope(setpoint, altitude, radius)

            link.command_velocity(setpoint.forward, setpoint.right,
                                  setpoint.down, setpoint.yaw_rate)
            latency["capture->command"].record(
                (time.perf_counter() - detection.captured_at) * 1000.0
            )

            if now - last_report >= 1.0:
                log.info("  target off=(%+.2f,%+.2f) size=%.2f -> "
                         "fwd=%+.1fm/s yaw=%+.0f deg/s down=%+.1fm/s%s",
                         detection.offset_x, detection.offset_y,
                         detection.size_ratio, setpoint.forward,
                         setpoint.yaw_rate * 57.2958, setpoint.down,
                         f"  [{limited}]" if limited else "")
                last_report = now
        else:
            # Hovering is the right response to "I cannot see it" -- the last
            # command must not persist.
            link.hold()
            lost_for = now - last_seen
            if lost_for >= cfg.abort_after_s:
                outcome = f"target lost for {lost_for:.1f}s"
                log.warning("Giving up: %s", outcome)
                break
            if now - last_report >= 1.0:
                log.info("  no target (%.1fs) -- holding", lost_for)
                last_report = now

        slack = interval - (time.perf_counter() - cycle_start)
        if slack > 0:
            time.sleep(slack)

    link.hold()

    stats = perception.stats
    summary = {
        "outcome": outcome,
        "iterations": iterations,
        "tracked_s": tracked_s,
        "frames": stats["frames"],
        "detection_rate": stats["hit_rate"],
        "camera_fps": stats["fps"],
        "setpoints_sent": link.sent,
        "latency_lines": (
            latency.lines() + perception.latency.lines() + [link.jitter.summary()]
        ),
    }
    return summary
