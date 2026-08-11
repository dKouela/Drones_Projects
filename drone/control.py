"""Flight primitives: pre-arm checks, arming, takeoff, navigation, landing.

Every blocking helper takes a ``timeout`` and raises :class:`FlightError` rather
than spinning forever, so a stuck mission fails loudly instead of hanging.
"""

import logging
import time

from dronekit import Command, LocationGlobalRelative
from pymavlink import mavutil

from drone.geo import distance_m

log = logging.getLogger(__name__)

# Fraction of the target altitude that counts as "reached" during takeoff.
TAKEOFF_ALT_TOLERANCE = 0.95
# How close (metres) counts as having arrived at a waypoint.
WAYPOINT_RADIUS_M = 1.5
# How often to refresh an in-progress GUIDED target (seconds).
GOTO_RESEND_INTERVAL_S = 5.0


class FlightError(RuntimeError):
    """Raised when a flight operation fails or times out."""


def _wait_for(predicate, timeout, description, poll=0.5):
    """Poll ``predicate`` until it is true, or raise :class:`FlightError`."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(poll)
    raise FlightError(f"Timed out after {timeout}s waiting for {description}")


def wait_for_position(vehicle, timeout=120):
    """Block until the vehicle reports a usable global position.

    DroneKit's ``wait_ready`` covers parameters, mode, armed state, GPS and
    attitude -- but *not* location. Immediately after connecting, lat/lon
    therefore still read 0.0, and any mission that plans a route relative to
    "home" would lay it out in the Gulf of Guinea and fly off to sea.

    (A vehicle genuinely sitting at 0N 0E would defeat the check, which is a
    trade worth making.)
    """
    log.info("Waiting for a valid GPS position...")

    def has_position():
        loc = vehicle.location.global_frame
        if loc.lat is None or loc.lon is None:
            return False
        if abs(loc.lat) < 1e-6 and abs(loc.lon) < 1e-6:
            return False
        return vehicle.gps_0.fix_type >= 3

    _wait_for(has_position, timeout, "a valid GPS position")
    loc = vehicle.location.global_frame
    log.info("Position acquired: %.6f, %.6f", loc.lat, loc.lon)


def wait_until_armable(vehicle, timeout=120):
    """Block until the EKF has converged and the autopilot will accept arming."""
    log.info("Waiting for vehicle to become armable (EKF/GPS settling)...")

    def ready():
        if vehicle.is_armable:
            return True
        log.debug(
            "  not armable yet: gps_fix=%s sats=%s ekf_ok=%s",
            vehicle.gps_0.fix_type, vehicle.gps_0.satellites_visible, vehicle.ekf_ok,
        )
        return False

    _wait_for(ready, timeout, "vehicle to become armable")
    log.info("Vehicle is armable (GPS fix %s, %s sats)",
             vehicle.gps_0.fix_type, vehicle.gps_0.satellites_visible)


def _send_set_mode(vehicle, mode_name):
    """Request a mode change using the legacy ``SET_MODE`` message.

    DroneKit's ``vehicle.mode = ...`` setter delegates to pymavlink, which since
    2.4.x sends ``MAV_CMD_DO_SET_MODE`` as a COMMAND_LONG. ArduCopter 3.3 -- the
    build dronekit-sitl ships -- predates that and silently ignores it, so the
    mode never changes. The older ``SET_MODE`` message is understood by both old
    and current ArduPilot, so we send that directly.
    """
    mapping = vehicle._mode_mapping
    if mode_name not in mapping:
        raise FlightError(
            f"Mode {mode_name!r} is not supported by this vehicle "
            f"(known modes: {', '.join(sorted(mapping))})"
        )
    master = vehicle._master
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode_name],
    )


def set_mode(vehicle, mode_name, timeout=30):
    """Switch flight mode and confirm the autopilot accepted it."""
    if vehicle.mode.name == mode_name:
        return

    log.info("Switching mode -> %s", mode_name)
    deadline = time.time() + timeout
    while time.time() < deadline:
        # Re-send each cycle: a single request can be dropped, and the
        # autopilot may refuse until its pre-arm state allows the mode.
        _send_set_mode(vehicle, mode_name)
        time.sleep(1)
        if vehicle.mode.name == mode_name:
            log.info("Mode is now %s", vehicle.mode.name)
            return
    raise FlightError(
        f"Timed out after {timeout}s switching to {mode_name} "
        f"(still in {vehicle.mode.name})"
    )


def arm(vehicle, timeout=60):
    """Arm the motors (vehicle must already be in an armable mode)."""
    log.info("Arming motors")
    vehicle.armed = True
    # The autopilot can silently reject arming, so keep re-asserting it.
    deadline = time.time() + timeout
    while not vehicle.armed and time.time() < deadline:
        vehicle.armed = True
        time.sleep(1)
    if not vehicle.armed:
        raise FlightError(f"Failed to arm within {timeout}s")
    log.info("Armed")


def arm_and_takeoff(vehicle, target_alt_m, timeout=120):
    """Full launch sequence: GUIDED -> arm -> climb to ``target_alt_m``."""
    if target_alt_m <= 0:
        raise ValueError("target_alt_m must be positive")

    wait_until_armable(vehicle)
    set_mode(vehicle, "GUIDED")
    arm(vehicle)

    log.info("Taking off to %.1f m", target_alt_m)
    vehicle.simple_takeoff(target_alt_m)

    def at_altitude():
        alt = vehicle.location.global_relative_frame.alt
        log.info("  climbing: %.1f m", alt)
        return alt >= target_alt_m * TAKEOFF_ALT_TOLERANCE

    # simple_takeoff only *starts* the climb; we must wait for it ourselves.
    _wait_for(at_altitude, timeout, f"takeoff to {target_alt_m}m", poll=1.0)
    log.info("Reached target altitude")


def goto(vehicle, location, groundspeed=None, radius_m=WAYPOINT_RADIUS_M, timeout=180):
    """Fly to ``location`` in GUIDED mode and block until within ``radius_m``."""
    set_mode(vehicle, "GUIDED")
    if groundspeed is not None:
        vehicle.groundspeed = groundspeed

    start = vehicle.location.global_relative_frame
    leg_length = distance_m(start, location)
    log.info("Going to (%.6f, %.6f) alt %.1fm  [%.1f m away]",
             location.lat, location.lon, location.alt, leg_length)

    deadline = time.time() + timeout
    last_sent = 0.0
    while time.time() < deadline:
        # A GUIDED target is a fire-and-forget message with no acknowledgement.
        # If it is dropped the vehicle simply hovers, so refresh it periodically
        # rather than trusting a single send.
        if time.time() - last_sent >= GOTO_RESEND_INTERVAL_S:
            vehicle.simple_goto(location, groundspeed=groundspeed)
            last_sent = time.time()

        remaining = distance_m(vehicle.location.global_relative_frame, location)
        log.info("  %.1f m remaining", remaining)
        if remaining <= radius_m:
            log.info("Waypoint reached")
            return
        time.sleep(1.0)

    raise FlightError(
        f"Timed out after {timeout}s flying to "
        f"({location.lat:.6f}, {location.lon:.6f})"
    )


def goto_offset(vehicle, north_m, east_m, alt_m=None, **kwargs):
    """Fly to a point offset from the *current* position, in metres."""
    from drone.geo import offset  # local import keeps geo optional for users

    here = vehicle.location.global_relative_frame
    target = offset(here, north_m, east_m)
    if alt_m is not None:
        target = LocationGlobalRelative(target.lat, target.lon, alt_m)
    goto(vehicle, target, **kwargs)


def upload_mission(vehicle, waypoints):
    """Replace the onboard mission with ``waypoints``, ending in an RTL.

    ``waypoints`` is a sequence of :class:`LocationGlobalRelative`.

    DroneKit reserves sequence 0 for the home position and starts the commands
    you add at sequence 1, so no placeholder command is needed. (DroneKit's own
    examples prepend a dummy "home" item; on ArduCopter 4.x that is *not*
    absorbed as home and instead becomes a live waypoint to 0N 0E.)

    Returns the sequence index of the final RTL command, which
    :func:`run_mission` uses to detect completion.
    """
    cmds = vehicle.commands
    log.info("Downloading existing mission")
    cmds.download()
    cmds.wait_ready()
    cmds.clear()

    def _cmd(command, p5, p6, p7):
        return Command(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command,
            0, 0,          # current, autocontinue
            0, 0, 0, 0,    # params 1-4
            p5, p6, p7,    # x (lat), y (lon), z (alt)
        )

    for wp in waypoints:
        cmds.add(_cmd(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, wp.lat, wp.lon, wp.alt))

    # Return to launch once the last waypoint is visited.
    cmds.add(_cmd(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0))

    log.info("Uploading %d waypoints + RTL", len(waypoints))
    cmds.upload()

    # Read the mission back rather than assuming how it was numbered: where the
    # autopilot places the commands relative to the home slot varies between
    # firmware versions, and guessing wrong means run_mission() either finishes
    # a leg early or never sees the mission end.
    cmds.download()
    cmds.wait_ready()

    rtl_index = None
    stored_waypoints = 0
    for command in cmds:
        if command.command == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            rtl_index = command.seq
        elif command.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
            stored_waypoints += 1

    if rtl_index is None:
        raise FlightError("Mission upload failed: no RTL command was stored")
    if stored_waypoints != len(waypoints):
        raise FlightError(
            f"Mission upload mismatch: sent {len(waypoints)} waypoints but the "
            f"autopilot stored {stored_waypoints}"
        )

    log.info("Mission uploaded and verified; RTL is command %d", rtl_index)
    return rtl_index


def run_mission(vehicle, rtl_index, timeout=600, poll=1.0):
    """Switch to AUTO and block until the mission hands over to RTL.

    ``rtl_index`` is the value returned by :func:`upload_mission`. The vehicle
    is still airborne when this returns; follow it with :func:`return_to_launch`
    (or just wait for disarm) to see the flight all the way down.
    """
    set_mode(vehicle, "AUTO")
    log.info("Mission running")

    def finished():
        next_wp = vehicle.commands.next
        loc = vehicle.location.global_relative_frame
        log.info("  at command %s/%s  mode=%s alt=%.1fm spd=%.1fm/s",
                 next_wp, rtl_index, vehicle.mode.name, loc.alt, vehicle.groundspeed)
        return next_wp >= rtl_index

    _wait_for(finished, timeout, "mission completion", poll=poll)
    log.info("All waypoints visited; RTL engaged")


def land(vehicle, timeout=180):
    """Land in place and block until the motors disarm."""
    set_mode(vehicle, "LAND")
    log.info("Landing")

    def down():
        alt = vehicle.location.global_relative_frame.alt
        log.info("  descending: %.1f m (armed=%s)", alt, vehicle.armed)
        return not vehicle.armed

    _wait_for(down, timeout, "landing and disarm", poll=1.0)
    log.info("Landed and disarmed")


def return_to_launch(vehicle, timeout=300):
    """Trigger RTL and block until the vehicle disarms back at home."""
    set_mode(vehicle, "RTL")
    log.info("Returning to launch")

    def home_and_disarmed():
        loc = vehicle.location.global_relative_frame
        log.info("  RTL: alt=%.1f m (armed=%s)", loc.alt, vehicle.armed)
        return not vehicle.armed

    _wait_for(home_and_disarmed, timeout, "RTL completion", poll=1.0)
    log.info("Back home and disarmed")
