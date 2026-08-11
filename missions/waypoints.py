"""Upload a waypoint mission and let the autopilot fly it in AUTO mode.

Contrast with ``square``: here the whole route is handed to ArduCopter up front,
so the flight continues even if the companion computer or link drops out. This
is how real autonomous sorties are usually structured.
"""

import logging
import math

import drone  # noqa: F401  -- must precede `dronekit`; installs the 3.10+ shims
from dronekit import LocationGlobalRelative

from drone import control
from drone.geo import offset

DESCRIPTION = "Upload a circular waypoint route and fly it in AUTO mode"

log = logging.getLogger(__name__)


def add_arguments(parser):
    parser.add_argument("--alt", type=float, default=20.0,
                        help="mission altitude in metres (default: 20)")
    parser.add_argument("--radius", type=float, default=40.0,
                        help="radius of the route in metres (default: 40)")
    parser.add_argument("--points", type=int, default=6,
                        help="number of waypoints around the circle (default: 6)")


def build_route(home, radius_m, points, alt_m):
    """Return ``points`` locations evenly spaced around ``home``."""
    route = []
    for i in range(points):
        angle = 2 * math.pi * i / points
        wp = offset(home, radius_m * math.cos(angle), radius_m * math.sin(angle))
        route.append(LocationGlobalRelative(wp.lat, wp.lon, alt_m))
    return route


def run(vehicle, args):
    if args.points < 3:
        raise ValueError("--points must be at least 3")

    home = vehicle.location.global_relative_frame
    route = build_route(home, args.radius, args.points, args.alt)
    log.info("Built a %d-point route of radius %.0f m", len(route), args.radius)

    rtl_index = control.upload_mission(vehicle, route)

    # Arm and climb in GUIDED, then hand control to the uploaded mission.
    control.arm_and_takeoff(vehicle, args.alt)
    control.run_mission(vehicle, rtl_index)

    # run_mission returns as RTL begins; wait for the vehicle to settle.
    control.return_to_launch(vehicle)
