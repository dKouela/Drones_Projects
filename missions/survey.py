"""Lawnmower survey grid -- the pattern used for mapping and photogrammetry.

The area is covered by parallel strips spaced ``--spacing`` apart, flown in
alternating directions so the vehicle never backtracks.
"""

import logging

import drone  # noqa: F401  -- must precede `dronekit`; installs the 3.10+ shims
from dronekit import LocationGlobalRelative

from drone import control
from drone.geo import offset

DESCRIPTION = "Fly a lawnmower survey grid over a rectangular area in AUTO mode"

log = logging.getLogger(__name__)


def add_arguments(parser):
    parser.add_argument("--alt", type=float, default=30.0,
                        help="survey altitude in metres (default: 30)")
    parser.add_argument("--width", type=float, default=80.0,
                        help="east-west extent in metres (default: 80)")
    parser.add_argument("--height", type=float, default=60.0,
                        help="north-south extent in metres (default: 60)")
    parser.add_argument("--spacing", type=float, default=20.0,
                        help="distance between survey strips in metres (default: 20)")


def build_grid(origin, width_m, height_m, spacing_m, alt_m):
    """Return waypoints covering a ``width_m`` x ``height_m`` box from ``origin``.

    ``origin`` is the south-west corner. Strips run north-south and step east.
    """
    if spacing_m <= 0:
        raise ValueError("--spacing must be positive")

    waypoints = []
    strips = int(width_m // spacing_m) + 1
    for i in range(strips):
        east = i * spacing_m
        # Alternate direction each strip so the path is continuous.
        ends = (0, height_m) if i % 2 == 0 else (height_m, 0)
        for north in ends:
            point = offset(origin, north, east)
            waypoints.append(LocationGlobalRelative(point.lat, point.lon, alt_m))
    return waypoints


def run(vehicle, args):
    # Centre the survey box on home so the vehicle isn't asked to fly off in
    # one direction only.
    home = vehicle.location.global_relative_frame
    south_west = offset(home, -args.height / 2, -args.width / 2)

    grid = build_grid(south_west, args.width, args.height, args.spacing, args.alt)
    log.info("Survey grid: %.0f x %.0f m, %.0f m spacing -> %d waypoints",
             args.width, args.height, args.spacing, len(grid))

    rtl_index = control.upload_mission(vehicle, grid)

    control.arm_and_takeoff(vehicle, args.alt)
    control.run_mission(vehicle, rtl_index, timeout=900)
    control.return_to_launch(vehicle)
