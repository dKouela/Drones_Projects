"""Fly a square using GUIDED-mode position targets, then return home.

This is the "closed loop from Python" style: the script decides where to go
next and streams targets to the autopilot, rather than uploading a mission.
"""

import logging

from drone import control

DESCRIPTION = "Fly a square of a given side length in GUIDED mode, then RTL"

log = logging.getLogger(__name__)


def add_arguments(parser):
    parser.add_argument("--alt", type=float, default=15.0,
                        help="flight altitude in metres (default: 15)")
    parser.add_argument("--size", type=float, default=25.0,
                        help="side length of the square in metres (default: 25)")
    parser.add_argument("--speed", type=float, default=5.0,
                        help="groundspeed in m/s (default: 5)")


def run(vehicle, args):
    control.arm_and_takeoff(vehicle, args.alt)

    side = args.size
    # North, East, South, West -- back to where we started.
    legs = [(side, 0), (0, side), (-side, 0), (0, -side)]

    for index, (north, east) in enumerate(legs, start=1):
        log.info("Leg %d/4: %+.0f m north, %+.0f m east", index, north, east)
        control.goto_offset(vehicle, north, east, groundspeed=args.speed)

    control.return_to_launch(vehicle)
