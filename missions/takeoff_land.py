"""Simplest possible sortie: climb, hover, land. Good first smoke test."""

import logging
import time

from drone import control

DESCRIPTION = "Take off, hover, then land in place"

log = logging.getLogger(__name__)


def add_arguments(parser):
    parser.add_argument("--alt", type=float, default=10.0,
                        help="target altitude in metres (default: 10)")
    parser.add_argument("--hover", type=float, default=5.0,
                        help="seconds to hover before landing (default: 5)")


def run(vehicle, args):
    control.arm_and_takeoff(vehicle, args.alt)

    log.info("Hovering for %.0f s", args.hover)
    time.sleep(args.hover)

    control.land(vehicle)
