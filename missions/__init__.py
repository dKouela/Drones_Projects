"""Mission registry.

Each mission module exposes:
  ``DESCRIPTION``            -- one-line summary shown by ``main.py list``
  ``add_arguments(parser)``  -- register its CLI options
  ``run(vehicle, args)``     -- fly it
"""

from missions import follow, square, survey, takeoff_land, waypoints

REGISTRY = {
    "takeoff-land": takeoff_land,
    "square": square,
    "waypoints": waypoints,
    "survey": survey,
    "follow": follow,
}

__all__ = ["REGISTRY"]
