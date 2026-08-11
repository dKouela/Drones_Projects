"""Autonomous drone toolkit built on DroneKit + ArduCopter SITL.

Importing this package applies the DroneKit compatibility shims *first*, so any
submodule (and any user script that does ``import drone``) can safely import
dronekit afterwards.
"""

from drone import compat  # noqa: F401  -- must be first, patches `collections`

__all__ = ["compat"]
