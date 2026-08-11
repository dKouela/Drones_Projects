"""Onboard autonomy ("the brain").

Perception and reactive control, kept deliberately separate from the mission
code in :mod:`drone`:

    frames.py      where images come from (webcam, synthetic, file)
    detectors.py   what is in them (pluggable; DNN or classical)
    perception.py  runs the two above in their own thread at camera rate
    fastlink.py    high-rate MAVLink velocity/yaw-rate control
    follow.py      turns a detection into a velocity command

The split matters for latency: inference runs in the perception thread while
the control loop reads the most recent result and keeps streaming setpoints at
a fixed rate. The vehicle is never left uncommanded waiting on a slow frame.

Every hardware-specific edge (camera capture, inference backend, link to the
flight controller) sits behind an interface, so moving from a laptop plus SITL
to a Jetson plus a real Pixhawk is a matter of swapping implementations.
"""

from drone import compat  # noqa: F401  -- keep dronekit importable everywhere


class BrainError(RuntimeError):
    """Perception could not be set up (no camera, missing model, bad config).

    Raised before the vehicle is ever armed, so it always means "nothing was
    flown" -- main.py reports it as a one-line error rather than a traceback.
    """


__all__ = ["BrainError", "compat"]
