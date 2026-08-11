"""Vehicle connection handling."""

import contextlib
import logging

import dronekit

log = logging.getLogger(__name__)


def connect_vehicle(connection_string, wait_ready=True, timeout=120):
    """Connect to a vehicle and wait until its state is populated.

    ``wait_ready=True`` blocks until DroneKit has received the attributes the
    missions rely on (position, mode, armed state, ...). Without it the first
    read of ``vehicle.location`` can return ``None``.
    """
    log.info("Connecting to vehicle on %s", connection_string)
    vehicle = dronekit.connect(
        connection_string,
        wait_ready=wait_ready,
        heartbeat_timeout=timeout,
        timeout=timeout,
    )
    log.info(
        "Connected: firmware=%s type=%s mode=%s armed=%s",
        vehicle.version, vehicle._vehicle_type, vehicle.mode.name, vehicle.armed,
    )
    return vehicle


@contextlib.contextmanager
def vehicle_session(connection_string, **kwargs):
    """Context manager that always closes the MAVLink connection."""
    vehicle = connect_vehicle(connection_string, **kwargs)
    try:
        yield vehicle
    finally:
        log.info("Closing vehicle connection")
        vehicle.close()
