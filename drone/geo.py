"""Small-scale geodesy helpers.

All of these use a flat-earth approximation around the reference point. That is
accurate to well under a metre over the few-hundred-metre distances these
missions fly, and it keeps the maths readable.
"""

import math

from dronekit import LocationGlobal, LocationGlobalRelative

EARTH_RADIUS_M = 6378137.0


def offset(location, north_m, east_m):
    """Return a new location ``north_m``/``east_m`` metres from ``location``.

    The altitude and location class (global vs. global-relative) are preserved.
    """
    d_lat = north_m / EARTH_RADIUS_M
    d_lon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(location.lat)))

    new_lat = location.lat + math.degrees(d_lat)
    new_lon = location.lon + math.degrees(d_lon)

    if isinstance(location, LocationGlobal):
        return LocationGlobal(new_lat, new_lon, location.alt)
    return LocationGlobalRelative(new_lat, new_lon, location.alt)


def distance_m(a, b):
    """Ground distance in metres between two locations (altitude ignored)."""
    d_lat = math.radians(b.lat - a.lat)
    d_lon = math.radians(b.lon - a.lon) * math.cos(math.radians(a.lat))
    return math.hypot(d_lat, d_lon) * EARTH_RADIUS_M


def bearing_deg(a, b):
    """Initial compass bearing from ``a`` to ``b``, in degrees (0-360)."""
    d_lon = math.radians(b.lon - a.lon)
    lat_a, lat_b = math.radians(a.lat), math.radians(b.lat)
    y = math.sin(d_lon) * math.cos(lat_b)
    x = math.cos(lat_a) * math.sin(lat_b) - math.sin(lat_a) * math.cos(lat_b) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360
