from functools import lru_cache

import numpy as np
from astropy import units as u
from astropy.time import Time
from skyfield.api import EarthSatellite, load
from skyfield.timelib import Time as SkyfieldTime

from simulate.astronomy.utils.iss_tle import resolve_iss_tle_lines

_iss_satellite: EarthSatellite | None = None
_iss_satellite_tle: tuple[str, str, str] | None = None


@lru_cache(maxsize=1)
def _timescale():
    return load.timescale()


def refresh_iss_tle() -> None:
    """Download the latest ISS TLE when possible and rebuild the satellite if it changed."""
    global _iss_satellite, _iss_satellite_tle

    name, line1, line2 = resolve_iss_tle_lines()
    tle = (name, line1, line2)
    if _iss_satellite is not None and _iss_satellite_tle == tle:
        return
    _iss_satellite = EarthSatellite(line1, line2, name, _timescale())
    _iss_satellite_tle = tle


def _iss_satellite_or_raise() -> EarthSatellite:
    if _iss_satellite is None:
        refresh_iss_tle()
    if _iss_satellite is None:
        raise RuntimeError("ISS satellite is not initialized")
    return _iss_satellite


def iss_orbital_period(time: Time) -> u.Quantity:
    """Sidereal orbital period from the active TLE mean motion."""
    _ = time
    mean_motion_rad_per_min = _iss_satellite_or_raise().model.no_kozai
    period_min = (2.0 * np.pi / mean_motion_rad_per_min) * u.min
    return period_min.to(u.s)


def iss_geocentric_gcrs_km(time: Time) -> np.ndarray:
    skyfield_time = _astropy_to_skyfield_time(time)
    position = _iss_satellite_or_raise().at(skyfield_time).position.km
    return np.array(position, dtype=float)


def _astropy_to_skyfield_time(time: Time) -> SkyfieldTime:
    return _timescale().from_astropy(time)
