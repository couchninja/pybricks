"""Stable pointing targets: directions that stay in the same sky region over a year."""

import numpy as np
from astropy import units as u
from astropy.coordinates import BarycentricMeanEcliptic, SkyCoord
from astropy.time import Time

from simulate.astronomy.constants import PointingTarget
from simulate.astronomy.utils.ephemeris import (
    observer_direction_ecliptic_for_target,
    observer_surface_vector_and_euler_angles_for_target,
)

_CYGNUS = SkyCoord.from_name("Deneb")
_LEO = SkyCoord.from_name("Regulus")

# Sample dates across one calendar year at the fixed observer (constants.OBSERVER_*).
_YEAR_SAMPLE_DAYS = np.linspace(0, 365, 13)


def _direction_ecliptic_to_icrs(direction: np.ndarray) -> SkyCoord:
    ecliptic = BarycentricMeanEcliptic(
        x=direction[0] * u.one,
        y=direction[1] * u.one,
        z=direction[2] * u.one,
        representation_type="cartesian",
    )
    return SkyCoord(ecliptic).icrs


def _times_across_year(year: int = 2025) -> list[Time]:
    start = Time(f"{year}-01-01")
    return [start + float(day) * u.day for day in _YEAR_SAMPLE_DAYS]


def test_earth_rotation_points_east_all_year() -> None:
    for time in _times_across_year():
        _surface, (yaw, pitch, _roll), _speed = observer_surface_vector_and_euler_angles_for_target(
            time, PointingTarget.EARTH_ROTATION
        )
        assert abs(yaw - 90.0) < 1.0
        assert abs(pitch) < 1.0


def test_milky_way_orbit_points_toward_cygnus_all_year() -> None:
    for time in _times_across_year():
        direction = observer_direction_ecliptic_for_target(time, PointingTarget.MILKY_WAY_ORBIT)
        assert direction is not None
        sky = _direction_ecliptic_to_icrs(direction)
        print(f"Milky Way orbit: {sky.separation(_CYGNUS).deg} degrees")
        assert sky.separation(_CYGNUS).deg < 15.0


def test_cmb_dipole_points_toward_leo_all_year() -> None:
    for time in _times_across_year():
        direction = observer_direction_ecliptic_for_target(time, PointingTarget.CMB_DIPOLE)
        assert direction is not None
        sky = _direction_ecliptic_to_icrs(direction)
        print(f"CMB dipole: {sky.separation(_LEO).deg} degrees")
        assert sky.separation(_LEO).deg < 30.0
