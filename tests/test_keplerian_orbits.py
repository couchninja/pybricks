import numpy as np
import pytest
from astropy import units as u
from astropy.time import Time

from simulate.astronomy.constants import PointingTarget
from simulate.astronomy.keplerian import (
    keplerian_from_cartesian,
    keplerian_position_au,
    sun_gravitational_parameter_au3_per_day2,
)
from simulate.astronomy.utils import iss_tle
from simulate.astronomy.utils.ephemeris import earth_heliocentric_ecliptic_au
from simulate.astronomy.web_scene import reset_web_scene_cache, scene_snapshot_payload


@pytest.fixture(autouse=True)
def _iss_tle_offline() -> None:
    iss_tle.set_celestrak_fetch_allowed(False)


def test_keplerian_earth_matches_ephemeris_near_epoch() -> None:
    time = Time("2024-06-15T12:00:00", scale="utc")
    dt = 1.0 * u.minute
    r0 = earth_heliocentric_ecliptic_au(time)
    r1 = earth_heliocentric_ecliptic_au(time + dt)
    v = (r1 - r0) / dt.to(u.day).value
    elements = keplerian_from_cartesian(
        r0,
        v,
        mu_au3_per_day2=sun_gravitational_parameter_au3_per_day2(),
        epoch=time,
    )
    propagated = keplerian_position_au(elements, time + 6 * u.hour)
    expected = earth_heliocentric_ecliptic_au(time + 6 * u.hour)
    error_au = float(np.linalg.norm(propagated - expected))
    assert error_au < 5e-5


def test_moon_keplerian_elements_are_physical() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.MOON)
    moon = next(entry for entry in payload["parametric_orbits"] if entry["name"] == "moon_orbit")
    assert moon["period_days"] > 20.0
    assert moon["e"] < 0.1
    assert 0.002 < moon["a_au"] < 0.003


def test_scene_includes_parametric_keplerian_orbits() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.MOON)
    names = {entry["name"] for entry in payload["parametric_orbits"]}
    assert "earth_orbit" in names
    assert "moon_orbit" in names
    assert "galactic_orbit" in names
    earth = next(entry for entry in payload["parametric_orbits"] if entry["name"] == "earth_orbit")
    assert earth["kind"] == "keplerian"
    assert earth["a_au"] > 0.9


def test_geocentric_parametric_orbits_include_heliocentric_earth_origin() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.MOON)
    moon = next(entry for entry in payload["parametric_orbits"] if entry["name"] == "moon_orbit")
    assert moon["origin_heliocentric_au"] is not None
    assert len(moon["origin_heliocentric_au"]) == 3


def test_scene_iss_parametric_is_tle_when_pointing_iss() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.ISS)
    iss = next(entry for entry in payload["parametric_orbits"] if entry["name"] == "iss_orbit")
    assert iss["kind"] == "tle"
    assert iss["line1"].startswith("1 ")
    assert len(iss["eci_to_ecliptic"]) == 9
