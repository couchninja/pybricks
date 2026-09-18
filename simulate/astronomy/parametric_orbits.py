"""Parametric orbit payloads (Keplerian + TLE) for the web scene snapshot."""

from __future__ import annotations

from typing import Any

import numpy as np
from astropy import units as u
from astropy.time import Time

from simulate.astronomy.constants import (
    GALACTIC_ORBIT_DISTANCE_SCALE,
    KPC_TO_AU,
    SOLAR_GALACTIC_ORBITAL_SPEED,
    PointingTarget,
)
from simulate.astronomy.keplerian import (
    KeplerianElements,
    earth_gravitational_parameter_au3_per_day2,
    keplerian_from_cartesian,
    keplerian_position_au,
    sun_gravitational_parameter_au3_per_day2,
)
from simulate.astronomy.utils.ephemeris import (
    earth_heliocentric_ecliptic_au,
    gcrs_to_mean_ecliptic_rotation,
    moon_heliocentric_ecliptic_au,
    sun_galactocentric_kpc,
)
from simulate.astronomy.utils.iss import iss_orbital_period
from simulate.astronomy.utils.iss_tle import resolve_iss_tle_lines


def parametric_orbit_payloads(
    time: Time,
    pointing_target: PointingTarget,
) -> list[dict[str, Any]]:
    orbits: list[dict[str, Any]] = [
        _earth_orbit_keplerian(time),
        _moon_orbit_keplerian(time),
        _galactic_orbit_keplerian(time),
    ]
    if pointing_target == PointingTarget.ISS:
        orbits.append(_iss_orbit_tle(time))
    return orbits


def _earth_orbit_keplerian(time: Time) -> dict[str, Any]:
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
    elements["origin_body"] = None
    return _serialize_keplerian("earth_orbit", elements)


def _moon_orbit_keplerian(time: Time) -> dict[str, Any]:
    dt = 1.0 * u.minute
    moon0 = moon_heliocentric_ecliptic_au(time)
    moon1 = moon_heliocentric_ecliptic_au(time + dt)
    earth0 = earth_heliocentric_ecliptic_au(time)
    earth1 = earth_heliocentric_ecliptic_au(time + dt)
    r = moon0 - earth0
    v = (moon1 - moon0 - (earth1 - earth0)) / dt.to(u.day).value
    elements = keplerian_from_cartesian(
        r,
        v,
        mu_au3_per_day2=earth_gravitational_parameter_au3_per_day2(),
        epoch=time,
    )
    elements["origin_body"] = "earth"
    return _serialize_keplerian("moon_orbit", elements)


def _galactic_orbit_keplerian(time: Time) -> dict[str, Any]:
    sun_kpc = sun_galactocentric_kpc(time)
    radius_kpc = float(np.hypot(sun_kpc[0], sun_kpc[1]))
    radius_au = radius_kpc * KPC_TO_AU * GALACTIC_ORBIT_DISTANCE_SCALE
    speed_au_per_day = float(SOLAR_GALACTIC_ORBITAL_SPEED.to(u.au / u.day).value)
    period_days = float(2.0 * np.pi * radius_au / speed_au_per_day) if speed_au_per_day > 0.0 else 1.0
    theta = float(np.arctan2(sun_kpc[1], sun_kpc[0]))
    if theta < 0.0:
        theta += 2.0 * np.pi
    mu = (2.0 * np.pi / period_days) ** 2 * radius_au
    elements = KeplerianElements(
        epoch_iso=time.iso,
        period_days=period_days,
        a_au=radius_au,
        e=0.0,
        i_rad=0.0,
        raan_rad=0.0,
        argp_rad=0.0,
        M0_rad=theta,
        mu_au3_per_day2=mu,
        origin_body=None,
    )
    return _serialize_keplerian("galactic_orbit", elements)


def _iss_orbit_tle(time: Time) -> dict[str, Any]:
    _name, line1, line2 = resolve_iss_tle_lines()
    period_s = float(iss_orbital_period(time).to(u.s).value)
    eci_to_ecliptic = gcrs_to_mean_ecliptic_rotation(time)
    return {
        "name": "iss_orbit",
        "kind": "tle",
        "line1": line1,
        "line2": line2,
        "epoch_iso": time.iso,
        "period_s": period_s,
        "origin_body": "earth",
        "eci_to_ecliptic": eci_to_ecliptic.T.reshape(-1).astype(float).tolist(),
    }


def _serialize_keplerian(name: str, elements: KeplerianElements) -> dict[str, Any]:
    return {
        "name": name,
        "kind": "keplerian",
        "epoch_iso": elements["epoch_iso"],
        "period_days": elements["period_days"],
        "a_au": elements["a_au"],
        "e": elements["e"],
        "i_rad": elements["i_rad"],
        "raan_rad": elements["raan_rad"],
        "argp_rad": elements["argp_rad"],
        "M0_rad": elements["M0_rad"],
        "mu_au3_per_day2": elements["mu_au3_per_day2"],
        "origin_body": elements["origin_body"],
    }


def keplerian_heliocentric_position_au(
    elements: KeplerianElements,
    time: Time,
    earth_position_au: np.ndarray,
) -> np.ndarray:
    position = keplerian_position_au(elements, time)
    if elements["origin_body"] == "earth":
        return position + earth_position_au
    return position
