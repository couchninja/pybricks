"""Two-body Keplerian orbit propagation in heliocentric ecliptic AU and days."""

from __future__ import annotations

from typing import TypedDict

import numpy as np
from astropy import units as u
from astropy.constants import GM_earth
from astropy.time import Time

_SIDEREAL_YEAR_DAYS = 365.256363004

# μ in AU³ d⁻² so n = sqrt(μ/a³) with n in rad/day (two-body, day time base).
_SUN_MU_AU3_PER_DAY2 = (2.0 * np.pi / _SIDEREAL_YEAR_DAYS) ** 2

_EARTH_MU_AU3_PER_DAY2 = float(GM_earth.to(u.au**3 / u.day**2).value)


class KeplerianElements(TypedDict):
    epoch_iso: str
    period_days: float
    a_au: float
    e: float
    i_rad: float
    raan_rad: float
    argp_rad: float
    M0_rad: float
    mu_au3_per_day2: float
    origin_body: str | None


def keplerian_from_cartesian(
    position_au: np.ndarray,
    velocity_au_per_day: np.ndarray,
    *,
    mu_au3_per_day2: float,
    epoch: Time,
) -> KeplerianElements:
    r = np.asarray(position_au, dtype=float).reshape(3)
    v = np.asarray(velocity_au_per_day, dtype=float).reshape(3)
    mu = float(mu_au3_per_day2)

    h = np.cross(r, v)
    h_norm = float(np.linalg.norm(h))
    if h_norm == 0.0:
        raise ValueError("degenerate orbit: angular momentum is zero")

    n_vec = np.cross(np.array([0.0, 0.0, 1.0]), h)
    n_norm = float(np.linalg.norm(n_vec))

    r_norm = float(np.linalg.norm(r))
    v_norm = float(np.linalg.norm(v))
    e_vec = ((v_norm**2 - mu / r_norm) * r - np.dot(r, v) * v) / mu
    e = float(np.linalg.norm(e_vec))
    if e >= 1.0:
        raise ValueError(f"hyperbolic or parabolic orbit not supported: e={e}")

    energy = 0.5 * v_norm**2 - mu / r_norm
    a = -mu / (2.0 * energy)

    i = float(np.arccos(np.clip(h[2] / h_norm, -1.0, 1.0)))
    if n_norm > 0.0:
        raan = float(np.arccos(np.clip(n_vec[0] / n_norm, -1.0, 1.0)))
        if n_vec[1] < 0.0:
            raan = 2.0 * np.pi - raan
    else:
        raan = 0.0

    if n_norm > 0.0 and e > 0.0:
        argp = float(np.arccos(np.clip(np.dot(n_vec, e_vec) / (n_norm * e), -1.0, 1.0)))
        if e_vec[2] < 0.0:
            argp = 2.0 * np.pi - argp
    elif e > 0.0:
        argp = float(np.arccos(np.clip(e_vec[0] / e, -1.0, 1.0)))
        if e_vec[1] < 0.0:
            argp = 2.0 * np.pi - argp
    else:
        argp = 0.0

    if e > 0.0:
        nu = float(np.arccos(np.clip(np.dot(e_vec, r) / (e * r_norm), -1.0, 1.0)))
        if np.dot(r, v) < 0.0:
            nu = 2.0 * np.pi - nu
    else:
        nu = float(np.arctan2(r[1], r[0]))
        if nu < 0.0:
            nu += 2.0 * np.pi

    M0 = _true_anomaly_to_mean_anomaly(nu, e)
    period_days = float(2.0 * np.pi * np.sqrt(a**3 / mu))

    return KeplerianElements(
        epoch_iso=epoch.iso,
        period_days=period_days,
        a_au=float(a),
        e=e,
        i_rad=i,
        raan_rad=raan,
        argp_rad=argp,
        M0_rad=M0,
        mu_au3_per_day2=mu,
        origin_body=None,
    )


def keplerian_position_au(elements: KeplerianElements, time: Time) -> np.ndarray:
    epoch = Time(elements["epoch_iso"], format="iso", scale=time.scale)
    delta_days = (time - epoch).to(u.day).value
    mu = elements["mu_au3_per_day2"]
    a = elements["a_au"]
    e = elements["e"]
    n = np.sqrt(mu / a**3)
    M = elements["M0_rad"] + n * delta_days
    M = float(M % (2.0 * np.pi))
    nu = _mean_anomaly_to_true_anomaly(M, e)
    return _position_from_orbital_elements(
        a,
        e,
        elements["i_rad"],
        elements["raan_rad"],
        elements["argp_rad"],
        nu,
    )


def sample_keplerian_orbit_au(
    elements: KeplerianElements,
    *,
    samples: int,
    origin_au: np.ndarray | None = None,
) -> np.ndarray:
    if samples < 3:
        raise ValueError("samples must be at least 3")
    period_days = elements["period_days"]
    epoch = Time(elements["epoch_iso"], format="iso")
    times = epoch + np.linspace(0.0, period_days, samples, endpoint=False) * u.day
    points = np.array([keplerian_position_au(elements, Time(t)) for t in times], dtype=float)
    if origin_au is not None:
        origin = np.asarray(origin_au, dtype=float).reshape(3)
        points = points + origin
    return points


def _position_from_orbital_elements(
    a: float,
    e: float,
    i: float,
    raan: float,
    argp: float,
    nu: float,
) -> np.ndarray:
    p = a * (1.0 - e**2)
    r_norm = p / (1.0 + e * np.cos(nu))
    x_p = r_norm * np.cos(nu)
    y_p = r_norm * np.sin(nu)
    z_p = 0.0
    cos_raan, sin_raan = np.cos(raan), np.sin(raan)
    cos_i, sin_i = np.cos(i), np.sin(i)
    cos_argp, sin_argp = np.cos(argp), np.sin(argp)

    r1 = np.array([x_p, y_p, z_p])
    rot = np.array(
        [
            [
                cos_raan * cos_argp - sin_raan * sin_argp * cos_i,
                -cos_raan * sin_argp - sin_raan * cos_argp * cos_i,
                sin_raan * sin_i,
            ],
            [
                sin_raan * cos_argp + cos_raan * sin_argp * cos_i,
                -sin_raan * sin_argp + cos_raan * cos_argp * cos_i,
                -cos_raan * sin_i,
            ],
            [sin_argp * sin_i, cos_argp * sin_i, cos_i],
        ],
        dtype=float,
    )
    return rot @ r1


def _mean_anomaly_to_true_anomaly(M: float, e: float) -> float:
    E = _solve_kepler(M, e)
    sin_E = np.sin(E)
    cos_E = np.cos(E)
    sin_nu = np.sqrt(1.0 - e**2) * sin_E / (1.0 - e * cos_E)
    cos_nu = (cos_E - e) / (1.0 - e * cos_E)
    nu = float(np.arctan2(sin_nu, cos_nu))
    if nu < 0.0:
        nu += 2.0 * np.pi
    return nu


def _true_anomaly_to_mean_anomaly(nu: float, e: float) -> float:
    tan_half = np.tan(nu / 2.0)
    E = 2.0 * np.arctan(np.sqrt((1.0 - e) / (1.0 + e)) * tan_half)
    M = E - e * np.sin(E)
    if M < 0.0:
        M += 2.0 * np.pi
    return float(M)


def _solve_kepler(M: float, e: float, *, tolerance: float = 1e-12) -> float:
    M = float(M % (2.0 * np.pi))
    if e < 1e-10:
        return M
    E = M if e < 0.8 else np.pi
    for _ in range(64):
        delta = (E - e * np.sin(E) - M) / (1.0 - e * np.cos(E))
        E -= delta
        if abs(delta) < tolerance:
            break
    return float(E)


def sun_gravitational_parameter_au3_per_day2() -> float:
    return _SUN_MU_AU3_PER_DAY2


def earth_gravitational_parameter_au3_per_day2() -> float:
    return _EARTH_MU_AU3_PER_DAY2
