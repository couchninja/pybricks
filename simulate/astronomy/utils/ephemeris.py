import numpy as np
from astropy import units as u
from astropy.coordinates import (
    GCRS,
    ITRS,
    BarycentricMeanEcliptic,
    CartesianRepresentation,
    EarthLocation,
    Galactic,
    Galactocentric,
    SkyCoord,
    get_body_barycentric,
)
from astropy.time import Time
from astropy.utils.iers import conf as iers_conf
from scipy.spatial.transform import Rotation

from simulate.astronomy.constants import (
    CMB_DIPOLE_B,
    CMB_DIPOLE_L,
    CMB_DIPOLE_SPEED,
    EARTH_RADIUS_AU,
    KPC_TO_AU,
    OBSERVER_LAT,
    OBSERVER_LON,
    SIDEREAL_DAY,
    SOLAR_GALACTIC_ORBITAL_SPEED,
    PointingTarget,
)

# Keep Astropy's default IERS auto-download when online. Offline (or when the
# download fails), the probe raises ValueError once predictive data is older
# than auto_max_age; allow the bundled table instead of crashing.
try:
    Time.now().ut1
except ValueError:
    iers_conf.auto_max_age = None


_BODY_POINTING_TARGETS = frozenset({PointingTarget.SUN, PointingTarget.MOON, PointingTarget.MILKY_WAY_CENTER})


def earth_orientation_matrix(time: Time) -> np.ndarray:
    spin_axis = earth_spin_axis_ecliptic(time)
    observer_dir = observer_direction_ecliptic(time)
    observer_local = observer_direction_itrs(time)
    rotation = Rotation.align_vectors(
        [spin_axis, observer_dir],
        [np.array([0.0, 0.0, 1.0]), observer_local],
    )[0]
    return rotation.as_matrix()


def earth_orbit_ecliptic_au(time: Time, samples: int = 360) -> np.ndarray:
    times = time + np.linspace(-0.5, 0.5, samples, endpoint=False) * u.year
    earth = get_body_barycentric("earth", times)
    sun = get_body_barycentric("sun", times)
    relative = SkyCoord(
        earth - sun,
        representation_type="cartesian",
        frame="icrs",
    ).transform_to(BarycentricMeanEcliptic())
    cartesian = relative.cartesian
    return np.column_stack(
        [
            cartesian.x.to_value(u.au),
            cartesian.y.to_value(u.au),
            cartesian.z.to_value(u.au),
        ]
    )


def earth_year_boundary_positions_ecliptic_au(time: Time) -> np.ndarray:
    """Earth positions at calendar Jan 1 dates within ±0.5 Julian years of ``time``.

    Returns one heliocentric ecliptic position (AU) per Jan 1 in
    ``[time - 0.5 year, time + 0.5 year)``. Usually exactly one; occasionally
    zero for a narrow band around mid-year. Julian years are 365.25 days while
    calendar years are 365/366 days, so the half-open window can fall strictly
    between two consecutive Jan 1 instants (both excluded by ``start <= boundary
    < end``).
    """
    start = time - 0.5 * u.year
    end = time + 0.5 * u.year
    start_year = int(np.floor(start.decimalyear))
    end_year = int(np.ceil(end.decimalyear))
    positions = []
    for year in range(start_year, end_year + 1):
        boundary = Time(f"{year}-01-01", scale=time.scale, format="iso")
        if start <= boundary < end:
            positions.append(earth_heliocentric_ecliptic_au(boundary))
    return np.array(positions, dtype=float)


def earth_heliocentric_ecliptic_au(time: Time) -> np.ndarray:
    earth = get_body_barycentric("earth", time)
    sun = get_body_barycentric("sun", time)
    return _heliocentric_ecliptic_au(earth, sun)


def moon_heliocentric_ecliptic_au(time: Time) -> np.ndarray:
    moon = get_body_barycentric("moon", time)
    sun = get_body_barycentric("sun", time)
    return _heliocentric_ecliptic_au(moon, sun)


def sun_galactocentric_kpc(time: Time) -> np.ndarray:
    sun = get_body_barycentric("sun", time)
    galactic = SkyCoord(
        sun,
        representation_type="cartesian",
        frame="icrs",
    ).transform_to(Galactocentric())
    cartesian = galactic.cartesian
    return np.array(
        [
            cartesian.x.to_value(u.kpc),
            cartesian.y.to_value(u.kpc),
            cartesian.z.to_value(u.kpc),
        ],
        dtype=float,
    )


def sun_galactic_orbit_kpc(time: Time, samples: int = 360) -> np.ndarray:
    sun_position = sun_galactocentric_kpc(time)
    radius = np.hypot(sun_position[0], sun_position[1])
    theta = np.linspace(0, 2 * np.pi, samples, endpoint=False)
    return np.column_stack(
        [
            radius * np.cos(theta),
            radius * np.sin(theta),
            np.zeros(samples, dtype=float),
        ]
    )


def ecliptic_to_galactocentric_rotation(time: Time) -> np.ndarray:
    return np.column_stack(
        [
            _ecliptic_direction_galactocentric(np.array([1.0, 0.0, 0.0]), time),
            _ecliptic_direction_galactocentric(np.array([0.0, 1.0, 0.0]), time),
            _ecliptic_direction_galactocentric(np.array([0.0, 0.0, 1.0]), time),
        ]
    )


def earth_spin_axis_ecliptic(time: Time) -> np.ndarray:
    return _gcrs_unit_vector_to_ecliptic(earth_spin_axis_gcrs(time), time)


def observer_direction_ecliptic(time: Time) -> np.ndarray:
    return _gcrs_unit_vector_to_ecliptic(observer_direction_gcrs(time), time)


def ecliptic_to_observer_surface(vector: np.ndarray, time: Time) -> np.ndarray:
    """Express an ecliptic vector in the observer's local surface frame.

    Components:
      x — north (tangent to the surface toward the north celestial pole)
      y — east (tangent to the surface, geographic east)
      z — up (local zenith / outward radial from Earth's center)
    """
    up = observer_direction_ecliptic(time)
    east = np.cross(earth_spin_axis_ecliptic(time), up)
    east /= np.linalg.norm(east)
    north = np.cross(up, east)
    return np.array(
        [np.dot(vector, north), np.dot(vector, east), np.dot(vector, up)],
        dtype=float,
    )


def observer_surface_euler_angles(surface_vector: np.ndarray) -> np.ndarray:
    """Euler angles (yaw, pitch, roll) in radians for a surface-frame direction.

    yaw — heading from north toward east (0 = north, π/2 = east, ±π = south);
          range (-π, π]
    pitch — elevation relative to the horizon (0 = horizon, π/2 = zenith,
            -π/2 = nadir); range [-π/2, π/2]
    roll — always 0 (a direction does not determine roll)
    """
    north, east, up = surface_vector
    yaw = float(np.atan2(east, north))
    pitch = float(np.atan2(up, np.hypot(north, east)))
    return np.array([yaw, pitch, 0.0], dtype=float)


def earth_spin_axis_gcrs(time: Time) -> np.ndarray:
    north_pole = ITRS(x=0 * u.m, y=0 * u.m, z=1 * u.m, obstime=time)
    gcrs = north_pole.transform_to(GCRS(obstime=time))
    direction = np.array(gcrs.cartesian.xyz.value, dtype=float)
    return direction / np.linalg.norm(direction)


def observer_direction_gcrs(time: Time) -> np.ndarray:
    location = EarthLocation.from_geodetic(
        lat=OBSERVER_LAT,
        lon=OBSERVER_LON,
    )
    gcrs = location.get_gcrs(time)
    direction = np.array(gcrs.cartesian.xyz.value, dtype=float)
    return direction / np.linalg.norm(direction)


def observer_direction_itrs(time: Time) -> np.ndarray:
    location = EarthLocation.from_geodetic(
        lat=OBSERVER_LAT,
        lon=OBSERVER_LON,
    )
    itrs = location.get_itrs(time)
    direction = np.array(itrs.cartesian.xyz.value, dtype=float)
    return direction / np.linalg.norm(direction)


def current_time() -> Time:
    return Time.now()


def sun_direction_ecliptic_from_observer(time: Time) -> np.ndarray:
    return _body_direction_ecliptic_from_observer(np.zeros(3, dtype=float), time)


def moon_direction_ecliptic_from_observer(time: Time) -> np.ndarray:
    return _body_direction_ecliptic_from_observer(moon_heliocentric_ecliptic_au(time), time)


def galactic_center_direction_ecliptic_from_observer(time: Time) -> np.ndarray:
    return _body_direction_ecliptic_from_observer(_galactic_center_heliocentric_ecliptic_au(time), time)


def observer_direction_ecliptic_for_target(time: Time, target: PointingTarget) -> np.ndarray | None:
    if target == PointingTarget.SUN:
        return sun_direction_ecliptic_from_observer(time)
    if target == PointingTarget.MOON:
        return moon_direction_ecliptic_from_observer(time)
    if target == PointingTarget.MILKY_WAY_CENTER:
        return galactic_center_direction_ecliptic_from_observer(time)
    velocity = observer_velocity_ecliptic_au_per_s(time, target)
    speed = float(np.linalg.norm(velocity))
    if speed == 0.0:
        return None
    return velocity / speed


def observer_surface_vector_and_euler_angles_for_target(
    time: Time, pointing_target: PointingTarget
) -> tuple[np.ndarray, np.ndarray, float]:
    """Surface-frame direction and heading for a pointing target.

    Uses ``observer_direction_ecliptic_for_target`` (body/sky direction for
    ``PointingTarget.SUN``, ``PointingTarget.MOON``, and
    ``PointingTarget.MILKY_WAY_CENTER``, otherwise normalized ecliptic velocity)
    and expresses that direction in the observer's local surface frame (see
    ``ecliptic_to_observer_surface``).

    Returns ``(surface_vector, euler_angles, speed)``:

    surface_vector
      Components of the direction in the surface frame:
        x — north, y — east, z — up (local zenith).
    euler_angles
      ``[yaw, pitch, roll]`` in degrees derived from ``surface_vector``:
        yaw — heading from north toward east (0 = north, 90 = east, ±180 = south);
              range (-180, 180]
        pitch — elevation relative to the horizon (0 = horizon, 90 = zenith,
                -90 = nadir); range [-90, 90]
        roll — always 0 (a direction does not determine roll)
    speed
      Speed of the ecliptic velocity vector in AU/s (0 for body/sky targets
      ``PointingTarget.SUN``, ``PointingTarget.MOON``, and
      ``PointingTarget.MILKY_WAY_CENTER``).

    When no direction is defined, ``surface_vector`` and ``euler_angles`` are both zero.
    """
    direction = observer_direction_ecliptic_for_target(time, pointing_target)
    if direction is None:
        return np.zeros(3, dtype=float), np.zeros(3, dtype=float), 0.0
    surface_vector = ecliptic_to_observer_surface(direction, time)
    euler_angles = np.degrees(observer_surface_euler_angles(surface_vector))
    if pointing_target in _BODY_POINTING_TARGETS:
        speed = 0.0
    else:
        speed = float(np.linalg.norm(observer_velocity_ecliptic_au_per_s(time, pointing_target)))
    return surface_vector, euler_angles, speed


def sun_or_moon_pointing_target(time: Time) -> PointingTarget:
    _surface, (_yaw, pitch, _roll), _speed = observer_surface_vector_and_euler_angles_for_target(
        time, PointingTarget.SUN
    )
    if pitch > 0:
        return PointingTarget.SUN
    return PointingTarget.MOON


def observer_velocity_ecliptic_au_per_s(time: Time, target: PointingTarget) -> np.ndarray:
    if target not in PointingTarget:
        raise ValueError(f"unknown pointing target: {target}")

    velocity = np.zeros(3, dtype=float)
    if target in (
        PointingTarget.EARTH_ROTATION,
        PointingTarget.SUN_ORBIT,
        PointingTarget.MILKY_WAY_ORBIT,
        PointingTarget.CMB_DIPOLE,
    ):
        velocity += _earth_rotation_velocity_ecliptic_au_per_s(time)
    if target in (
        PointingTarget.SUN_ORBIT,
        PointingTarget.MILKY_WAY_ORBIT,
        PointingTarget.CMB_DIPOLE,
    ):
        velocity += _earth_orbital_velocity_ecliptic_au_per_s(time)
    if target in (PointingTarget.MILKY_WAY_ORBIT, PointingTarget.CMB_DIPOLE):
        velocity += _sun_galactic_orbital_velocity_ecliptic_au_per_s(time)
    if target == PointingTarget.CMB_DIPOLE:
        velocity += _cmb_dipole_velocity_ecliptic_au_per_s(time)
    return velocity


def _galactic_center_heliocentric_ecliptic_au(time: Time) -> np.ndarray:
    sun_galactic = sun_galactocentric_kpc(time)
    galactic_rotation = ecliptic_to_galactocentric_rotation(time)
    return galactic_rotation.T @ (-sun_galactic) * KPC_TO_AU


def _body_direction_ecliptic_from_observer(body_heliocentric_au: np.ndarray, time: Time) -> np.ndarray:
    earth_position = earth_heliocentric_ecliptic_au(time)
    observer_dir = observer_direction_ecliptic(time)
    observer_position = earth_position + observer_dir * EARTH_RADIUS_AU
    direction = body_heliocentric_au - observer_position
    return direction / np.linalg.norm(direction)


def _heliocentric_ecliptic_au(earth: CartesianRepresentation, sun: CartesianRepresentation) -> np.ndarray:
    relative = SkyCoord(
        earth - sun,
        representation_type="cartesian",
        frame="icrs",
    ).transform_to(BarycentricMeanEcliptic())
    cartesian = relative.cartesian
    return np.array(
        [
            cartesian.x.to_value(u.au),
            cartesian.y.to_value(u.au),
            cartesian.z.to_value(u.au),
        ],
        dtype=float,
    )


def _ecliptic_direction_galactocentric(vector: np.ndarray, time: Time) -> np.ndarray:
    sun_galactic = sun_galactocentric_kpc(time)
    ecliptic = BarycentricMeanEcliptic(
        x=vector[0] * u.kpc,
        y=vector[1] * u.kpc,
        z=vector[2] * u.kpc,
        representation_type="cartesian",
    )
    gcrs = ecliptic.transform_to(GCRS(obstime=time))
    galactic = SkyCoord(gcrs, representation_type="cartesian").transform_to(Galactocentric())
    direction = np.array(galactic.cartesian.xyz.to_value(u.kpc), dtype=float)
    direction -= sun_galactic
    return direction / np.linalg.norm(direction)


def _earth_rotation_velocity_ecliptic_au_per_s(time: Time) -> np.ndarray:
    spin_axis = earth_spin_axis_ecliptic(time)
    observer_dir = observer_direction_ecliptic(time)
    angular_speed = (2 * np.pi / SIDEREAL_DAY).to_value(1 / u.s)
    omega = angular_speed * spin_axis
    radius = EARTH_RADIUS_AU * observer_dir
    return np.cross(omega, radius)


def _earth_orbital_velocity_ecliptic_au_per_s(time: Time) -> np.ndarray:
    delta = 1 * u.hour
    position_before = earth_heliocentric_ecliptic_au(time - delta)
    position_after = earth_heliocentric_ecliptic_au(time + delta)
    return (position_after - position_before) / (2 * delta.to(u.s).value)


def _sun_galactic_orbital_velocity_ecliptic_au_per_s(time: Time) -> np.ndarray:
    sun_galactic = sun_galactocentric_kpc(time)
    radius_xy = np.hypot(sun_galactic[0], sun_galactic[1])
    tangent = (
        np.array(
            [-sun_galactic[1], sun_galactic[0], 0.0],
            dtype=float,
        )
        / radius_xy
    )
    speed_kpc_per_s = SOLAR_GALACTIC_ORBITAL_SPEED.to_value(u.kpc / u.s)
    galactic_velocity_au_per_s = tangent * speed_kpc_per_s * KPC_TO_AU
    galactic_rotation = ecliptic_to_galactocentric_rotation(time)
    return galactic_rotation.T @ galactic_velocity_au_per_s


def cmb_dipole_direction_galactocentric(time: Time) -> np.ndarray:
    direction = SkyCoord(
        l=CMB_DIPOLE_L,
        b=CMB_DIPOLE_B,
        distance=1 * u.kpc,
        frame=Galactic,
    )
    galactocentric = direction.transform_to(Galactocentric())
    unit = np.array(galactocentric.cartesian.xyz.value, dtype=float)
    return unit / np.linalg.norm(unit)


def _cmb_dipole_velocity_ecliptic_au_per_s(time: Time) -> np.ndarray:
    direction = cmb_dipole_direction_galactocentric(time)
    galactic_rotation = ecliptic_to_galactocentric_rotation(time)
    ecliptic_direction = galactic_rotation.T @ direction
    return ecliptic_direction * CMB_DIPOLE_SPEED.to_value(u.au / u.s)


def _gcrs_unit_vector_to_ecliptic(vector: np.ndarray, time: Time) -> np.ndarray:
    gcrs = GCRS(
        x=vector[0] * u.one,
        y=vector[1] * u.one,
        z=vector[2] * u.one,
        obstime=time,
        representation_type="cartesian",
    )
    ecliptic = gcrs.transform_to(BarycentricMeanEcliptic())
    direction = np.array(ecliptic.cartesian.xyz.value, dtype=float)
    return direction / np.linalg.norm(direction)
