import json

import numpy as np
from astropy import units as u
from astropy.coordinates import SkyCoord

from gpio.button_config import pointing_target_button_rgba
from simulate.astronomy.constants import PointingTarget
from simulate.astronomy.web_scene import reset_web_scene_cache, scene_snapshot_payload


def test_scene_snapshot_json_serializable() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.EARTH_ROTATION)
    json.dumps(payload)
    body_names = {body["name"] for body in payload["bodies"]}
    assert "earth" in body_names
    assert "sun" in body_names
    assert "observer" in body_names
    assert payload["time_iso"]
    path_names = {path["name"] for path in payload["paths"]}
    assert "earth_orbit" in path_names
    assert "moon_orbit" in path_names
    assert "iss_orbit" not in path_names
    arrow_names = {arrow["name"] for arrow in payload["arrows"]}
    assert "observer_velocity_arrow" in arrow_names
    assert "cmb_dipole_arrow" not in arrow_names


def test_cmb_dipole_arrow_only_when_pointing_at_cmb_dipole() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.CMB_DIPOLE)
    arrow_names = {arrow["name"] for arrow in payload["arrows"]}
    assert "cmb_dipole_arrow" in arrow_names


def test_moon_orbit_loops_near_earth() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.MOON)
    earth = next(body for body in payload["bodies"] if body["name"] == "earth")
    moon = next(body for body in payload["bodies"] if body["name"] == "moon")
    moon_orbit = next(path for path in payload["paths"] if path["name"] == "moon_orbit")
    earth_position = np.array(earth["matrix"], dtype=float).reshape(4, 4).T[:3, 3]
    moon_position = np.array(moon["matrix"], dtype=float).reshape(4, 4).T[:3, 3]
    segment = moon_orbit["segments"][0]
    orbit_points = np.array(segment, dtype=float)
    distances_from_earth = np.linalg.norm(orbit_points - earth_position, axis=1)
    mean_distance = float(np.mean(distances_from_earth))
    assert 0.002 < mean_distance < 0.003
    moon_on_orbit = float(np.min(np.linalg.norm(orbit_points - moon_position, axis=1)))
    assert moon_on_orbit < 1e-6


def test_iss_orbit_loops_near_earth() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.ISS)
    earth = next(body for body in payload["bodies"] if body["name"] == "earth")
    iss = next(body for body in payload["bodies"] if body["name"] == "iss")
    iss_orbit = next(path for path in payload["paths"] if path["name"] == "iss_orbit")
    earth_position = np.array(earth["matrix"], dtype=float).reshape(4, 4).T[:3, 3]
    iss_position = np.array(iss["matrix"], dtype=float).reshape(4, 4).T[:3, 3]
    segment = iss_orbit["segments"][0]
    orbit_points = np.array(segment, dtype=float)
    distances_from_earth = np.linalg.norm(orbit_points - earth_position, axis=1)
    mean_distance = float(np.mean(distances_from_earth))
    assert 0.00004 < mean_distance < 0.0002
    iss_on_orbit = float(np.min(np.linalg.norm(orbit_points - iss_position, axis=1)))
    assert iss_on_orbit < 1e-6


def test_scene_snapshot_follows_pointing_target() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.MOON)
    assert payload["pointing_target"] == PointingTarget.MOON.value
    assert payload["pointing_target_label"] == PointingTarget.MOON.label


def test_observer_arrow_color_matches_button_for_target() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.SUN)
    observer_arrow = next(a for a in payload["arrows"] if a["name"] == "observer_velocity_arrow")
    expected = pointing_target_button_rgba(PointingTarget.SUN)
    assert observer_arrow["color"] == expected[:3]


def test_inertial_to_root_rotation_aligns_galactic_center_with_sagittarius() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.MILKY_WAY_CENTER)
    rotation = np.array(payload["inertial_to_root_rotation"], dtype=float).reshape(3, 3).T
    galactic_center = next(body for body in payload["bodies"] if body["name"] == "galactic_center")
    marker_direction = np.array(galactic_center["matrix"], dtype=float).reshape(4, 4).T[:3, 3]
    marker_direction /= np.linalg.norm(marker_direction)

    sgr_a = SkyCoord(ra=266.416 * u.deg, dec=-29.0078 * u.deg, frame="icrs").transform_to(
        "geocentricmeanecliptic",
    )
    ecliptic_direction = np.array(sgr_a.cartesian.xyz.value, dtype=float)
    ecliptic_direction /= np.linalg.norm(ecliptic_direction)

    sky_direction = rotation @ ecliptic_direction
    assert np.dot(sky_direction, marker_direction) > 0.999


def test_milky_way_diameter_bounds_galactic_orbit() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.EARTH_ROTATION)
    diameter = payload["milky_way_diameter_au"]
    assert diameter > payload["default_camera_distance_au"]
    galactic_center = next(body for body in payload["bodies"] if body["name"] == "galactic_center")
    gc_position = np.array(galactic_center["matrix"], dtype=float).reshape(4, 4).T[:3, 3]
    gc_distance = float(np.linalg.norm(gc_position))
    assert diameter >= 2 * gc_distance * 0.99


def test_scene_snapshot_includes_skybox_fade_metadata() -> None:
    reset_web_scene_cache()
    from simulate.astronomy.constants import (
        EARTH_ORBIT_RADIUS_AU,
        SKYBOX_FADE_CAMERA_DISTANCE_ORBIT_MULTIPLE,
        SKYBOX_FADE_SPAN_ORBIT_RADIUS_MULTIPLE,
    )

    payload = scene_snapshot_payload(PointingTarget.EARTH_ROTATION)
    assert payload["earth_orbit_radius_au"] == EARTH_ORBIT_RADIUS_AU
    assert payload["skybox_fade_camera_distance_orbit_multiple"] == SKYBOX_FADE_CAMERA_DISTANCE_ORBIT_MULTIPLE
    assert payload["skybox_fade_span_orbit_radius_multiple"] == SKYBOX_FADE_SPAN_ORBIT_RADIUS_MULTIPLE


def test_scene_snapshot_includes_arrow_mesh_length() -> None:
    reset_web_scene_cache()
    from simulate.astronomy.constants import EARTH_RADIUS_AU, OBSERVER_VELOCITY_ARROW_LENGTH_EARTH_RADII

    payload = scene_snapshot_payload(PointingTarget.EARTH_ROTATION)
    assert payload["arrow_mesh_length_au"] == OBSERVER_VELOCITY_ARROW_LENGTH_EARTH_RADII * EARTH_RADIUS_AU
    observer_arrow = next(a for a in payload["arrows"] if a["name"] == "observer_velocity_arrow")
    assert observer_arrow["distance_anchor"] == "observer"
    assert len(observer_arrow["base"]) == 3
    assert len(observer_arrow["direction"]) == 3


def test_observer_velocity_arrow_shaft_starts_beyond_observer_marker() -> None:
    reset_web_scene_cache()
    from simulate.astronomy.constants import OBSERVER_VELOCITY_ARROW_SHAFT_START_OBSERVER_RADIUS_MULTIPLE

    payload = scene_snapshot_payload(PointingTarget.EARTH_ROTATION)
    observer = next(body for body in payload["bodies"] if body["name"] == "observer")
    observer_arrow = next(a for a in payload["arrows"] if a["name"] == "observer_velocity_arrow")
    observer_position = np.array(observer["matrix"], dtype=float).reshape(4, 4).T[:3, 3]
    arrow_base = np.array(observer_arrow["base"], dtype=float)
    gap = np.linalg.norm(arrow_base - observer_position)
    expected_gap = OBSERVER_VELOCITY_ARROW_SHAFT_START_OBSERVER_RADIUS_MULTIPLE * observer["radius"]
    assert np.isclose(gap, expected_gap, rtol=1e-6, atol=2e-11)
