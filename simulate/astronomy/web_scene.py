"""JSON snapshots of the earth-sun scene for the web Three.js viewer."""

from __future__ import annotations

from time import perf_counter
from typing import Any

import numpy as np
import trimesh
from astropy.time import Time
from trimesh.transformations import transform_points

from simulate.astronomy.constants import (
    CAMERA_DISTANCE_EARTH_RADII,
    EARTH_ORBIT_RADIUS_AU,
    EARTH_RADIUS_AU,
    OBSERVER_VELOCITY_ARROW_LENGTH_CAMERA_DISTANCE_FRACTION,
    OBSERVER_VELOCITY_ARROW_LENGTH_EARTH_RADII,
    ROOT_FRAME,
    SKYBOX_FADE_CAMERA_DISTANCE_ORBIT_MULTIPLE,
    SKYBOX_FADE_SPAN_ORBIT_RADIUS_MULTIPLE,
    PointingTarget,
)
from simulate.astronomy.earth_sun_scene import (
    EarthSunAnimationState,
    build_earth_sun_scene,
    update_earth_sun_scene,
)
from simulate.astronomy.simulation_clock import (
    reanchor_wall_clock,
    reset_simulation_clock,
    simulation_time,
    time_scaling,
)
from simulate.astronomy.utils.camera import camera_clip_planes
from simulate.astronomy.utils.ephemeris import current_time, ecliptic_to_galactocentric_rotation
from simulate.astronomy.utils.iss import refresh_iss_tle
from simulate.astronomy.utils.iss_tle import ISS_TLE_REFRESH_INTERVAL_S

_BODY_NODES: tuple[tuple[str, str], ...] = (
    ("sun", "Sun"),
    ("earth", "Earth"),
    ("moon", "Moon"),
    ("iss", "ISS"),
    ("observer", "Observer"),
    ("galactic_center", "Milky Way center"),
)

_PATH_NODES: tuple[str, ...] = (
    "earth_orbit",
    "moon_orbit",
    "iss_orbit",
    "year_boundaries",
    "galactic_orbit",
    "earth_axis",
    "galactic_axis",
)

_ARROW_NODES: tuple[str, ...] = (
    "observer_velocity_arrow",
    "cmb_dipole_arrow",
)


class _WebSceneCache:
    scene: trimesh.Scene | None = None
    animation: EarthSunAnimationState | None = None


_cache = _WebSceneCache()


def reset_web_scene_cache() -> None:
    _cache.scene = None
    _cache.animation = None
    reset_simulation_clock()


def scene_snapshot_payload(pointing_target: PointingTarget) -> dict[str, Any]:
    scene = _ensure_scene()
    scene.metadata["pointing_target"] = pointing_target
    _advance_animation(scene)
    return _serialize_scene(scene, pointing_target)


def _ensure_scene() -> trimesh.Scene:
    if _cache.scene is None:
        start = simulation_time()
        scene = build_earth_sun_scene(start)
        reanchor_wall_clock()
        start = simulation_time()
        scene.metadata["earth_sun_animation"] = EarthSunAnimationState(
            start_time=start,
            last_orbit_time=None,
            last_iss_tle_refresh=perf_counter(),
            time_scaling=time_scaling(),
            wall_start=None,
            current_time=start,
        )
        _cache.scene = scene
        _cache.animation = scene.metadata["earth_sun_animation"]
        return scene

    return _cache.scene


def _advance_animation(scene: trimesh.Scene) -> None:
    animation = scene.metadata["earth_sun_animation"]
    if animation["wall_start"] is None:
        animation["wall_start"] = perf_counter()
    now = perf_counter()
    last_iss_tle_refresh = animation["last_iss_tle_refresh"]
    if last_iss_tle_refresh is None or now - last_iss_tle_refresh >= ISS_TLE_REFRESH_INTERVAL_S:
        refresh_iss_tle()
        animation["last_iss_tle_refresh"] = now
    time = simulation_time()
    animation["current_time"] = time
    animation["time_scaling"] = time_scaling()
    camera_distance = CAMERA_DISTANCE_EARTH_RADII * EARTH_RADIUS_AU
    animation["last_orbit_time"] = update_earth_sun_scene(
        scene,
        time,
        animation["last_orbit_time"],
        camera_distance,
    )


def _serialize_scene(scene: trimesh.Scene, pointing_target: PointingTarget) -> dict[str, Any]:
    animation = scene.metadata["earth_sun_animation"]
    current = animation["current_time"]
    time_iso = current.iso if isinstance(current, Time) else current_time().iso
    default_camera_distance = CAMERA_DISTANCE_EARTH_RADII * EARTH_RADIUS_AU
    z_near, z_far = camera_clip_planes(scene, camera_distance=default_camera_distance)
    return {
        "time_iso": time_iso,
        "pointing_target": pointing_target.value,
        "pointing_target_label": pointing_target.label,
        "scene_scale": float(scene.scale),
        "camera_distance_au": default_camera_distance,
        "default_camera_distance_au": default_camera_distance,
        "earth_radius_au": EARTH_RADIUS_AU,
        "earth_orbit_radius_au": EARTH_ORBIT_RADIUS_AU,
        "skybox_fade_camera_distance_orbit_multiple": SKYBOX_FADE_CAMERA_DISTANCE_ORBIT_MULTIPLE,
        "skybox_fade_span_orbit_radius_multiple": SKYBOX_FADE_SPAN_ORBIT_RADIUS_MULTIPLE,
        "arrow_mesh_length_au": OBSERVER_VELOCITY_ARROW_LENGTH_EARTH_RADII * EARTH_RADIUS_AU,
        "arrow_length_camera_distance_fraction": OBSERVER_VELOCITY_ARROW_LENGTH_CAMERA_DISTANCE_FRACTION,
        "z_near": z_near,
        "z_far": z_far,
        "inertial_to_root_rotation": _rotation_to_three(
            ecliptic_to_galactocentric_rotation(current),
        ),
        "milky_way_diameter_au": _milky_way_diameter_au(scene),
        "bodies": [_serialize_body(scene, node, label) for node, label in _BODY_NODES],
        "paths": [
            _serialize_path(scene, node)
            for node in _PATH_NODES
            if node != "iss_orbit" or pointing_target == PointingTarget.ISS
        ],
        "arrows": [
            _serialize_arrow(scene, node)
            for node in _ARROW_NODES
            if node != "cmb_dipole_arrow" or pointing_target == PointingTarget.CMB_DIPOLE
        ],
    }


def _serialize_body(scene: trimesh.Scene, node_name: str, label: str) -> dict[str, Any]:
    transform, geometry_name = scene.graph.get(node_name, ROOT_FRAME)
    mesh = scene.geometry[geometry_name]
    # bounding_sphere integrates volume; tiny AU-scale markers can divide by zero.
    radius = float(np.max(mesh.extents) / 2.0)
    color = _mesh_color(mesh)
    return {
        "name": node_name,
        "label": label,
        "radius": radius,
        "color": color,
        "matrix": _matrix_to_three(transform),
    }


def _serialize_path(scene: trimesh.Scene, node_name: str) -> dict[str, Any]:
    transform, geometry_name = scene.graph.get(node_name, ROOT_FRAME)
    geometry = scene.geometry[geometry_name]
    color = _path_color(geometry)
    segments = _path_segments_world(geometry, transform)
    return {
        "name": node_name,
        "color": color,
        "segments": segments,
    }


def _serialize_arrow(scene: trimesh.Scene, node_name: str) -> dict[str, Any]:
    transform, geometry_name = scene.graph.get(node_name, ROOT_FRAME)
    mesh = scene.geometry[geometry_name]
    color = _mesh_color(mesh)
    base = transform[:3, 3]
    z_axis = transform[:3, 2]
    z_length = float(np.linalg.norm(z_axis))
    if z_length > 0.0:
        direction = (z_axis / z_length).astype(float)
    else:
        direction = np.array([0.0, 0.0, 1.0], dtype=float)
    distance_anchor = "galactic_center" if node_name == "cmb_dipole_arrow" else "observer"
    return {
        "name": node_name,
        "color": color,
        "base": base.astype(float).tolist(),
        "direction": direction.tolist(),
        "distance_anchor": distance_anchor,
    }


def _mesh_color(mesh: trimesh.Trimesh) -> list[int]:
    colors = mesh.visual.face_colors
    if len(colors) == 0:
        return [200, 200, 200]
    channel = colors[0]
    return [int(channel[0]), int(channel[1]), int(channel[2])]


def _path_color(geometry: trimesh.path.Path3D) -> list[int]:
    if geometry.colors is None or len(geometry.colors) == 0:
        return [200, 200, 200]
    channel = geometry.colors[0]
    return [int(channel[0]), int(channel[1]), int(channel[2])]


def _path_segments_world(geometry: trimesh.path.Path3D, transform: np.ndarray) -> list[list[list[float]]]:
    segments: list[list[list[float]]] = []
    for entity in geometry.entities:
        points = geometry.vertices[entity.points]
        if len(points) < 2:
            continue
        world = transform_points(points, transform)
        segments.append(world.astype(float).tolist())
    return segments


def _matrix_to_three(matrix: np.ndarray) -> list[float]:
    return matrix.T.reshape(-1).astype(float).tolist()


def _rotation_to_three(rotation: np.ndarray) -> list[float]:
    return rotation.T.reshape(-1).astype(float).tolist()


def _milky_way_diameter_au(scene: trimesh.Scene) -> float:
    gc_transform, _ = scene.graph.get("galactic_center", ROOT_FRAME)
    galactic_center = gc_transform[:3, 3]
    orbit_transform, orbit_geometry_name = scene.graph.get("galactic_orbit", ROOT_FRAME)
    orbit = scene.geometry[orbit_geometry_name]
    orbit_world = transform_points(orbit.vertices, orbit_transform)
    orbit_radius = float(np.max(np.linalg.norm(orbit_world - galactic_center, axis=1)))
    return 2.0 * orbit_radius
