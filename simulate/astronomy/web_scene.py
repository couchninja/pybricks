"""JSON snapshots of the earth-sun scene for the web Three.js viewer."""

from __future__ import annotations

from typing import Any

import numpy as np
import trimesh
from trimesh.visual.color import ColorVisuals

from simulate.astronomy.constants import (
    CAMERA_DISTANCE_EARTH_RADII,
    EARTH_RADIUS_AU,
    ROOT_FRAME,
    PointingTarget,
)
from simulate.astronomy.earth_sun_scene import (
    EarthSunAnimationState,
    build_earth_sun_scene,
    update_earth_sun_scene,
)
from simulate.astronomy.parametric_orbits import parametric_orbit_payloads
from simulate.astronomy.simulation_clock import (
    reanchor_wall_clock,
    reset_simulation_clock,
    simulation_time,
    time_scaling,
)
from simulate.astronomy.utils.ephemeris import earth_heliocentric_ecliptic_au, ecliptic_to_galactocentric_rotation

_BODY_NODES: tuple[str, ...] = (
    "sun",
    "earth",
    "moon",
    "iss",
    "observer",
    "galactic_center",
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

_EPHEMERIS_ORBIT_PATHS = frozenset(
    {"earth_orbit", "moon_orbit", "iss_orbit", "galactic_orbit"},
)


_cache_scene: trimesh.Scene | None = None


def reset_web_scene_cache() -> None:
    global _cache_scene
    _cache_scene = None
    reset_simulation_clock()


def scene_snapshot_payload(
    pointing_target: PointingTarget,
    *,
    include_ephemeris_orbit_paths: bool = True,
) -> dict[str, Any]:
    scene = _ensure_scene()
    scene.metadata["pointing_target"] = pointing_target
    _advance_animation(scene)
    return _serialize_scene(scene, pointing_target, include_ephemeris_orbit_paths=include_ephemeris_orbit_paths)


def _ensure_scene() -> trimesh.Scene:
    global _cache_scene
    if _cache_scene is None:
        start = simulation_time()
        scene = build_earth_sun_scene(start)
        reanchor_wall_clock()
        start = simulation_time()
        scene.metadata["earth_sun_animation"] = EarthSunAnimationState(
            last_orbit_time=None,
            last_moon_orbit_time=None,
            last_iss_orbit_time=None,
            time_scaling=time_scaling(),
            current_time=start,
        )
        _cache_scene = scene
        return scene

    return _cache_scene


def _advance_animation(scene: trimesh.Scene) -> None:
    animation = scene.metadata["earth_sun_animation"]
    time = simulation_time()
    animation["current_time"] = time
    animation["time_scaling"] = time_scaling()
    camera_distance = CAMERA_DISTANCE_EARTH_RADII * EARTH_RADIUS_AU
    last_orbit_time, last_moon_orbit_time, last_iss_orbit_time = update_earth_sun_scene(
        scene,
        time,
        animation["last_orbit_time"],
        camera_distance,
        last_moon_orbit_time=animation["last_moon_orbit_time"],
        last_iss_orbit_time=animation["last_iss_orbit_time"],
    )
    animation["last_orbit_time"] = last_orbit_time
    animation["last_moon_orbit_time"] = last_moon_orbit_time
    animation["last_iss_orbit_time"] = last_iss_orbit_time


def _include_path_in_snapshot(
    node_name: str,
    pointing_target: PointingTarget,
    include_ephemeris_orbit_paths: bool,
) -> bool:
    if node_name in _EPHEMERIS_ORBIT_PATHS:
        if not include_ephemeris_orbit_paths:
            return False
        if node_name == "iss_orbit" and pointing_target != PointingTarget.ISS:
            return False
        return True
    return True


def _serialize_scene(
    scene: trimesh.Scene,
    pointing_target: PointingTarget,
    *,
    include_ephemeris_orbit_paths: bool,
) -> dict[str, Any]:
    animation = scene.metadata["earth_sun_animation"]
    current = animation["current_time"]
    return {
        "simulation_time_iso": current.iso,
        "inertial_to_root_rotation": _rotation_to_three(
            ecliptic_to_galactocentric_rotation(current),
        ),
        "bodies": [_serialize_body(scene, node) for node in _BODY_NODES],
        "paths": [
            _serialize_path(scene, node)
            for node in _PATH_NODES
            if _include_path_in_snapshot(node, pointing_target, include_ephemeris_orbit_paths)
        ],
        "parametric_orbits": _serialize_parametric_orbits(scene, current, pointing_target),
        "arrows": [
            _serialize_arrow(scene, node)
            for node in _ARROW_NODES
            if node != "cmb_dipole_arrow" or pointing_target == PointingTarget.CMB_DIPOLE
        ],
    }


def _serialize_body(scene: trimesh.Scene, node_name: str) -> dict[str, Any]:
    transform, geometry_name = scene.graph.get(node_name, ROOT_FRAME)
    mesh = scene.geometry[geometry_name]
    # bounding_sphere integrates volume; tiny AU-scale markers can divide by zero.
    radius = float(np.max(mesh.extents) / 2.0)
    color = _mesh_color(mesh)
    return {
        "name": node_name,
        "radius": radius,
        "color": color,
        "matrix": _matrix_to_three(transform),
    }


def _serialize_parametric_orbits(
    scene: trimesh.Scene,
    time: Any,
    pointing_target: PointingTarget,
) -> list[dict[str, Any]]:
    payloads = parametric_orbit_payloads(time, pointing_target)
    for entry in payloads:
        node_name = entry["name"]
        transform, geometry_name = scene.graph.get(node_name, ROOT_FRAME)
        geometry = scene.geometry[geometry_name]
        entry["color"] = _path_color(geometry)
        entry["matrix"] = _matrix_to_three(transform)
        if entry.get("origin_body") == "earth":
            entry["origin_heliocentric_au"] = earth_heliocentric_ecliptic_au(time).astype(float).tolist()
    return payloads


def _serialize_path(scene: trimesh.Scene, node_name: str) -> dict[str, Any]:
    transform, geometry_name = scene.graph.get(node_name, ROOT_FRAME)
    geometry = scene.geometry[geometry_name]
    path_cache = scene.metadata.setdefault("path_serialize_cache", {})
    cached = path_cache.get(node_name)
    geometry_key = id(geometry)
    if cached is None or cached["geometry_key"] != geometry_key:
        cached = {
            "geometry_key": geometry_key,
            "color": _path_color(geometry),
            "segments": _path_segments_local(geometry),
        }
        path_cache[node_name] = cached
    return {
        "name": node_name,
        "color": cached["color"],
        "matrix": _matrix_to_three(transform),
        "segments": cached["segments"],
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
    visual = mesh.visual
    if not isinstance(visual, ColorVisuals):
        raise TypeError("mesh must use ColorVisuals")
    colors = visual.face_colors
    if len(colors) == 0:
        raise ValueError("mesh has no face colors")
    channel = colors[0]
    return [int(channel[0]), int(channel[1]), int(channel[2])]


def _path_color(geometry: trimesh.path.Path3D) -> list[int]:
    if geometry.colors is None or len(geometry.colors) == 0:
        return [200, 200, 200]
    channel = geometry.colors[0]
    return [int(channel[0]), int(channel[1]), int(channel[2])]


def _path_segments_local(geometry: trimesh.path.Path3D) -> list[list[list[float]]]:
    segments: list[list[list[float]]] = []
    for entity in geometry.entities:
        points = geometry.vertices[entity.points]
        if len(points) < 2:
            continue
        segments.append(points.astype(float).tolist())
    return segments


def _matrix_to_three(matrix: np.ndarray) -> list[float]:
    return matrix.T.reshape(-1).astype(float).tolist()


def _rotation_to_three(rotation: np.ndarray) -> list[float]:
    return rotation.T.reshape(-1).astype(float).tolist()
