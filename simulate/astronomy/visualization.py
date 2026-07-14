"""Earth-sun and Milky Way visualization.

Scene graph:
  world -> earth_view (translates so earth stays at the origin)
         -> milky_way (galactic center, axis, sun's galactic orbit)
                -> solar_system (sun at origin, earth and earth orbit in ecliptic coords)

Galactic kpc positions come from ephemeris at true scale, then multiplied by
GALACTIC_ORBIT_DISTANCE_SCALE after kpc->AU conversion (1 = true scale; solar-system
geometry stays true AU). Only lengths are scaled, not the ecliptic-to-galactic rotation.

Viewer note: Trimesh's SceneViewer shifts GL_LINES in camera space by scene.scale / 1000
to reduce z-fighting. Galactic geometry sets scene.scale, so the default offset visibly
displaces orbit lines while meshes (sun, earth) stay put. Pass offset_lines=False so orbit
paths align with the bodies they describe.

Camera note: Trimesh's default z_near is 0.01 AU. Exaggerated Earth is much smaller, so
zooming in clips the planet unless z_near is reduced. z_near is clamped by MAX_DEPTH_RATIO
and OPENGL_Z_NEAR_MIN_AU because gluPerspective rejects smaller near planes on macOS.
z_far is set from camera distance plus scene scale so galactic geometry stays visible
when zooming out.
"""

import warnings
from time import perf_counter
from typing import TypedDict

import numpy as np
import trimesh
from astropy import units as u
from astropy.time import Time
from erfa import ErfaWarning
from trimesh.visual.color import ColorVisuals

from simulate.astronomy.constants import (
    ANIMATION_CALLBACK_PERIOD,
    AXIS_COLOR,
    AXIS_HALF_LENGTH_CAMERA_DISTANCE_FRACTION,
    AXIS_MIN_TOTAL_LENGTH_EARTH_DIAMETERS,
    CAMERA_DISTANCE_EARTH_RADII,
    EARTH_ORBIT_SEGMENTS,
    EARTH_RADIUS_AU,
    EARTH_VIEW_FRAME,
    EARTH_VIEW_ORIGIN,
    GALACTIC_AXIS_COLOR,
    GALACTIC_AXIS_HALF_LENGTH_ORBIT_FRACTION,
    GALACTIC_CENTER_COLOR,
    GALACTIC_CENTER_RADIUS_ORBIT_FRACTION,
    GALACTIC_ORBIT_COLOR,
    GALACTIC_ORBIT_DISTANCE_SCALE,
    KPC_TO_AU,
    LINE_WIDTH_PIXELS,
    MILKY_WAY_FRAME,
    NETHERLANDS_COLOR,
    NETHERLANDS_MARKER_EARTH_RADII,
    ORBIT_COLOR,
    ORBIT_UPDATE_INTERVAL,
    SOLAR_SYSTEM_FRAME,
    SUN_COLOR,
    SUN_RADIUS_AU,
    YEAR_BOUNDARY_COLOR,
)
from simulate.astronomy.earth_centered_viewer import EarthCenteredViewer
from simulate.astronomy.utils.earth_mesh import create_earth
from simulate.astronomy.utils.ephemeris import (
    current_time,
    earth_heliocentric_ecliptic_au,
    earth_orbit_ecliptic_au,
    earth_orientation_matrix,
    earth_spin_axis_ecliptic,
    earth_year_boundary_positions_ecliptic_au,
    ecliptic_to_galactocentric_rotation,
    netherlands_direction_ecliptic,
    sun_galactic_orbit_kpc,
    sun_galactocentric_kpc,
)


class EarthSunAnimationState(TypedDict):
    start_time: Time
    last_orbit_time: Time | None
    time_scaling: float
    wall_start: float | None
    current_time: Time | None


class EarthSunState(TypedDict):
    earth_position: np.ndarray
    earth_rotation: np.ndarray
    spin_axis: np.ndarray
    netherlands_position: np.ndarray
    galactic_rotation: np.ndarray
    sun_galactic_position: np.ndarray
    galactic_axis_half_length_au: float
    galactic_center_radius_au: float


def show_earth_sun(
    time: Time | None = None,
    *,
    time_scaling: float = 1.0,
) -> None:
    warnings.filterwarnings("ignore", message=".*dubious year.*", category=ErfaWarning)

    if time is None:
        time = current_time()

    scene = build_earth_sun_scene(time)
    scene.set_camera(
        center=EARTH_VIEW_ORIGIN,
        distance=CAMERA_DISTANCE_EARTH_RADII * EARTH_RADIUS_AU,
    )
    scene.metadata["earth_sun_animation"] = EarthSunAnimationState(
        start_time=time,
        last_orbit_time=None,
        time_scaling=time_scaling,
        wall_start=None,
        current_time=time,
    )
    # Orbit paths are GL_LINES; see module docstring for why offset_lines must be False.
    scene.show(
        viewer=EarthCenteredViewer,
        camera_distance=CAMERA_DISTANCE_EARTH_RADII * EARTH_RADIUS_AU,
        offset_lines=False,
        line_settings={"line_width": LINE_WIDTH_PIXELS},
        callback=_earth_sun_animation_callback,
        callback_period=ANIMATION_CALLBACK_PERIOD,
        caption=f"Earth-sun ({time.iso})",
    )


def build_earth_sun_scene(time: Time | None = None) -> trimesh.Scene:
    if time is None:
        time = current_time()

    scene = trimesh.Scene()
    scene.graph.update(EARTH_VIEW_FRAME, "world", matrix=np.eye(4))
    scene.graph.update(MILKY_WAY_FRAME, EARTH_VIEW_FRAME, matrix=np.eye(4))
    scene.graph.update(SOLAR_SYSTEM_FRAME, MILKY_WAY_FRAME, matrix=np.eye(4))

    scene.add_geometry(
        _color_mesh(
            trimesh.creation.icosphere(radius=SUN_RADIUS_AU, subdivisions=4),
            SUN_COLOR,
        ),
        geom_name="sun",
        node_name="sun",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    scene.add_geometry(
        create_earth(EARTH_RADIUS_AU),
        geom_name="earth",
        node_name="earth",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    scene.add_geometry(
        _color_mesh(
            trimesh.creation.icosphere(
                radius=NETHERLANDS_MARKER_EARTH_RADII * EARTH_RADIUS_AU,
                subdivisions=2,
            ),
            NETHERLANDS_COLOR,
        ),
        geom_name="netherlands",
        node_name="netherlands",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    scene.add_geometry(
        _color_mesh(
            trimesh.creation.icosphere(radius=1.0, subdivisions=3),
            GALACTIC_CENTER_COLOR,
        ),
        geom_name="galactic_center",
        node_name="galactic_center",
        parent_node_name=MILKY_WAY_FRAME,
    )

    for geom_name, color in (
        ("galactic_orbit", GALACTIC_ORBIT_COLOR),
        ("galactic_axis", GALACTIC_AXIS_COLOR),
        ("orbit", ORBIT_COLOR),
        ("year_boundaries", YEAR_BOUNDARY_COLOR),
        ("earth_axis", AXIS_COLOR),
    ):
        scene.add_geometry(
            _placeholder_path(color),
            geom_name=geom_name,
            node_name=geom_name,
            parent_node_name=MILKY_WAY_FRAME
            if geom_name in ("galactic_orbit", "galactic_axis")
            else SOLAR_SYSTEM_FRAME,
        )

    update_earth_sun_scene(scene, time, None, CAMERA_DISTANCE_EARTH_RADII * EARTH_RADIUS_AU)
    return scene


def update_earth_sun_scene(
    scene: trimesh.Scene,
    time: Time,
    last_orbit_time: Time | None,
    camera_distance_au: float,
) -> Time:
    state = _earth_sun_state(time)
    scene.graph.update(
        SOLAR_SYSTEM_FRAME,
        MILKY_WAY_FRAME,
        matrix=_transform_matrix(state["galactic_rotation"], state["sun_galactic_position"]),
    )
    scene.graph.update(
        "earth",
        SOLAR_SYSTEM_FRAME,
        matrix=_transform_matrix(state["earth_rotation"], state["earth_position"]),
    )
    scene.graph.update(
        "netherlands",
        SOLAR_SYSTEM_FRAME,
        matrix=_transform_matrix(np.eye(3), state["netherlands_position"]),
    )
    scene.geometry["earth_axis"] = _earth_axis_path(state, camera_distance_au)
    scene.geometry["galactic_axis"] = _segment_path(
        np.array([0.0, 0.0, -state["galactic_axis_half_length_au"]]),
        np.array([0.0, 0.0, state["galactic_axis_half_length_au"]]),
        GALACTIC_AXIS_COLOR,
    )
    _sync_earth_view_frame(scene, state)

    if last_orbit_time is None or abs(time - last_orbit_time) >= ORBIT_UPDATE_INTERVAL:
        scene.geometry["orbit"] = _colored_path(
            earth_orbit_ecliptic_au(time, samples=EARTH_ORBIT_SEGMENTS),
            ORBIT_COLOR,
        )
        scene.geometry["year_boundaries"] = _year_boundary_path(time)
        scene.geometry["galactic_orbit"] = _colored_path(
            _galactic_orbit_points(time),
            GALACTIC_ORBIT_COLOR,
        )
        scene.geometry["galactic_center"] = _color_mesh(
            trimesh.creation.icosphere(radius=state["galactic_center_radius_au"], subdivisions=3),
            GALACTIC_CENTER_COLOR,
        )
        return time

    return last_orbit_time


def _earth_sun_animation_callback(scene: trimesh.Scene) -> None:
    animation = scene.metadata["earth_sun_animation"]
    if animation["wall_start"] is None:
        animation["wall_start"] = perf_counter()
    elapsed = perf_counter() - animation["wall_start"]
    time = animation["start_time"] + elapsed * animation["time_scaling"] * u.second
    animation["current_time"] = time
    animation["last_orbit_time"] = update_earth_sun_scene(
        scene,
        time,
        animation["last_orbit_time"],
        _camera_distance_au(scene),
    )


def _camera_distance_au(scene: trimesh.Scene) -> float:
    eye = scene.camera_transform[:3, 3]
    return float(np.linalg.norm(eye - EARTH_VIEW_ORIGIN))


def _galactic_orbit_points(time: Time) -> np.ndarray:
    galactic_scale = KPC_TO_AU * GALACTIC_ORBIT_DISTANCE_SCALE
    return sun_galactic_orbit_kpc(time) * galactic_scale


def _earth_sun_state(time: Time) -> EarthSunState:
    earth_position = earth_heliocentric_ecliptic_au(time)
    spin_axis = earth_spin_axis_ecliptic(time)
    netherlands_dir = netherlands_direction_ecliptic(time)

    sun_galactic_kpc = sun_galactocentric_kpc(time)
    sun_galactic_distance_kpc = np.linalg.norm(sun_galactic_kpc)
    galactic_scale = KPC_TO_AU * GALACTIC_ORBIT_DISTANCE_SCALE
    sun_galactic_distance_au = sun_galactic_distance_kpc * galactic_scale
    sun_galactic_position = sun_galactic_kpc * galactic_scale

    return {
        "earth_position": earth_position,
        "earth_rotation": earth_orientation_matrix(time),
        "spin_axis": spin_axis,
        "netherlands_position": earth_position + netherlands_dir * EARTH_RADIUS_AU,
        "galactic_rotation": ecliptic_to_galactocentric_rotation(time),
        "sun_galactic_position": sun_galactic_position,
        "galactic_axis_half_length_au": (sun_galactic_distance_au * GALACTIC_AXIS_HALF_LENGTH_ORBIT_FRACTION),
        "galactic_center_radius_au": (sun_galactic_distance_au * GALACTIC_CENTER_RADIUS_ORBIT_FRACTION),
    }


def _sync_earth_view_frame(scene: trimesh.Scene, state: EarthSunState) -> None:
    earth_world = _earth_world_position(state)
    scene.graph.update(
        frame_to=EARTH_VIEW_FRAME,
        frame_from="world",
        matrix=_transform_matrix(np.eye(3), -earth_world),
    )


def _earth_world_position(state: EarthSunState) -> np.ndarray:
    solar_system = _transform_matrix(state["galactic_rotation"], state["sun_galactic_position"])
    earth = _transform_matrix(state["earth_rotation"], state["earth_position"])
    return (solar_system @ earth)[:3, 3]


def _earth_axis_path(state: EarthSunState, camera_distance_au: float) -> trimesh.path.Path3D:
    earth_position = state["earth_position"]
    spin_axis = state["spin_axis"]
    axis_half_length = max(
        camera_distance_au * AXIS_HALF_LENGTH_CAMERA_DISTANCE_FRACTION,
        AXIS_MIN_TOTAL_LENGTH_EARTH_DIAMETERS * EARTH_RADIUS_AU,
    )
    axis_start = earth_position - spin_axis * axis_half_length
    axis_end = earth_position + spin_axis * axis_half_length
    return _segment_path(axis_start, axis_end, AXIS_COLOR)


def _colored_path(points: np.ndarray, color: list[int]) -> trimesh.path.Path3D:
    path = trimesh.load_path(points)
    path.colors = np.tile(color, (len(path.entities), 1))
    return path


def _placeholder_path(color: list[int]) -> trimesh.path.Path3D:
    origin = np.zeros(3, dtype=float)
    return _segment_path(origin, origin, color)


def _segment_path(start: np.ndarray, end: np.ndarray, color: list[int]) -> trimesh.path.Path3D:
    path = trimesh.load_path(np.array([[start, end]]))
    path.colors = np.tile(color, (len(path.entities), 1))
    return path


def _year_boundary_path(time: Time) -> trimesh.path.Path3D:
    positions = earth_year_boundary_positions_ecliptic_au(time)
    if len(positions) == 0:
        # earth_year_boundary_positions_ecliptic_au can return no Jan 1 in the
        # ±0.5 Julian year window (see its docstring). Trimesh's Scene.scale/bounds
        # computation assumes geometry bounds exist, so use a zero-length segment.
        return _placeholder_path(YEAR_BOUNDARY_COLOR)
    return _colored_path(
        np.array([[np.zeros(3, dtype=float), position] for position in positions]),
        YEAR_BOUNDARY_COLOR,
    )


def _transform_matrix(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def _color_mesh(mesh: trimesh.Trimesh, color: list[int]) -> trimesh.Trimesh:
    visual = mesh.visual
    if not isinstance(visual, ColorVisuals):
        raise TypeError("mesh must use ColorVisuals")
    visual.face_colors = np.asarray(color, dtype=np.uint8)
    return mesh


if __name__ == "__main__":
    # show_earth_sun()
    show_earth_sun(time_scaling=86_400)  # 1 day per second
    # show_earth_sun(time_scaling=60 * 60 * 24)  # 1 day per second
    # show_earth_sun(time_scaling=60 * 60 * 24 * 30)  # 1 month per second
