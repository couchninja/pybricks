"""Earth-sun and Milky Way scene graph and animation.

Scene graph:
  root_frame
    earth_center
      milky_way
        solar_system
          sun, earth, moon, iss, observer, earth_orbit, moon_orbit, iss_orbit, year_boundaries, earth_axis
        galactic_center, galactic_orbit, galactic_axis, cmb_dipole_arrow

The graph stacks two independent concerns:

1. Coordinate frames (physical nesting). ``milky_way`` holds galactic geometry and
   parents ``solar_system``, which uses heliocentric ecliptic coords (Sun at the
   origin, Earth, Moon, and Earth's orbit underneath). The edge ``milky_way -> solar_system``
   carries the ecliptic-to-galactic rotation and the Sun's galactocentric position.

2. Display recentering. ``earth_center`` translates the entire subtree by
   ``-earth_root`` each frame so Earth stays at ``EARTH_CENTER_ORIGIN``. The web
   viewer camera orbits that fixed origin. ``earth_center`` parents ``milky_way`` not
   because Earth contains the galaxy, but because every body — solar-system and
   galactic — must shift together when recentering. If galactic geometry sat outside
   ``earth_center``, it would stay fixed in root-frame space while only the solar
   system moved, breaking relative positions.

Galactic kpc positions come from ephemeris at true scale, then multiplied by
GALACTIC_ORBIT_DISTANCE_SCALE after kpc->AU conversion (1 = true scale; solar-system
geometry stays true AU). Only lengths are scaled, not the ecliptic-to-galactic rotation.
"""

from typing import TypedDict

import numpy as np
import trimesh
from astropy import units as u
from astropy.time import Time
from trimesh.visual.color import ColorVisuals

from gpio.button_config import pointing_target_button_rgba
from simulate.astronomy.constants import (
    AXIS_COLOR,
    AXIS_HALF_LENGTH_CAMERA_DISTANCE_FRACTION,
    AXIS_MIN_TOTAL_LENGTH_EARTH_DIAMETERS,
    CAMERA_DISTANCE_EARTH_RADII,
    CMB_DIPOLE_ARROW_COLOR,
    EARTH_CENTER_FRAME,
    EARTH_ORBIT_COLOR,
    EARTH_ORBIT_SEGMENTS,
    EARTH_RADIUS_AU,
    GALACTIC_AXIS_COLOR,
    GALACTIC_AXIS_HALF_LENGTH_ORBIT_FRACTION,
    GALACTIC_CENTER_COLOR,
    GALACTIC_CENTER_RADIUS_ORBIT_FRACTION,
    GALACTIC_ORBIT_COLOR,
    GALACTIC_ORBIT_DISTANCE_SCALE,
    ISS_COLOR,
    ISS_MARKER_EARTH_RADII,
    ISS_ORBIT_COLOR,
    ISS_ORBIT_SEGMENTS,
    KPC_TO_AU,
    MILKY_WAY_FRAME,
    MOON_COLOR,
    MOON_ORBIT_COLOR,
    MOON_ORBIT_SEGMENTS,
    MOON_RADIUS_AU,
    MOON_SIDEREAL_ORBIT_PERIOD,
    OBSERVER_COLOR,
    OBSERVER_MARKER_EARTH_RADII,
    OBSERVER_VELOCITY_ARROW_HEAD_LENGTH_FRACTION,
    OBSERVER_VELOCITY_ARROW_HEAD_RADIUS_EARTH_RADII,
    OBSERVER_VELOCITY_ARROW_LENGTH_CAMERA_DISTANCE_FRACTION,
    OBSERVER_VELOCITY_ARROW_LENGTH_EARTH_RADII,
    OBSERVER_VELOCITY_ARROW_SHAFT_RADIUS_EARTH_RADII,
    OBSERVER_VELOCITY_ARROW_SHAFT_START_OBSERVER_RADIUS_MULTIPLE,
    ORBIT_GEOMETRY_MIN_UPDATE_INTERVAL,
    ORBIT_UPDATE_INTERVAL,
    ROOT_FRAME,
    SOLAR_SYSTEM_FRAME,
    SUN_COLOR,
    SUN_RADIUS_AU,
    YEAR_BOUNDARY_COLOR,
    PointingTarget,
)
from simulate.astronomy.simulation_clock import time_scaling
from simulate.astronomy.utils.camera import camera_distance_to_point_au
from simulate.astronomy.utils.earth_mesh import create_earth
from simulate.astronomy.utils.ephemeris import (
    current_time,
    earth_heliocentric_ecliptic_au,
    earth_orbit_ecliptic_au,
    earth_orientation_matrix,
    earth_spin_axis_ecliptic,
    earth_year_boundary_positions_ecliptic_au,
    ecliptic_to_galactocentric_rotation,
    iss_heliocentric_ecliptic_au,
    iss_orbit_ecliptic_au,
    milky_way_cmb_direction_galactocentric,
    moon_heliocentric_ecliptic_au,
    moon_orbit_ecliptic_au,
    observer_direction_ecliptic,
    observer_direction_ecliptic_for_target,
    sun_galactic_orbit_kpc,
    sun_galactocentric_kpc,
)
from simulate.astronomy.utils.iss import iss_orbital_period, refresh_iss_tle


class EarthSunAnimationState(TypedDict):
    last_orbit_time: Time | None
    last_moon_orbit_time: Time | None
    last_iss_orbit_time: Time | None
    time_scaling: float
    current_time: Time | None


class EarthSunState(TypedDict):
    earth_position: np.ndarray
    iss_position: np.ndarray
    moon_position: np.ndarray
    earth_rotation: np.ndarray
    spin_axis: np.ndarray
    observer_position: np.ndarray
    galactic_rotation: np.ndarray
    sun_galactic_position: np.ndarray
    galactic_axis_half_length_au: float
    galactic_center_radius_au: float


def build_earth_sun_scene(time: Time | None = None) -> trimesh.Scene:
    if time is None:
        time = current_time()

    refresh_iss_tle()

    scene = trimesh.Scene(base_frame=ROOT_FRAME)
    scene.graph.update(EARTH_CENTER_FRAME, ROOT_FRAME, matrix=np.eye(4))
    scene.graph.update(MILKY_WAY_FRAME, EARTH_CENTER_FRAME, matrix=np.eye(4))
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
            trimesh.creation.icosphere(radius=MOON_RADIUS_AU, subdivisions=3),
            MOON_COLOR,
        ),
        geom_name="moon",
        node_name="moon",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    scene.add_geometry(
        _color_mesh(
            trimesh.creation.icosphere(
                radius=ISS_MARKER_EARTH_RADII * EARTH_RADIUS_AU,
                subdivisions=2,
            ),
            ISS_COLOR,
        ),
        geom_name="iss",
        node_name="iss",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    scene.add_geometry(
        _color_mesh(
            trimesh.creation.icosphere(
                radius=OBSERVER_MARKER_EARTH_RADII * EARTH_RADIUS_AU,
                subdivisions=2,
            ),
            OBSERVER_COLOR,
        ),
        geom_name="observer",
        node_name="observer",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    scene.add_geometry(
        _color_mesh(
            _velocity_arrow_mesh(),
            pointing_target_button_rgba(PointingTarget.EARTH_ROTATION),
        ),
        geom_name="observer_velocity_arrow",
        node_name="observer_velocity_arrow",
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

    scene.add_geometry(
        _color_mesh(
            _velocity_arrow_mesh(),
            CMB_DIPOLE_ARROW_COLOR,
        ),
        geom_name="cmb_dipole_arrow",
        node_name="cmb_dipole_arrow",
        parent_node_name=MILKY_WAY_FRAME,
    )

    for geom_name, color in (
        ("galactic_orbit", GALACTIC_ORBIT_COLOR),
        ("galactic_axis", GALACTIC_AXIS_COLOR),
        ("earth_orbit", EARTH_ORBIT_COLOR),
        ("moon_orbit", MOON_ORBIT_COLOR),
        ("iss_orbit", ISS_ORBIT_COLOR),
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

    update_earth_sun_scene(
        scene,
        time,
        None,
        CAMERA_DISTANCE_EARTH_RADII * EARTH_RADIUS_AU,
        last_moon_orbit_time=None,
        last_iss_orbit_time=None,
    )
    _print_scene_graph(scene)
    return scene


def update_earth_sun_scene(
    scene: trimesh.Scene,
    time: Time,
    last_orbit_time: Time | None,
    camera_distance_au: float,
    *,
    last_moon_orbit_time: Time | None,
    last_iss_orbit_time: Time | None,
) -> tuple[Time | None, Time | None, Time | None]:
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
        "moon",
        SOLAR_SYSTEM_FRAME,
        matrix=_transform_matrix(np.eye(3), state["moon_position"]),
    )
    scene.graph.update(
        "iss",
        SOLAR_SYSTEM_FRAME,
        matrix=_transform_matrix(np.eye(3), state["iss_position"]),
    )
    scene.graph.update(
        "observer",
        SOLAR_SYSTEM_FRAME,
        matrix=_transform_matrix(np.eye(3), state["observer_position"]),
    )
    pointing_target = scene.metadata.get("pointing_target", PointingTarget.EARTH_ROTATION)
    _color_mesh(
        scene.geometry["observer_velocity_arrow"],
        pointing_target_button_rgba(pointing_target),
    )
    scene.graph.update(
        "observer_velocity_arrow",
        SOLAR_SYSTEM_FRAME,
        matrix=_observer_velocity_arrow_transform(
            state["observer_position"],
            time,
            pointing_target,
            camera_distance_au,
        ),
    )
    if pointing_target == PointingTarget.CMB_DIPOLE:
        cmb_dipole_arrow_matrix = _cmb_dipole_arrow_transform(
            time,
            camera_distance_to_point_au(scene, _galactic_center_position(state)),
        )
    else:
        cmb_dipole_arrow_matrix = _transform_matrix(np.diag([0.0, 0.0, 0.0]), np.zeros(3))
    scene.graph.update(
        "cmb_dipole_arrow",
        MILKY_WAY_FRAME,
        matrix=cmb_dipole_arrow_matrix,
    )
    scene.geometry["earth_axis"] = _earth_axis_path(state, camera_distance_au)
    scene.geometry["galactic_axis"] = _segment_path(
        np.array([0.0, 0.0, -state["galactic_axis_half_length_au"]]),
        np.array([0.0, 0.0, state["galactic_axis_half_length_au"]]),
        GALACTIC_AXIS_COLOR,
    )
    _sync_earth_center_frame(scene, state)

    moon_orbit_interval = _scaled_orbit_geometry_interval(MOON_SIDEREAL_ORBIT_PERIOD, MOON_ORBIT_SEGMENTS)
    if last_moon_orbit_time is None or abs(time - last_moon_orbit_time) >= moon_orbit_interval:
        scene.geometry["moon_orbit"] = _colored_path(
            moon_orbit_ecliptic_au(time, samples=MOON_ORBIT_SEGMENTS),
            MOON_ORBIT_COLOR,
        )
        last_moon_orbit_time = time

    pointing_target = scene.metadata.get("pointing_target", PointingTarget.EARTH_ROTATION)
    if pointing_target == PointingTarget.ISS:
        iss_period = iss_orbital_period(time)
        iss_orbit_interval = _scaled_orbit_geometry_interval(iss_period, ISS_ORBIT_SEGMENTS)
        if last_iss_orbit_time is None or abs(time - last_iss_orbit_time) >= iss_orbit_interval:
            scene.geometry["iss_orbit"] = _colored_path(
                iss_orbit_ecliptic_au(time, samples=ISS_ORBIT_SEGMENTS),
                ISS_ORBIT_COLOR,
            )
            scene.metadata["iss_orbit_is_placeholder"] = False
            last_iss_orbit_time = time
    elif scene.metadata.get("iss_orbit_is_placeholder") is not True:
        scene.geometry["iss_orbit"] = _placeholder_path(ISS_ORBIT_COLOR)
        scene.metadata["iss_orbit_is_placeholder"] = True
        last_iss_orbit_time = None

    if last_orbit_time is None or abs(time - last_orbit_time) >= ORBIT_UPDATE_INTERVAL:
        scene.geometry["earth_orbit"] = _colored_path(
            earth_orbit_ecliptic_au(time, samples=EARTH_ORBIT_SEGMENTS),
            EARTH_ORBIT_COLOR,
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
        scene.metadata["milky_way_diameter_au"] = _milky_way_diameter_au(scene)
        last_orbit_time = time

    return last_orbit_time, last_moon_orbit_time, last_iss_orbit_time


def _milky_way_diameter_au(scene: trimesh.Scene) -> float:
    gc_transform, _ = scene.graph.get("galactic_center", ROOT_FRAME)
    galactic_center = gc_transform[:3, 3]
    orbit_transform, orbit_geometry_name = scene.graph.get("galactic_orbit", ROOT_FRAME)
    orbit = scene.geometry[orbit_geometry_name]
    orbit_world = trimesh.transformations.transform_points(orbit.vertices, orbit_transform)
    orbit_radius = float(np.max(np.linalg.norm(orbit_world - galactic_center, axis=1)))
    return 2.0 * orbit_radius


def _scaled_orbit_geometry_interval(orbit_period: u.Quantity, segments: int) -> u.Quantity:
    """Sim-time between orbit path rebuilds; shrinks at higher time scale so the body stays on the line."""
    segment_time = orbit_period / segments
    scaled = segment_time / max(time_scaling(), 1.0)
    return scaled if scaled > ORBIT_GEOMETRY_MIN_UPDATE_INTERVAL else ORBIT_GEOMETRY_MIN_UPDATE_INTERVAL


def _galactic_center_position(state: EarthSunState) -> np.ndarray:
    return -_earth_root_position(state)


def _galactic_orbit_points(time: Time) -> np.ndarray:
    galactic_scale = KPC_TO_AU * GALACTIC_ORBIT_DISTANCE_SCALE
    return sun_galactic_orbit_kpc(time) * galactic_scale


def _earth_sun_state(time: Time) -> EarthSunState:
    earth_position = earth_heliocentric_ecliptic_au(time)
    spin_axis = earth_spin_axis_ecliptic(time)
    observer_dir = observer_direction_ecliptic(time)

    sun_galactic_kpc = sun_galactocentric_kpc(time)
    sun_galactic_distance_kpc = np.linalg.norm(sun_galactic_kpc)
    galactic_scale = KPC_TO_AU * GALACTIC_ORBIT_DISTANCE_SCALE
    sun_galactic_distance_au = sun_galactic_distance_kpc * galactic_scale
    sun_galactic_position = sun_galactic_kpc * galactic_scale

    return {
        "earth_position": earth_position,
        "iss_position": iss_heliocentric_ecliptic_au(time),
        "moon_position": moon_heliocentric_ecliptic_au(time),
        "earth_rotation": earth_orientation_matrix(time),
        "spin_axis": spin_axis,
        "observer_position": earth_position + observer_dir * EARTH_RADIUS_AU,
        "galactic_rotation": ecliptic_to_galactocentric_rotation(time),
        "sun_galactic_position": sun_galactic_position,
        "galactic_axis_half_length_au": (sun_galactic_distance_au * GALACTIC_AXIS_HALF_LENGTH_ORBIT_FRACTION),
        "galactic_center_radius_au": (sun_galactic_distance_au * GALACTIC_CENTER_RADIUS_ORBIT_FRACTION),
    }


def _sync_earth_center_frame(scene: trimesh.Scene, state: EarthSunState) -> None:
    earth_root = _earth_root_position(state)
    scene.graph.update(
        frame_to=EARTH_CENTER_FRAME,
        frame_from=ROOT_FRAME,
        matrix=_transform_matrix(np.eye(3), -earth_root),
    )


def _earth_root_position(state: EarthSunState) -> np.ndarray:
    solar_system = _transform_matrix(state["galactic_rotation"], state["sun_galactic_position"])
    earth = _transform_matrix(state["earth_rotation"], state["earth_position"])
    return (solar_system @ earth)[:3, 3]


def _velocity_arrow_mesh() -> trimesh.Trimesh:
    total_length = OBSERVER_VELOCITY_ARROW_LENGTH_EARTH_RADII
    head_length = total_length * OBSERVER_VELOCITY_ARROW_HEAD_LENGTH_FRACTION
    shaft_length = total_length - head_length
    shaft_radius = OBSERVER_VELOCITY_ARROW_SHAFT_RADIUS_EARTH_RADII
    head_radius = OBSERVER_VELOCITY_ARROW_HEAD_RADIUS_EARTH_RADII
    shaft = trimesh.creation.cylinder(radius=shaft_radius, height=shaft_length)
    shaft.apply_translation([0.0, 0.0, shaft_length / 2])
    head = trimesh.creation.cone(radius=head_radius, height=head_length)
    head.apply_translation([0.0, 0.0, shaft_length + head_length / 2])
    arrow = trimesh.util.concatenate([shaft, head])
    if not isinstance(arrow, trimesh.Trimesh):
        raise TypeError("velocity arrow must be a Trimesh")
    arrow.apply_scale(EARTH_RADIUS_AU)
    return arrow


def _velocity_arrow_mesh_length_au() -> float:
    return OBSERVER_VELOCITY_ARROW_LENGTH_EARTH_RADII * EARTH_RADIUS_AU


def _velocity_arrow_length_au(camera_distance_au: float) -> float:
    return camera_distance_au * OBSERVER_VELOCITY_ARROW_LENGTH_CAMERA_DISTANCE_FRACTION


def _observer_velocity_arrow_shaft_start_au() -> float:
    return OBSERVER_VELOCITY_ARROW_SHAFT_START_OBSERVER_RADIUS_MULTIPLE * OBSERVER_MARKER_EARTH_RADII * EARTH_RADIUS_AU


def _direction_arrow_transform(
    origin: np.ndarray,
    direction: np.ndarray,
    camera_distance_au: float,
    shaft_start_au: float = 0.0,
) -> np.ndarray:
    mesh_length = _velocity_arrow_mesh_length_au()
    arrow_length = _velocity_arrow_length_au(camera_distance_au)
    scale = arrow_length / mesh_length
    rotation = _rotation_align_z_to(direction)
    unit = direction / np.linalg.norm(direction)
    matrix = np.eye(4)
    matrix[:3, :3] = rotation * scale
    matrix[:3, 3] = origin + unit * shaft_start_au
    return matrix


def _observer_velocity_arrow_transform(
    observer_position: np.ndarray,
    time: Time,
    pointing_target: PointingTarget,
    camera_distance_au: float,
) -> np.ndarray:
    direction = observer_direction_ecliptic_for_target(time, pointing_target)
    if direction is None:
        return _transform_matrix(np.eye(3), observer_position)

    return _direction_arrow_transform(
        observer_position,
        direction,
        camera_distance_au,
        shaft_start_au=_observer_velocity_arrow_shaft_start_au(),
    )


def _cmb_dipole_arrow_transform(
    time: Time,
    camera_distance_au: float,
) -> np.ndarray:
    direction = milky_way_cmb_direction_galactocentric(time)
    origin = np.zeros(3, dtype=float)
    return _direction_arrow_transform(origin, direction, camera_distance_au)


def _rotation_align_z_to(direction: np.ndarray) -> np.ndarray:
    z_axis = np.array([0.0, 0.0, 1.0])
    if np.allclose(direction, z_axis):
        return np.eye(3)
    if np.allclose(direction, -z_axis):
        return np.diag([1.0, -1.0, -1.0])
    axis = np.cross(z_axis, direction)
    axis /= np.linalg.norm(axis)
    angle = float(np.arccos(np.clip(np.dot(z_axis, direction), -1.0, 1.0)))
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    cross = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    return np.eye(3) * cos_angle + cross * sin_angle + np.outer(axis, axis) * (1.0 - cos_angle)


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


def _print_scene_graph(scene: trimesh.Scene) -> None:
    parents = scene.graph.transforms.parents
    root = scene.graph.base_frame
    if not isinstance(root, str):
        raise TypeError("scene graph base_frame must be a str")

    def print_node(node: str, indent: int) -> None:
        print("  " * indent + node)  # noqa: T201
        children = sorted(child for child, parent in parents.items() if parent == node)
        for child in children:
            print_node(child, indent + 1)

    print("Scene graph:")  # noqa: T201
    print_node(root, 0)
