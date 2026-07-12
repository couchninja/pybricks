"""Earth-sun and Milky Way visualization.

Scene graph:
  world -> milky_way (galactic center, axis, sun's galactic orbit)
         -> solar_system (sun at origin, earth and earth orbit in ecliptic coords)

Galactic distances are scaled by GALACTIC_ORBIT_VISUAL_RADIUS_AU; only lengths are
scaled, not the ecliptic-to-galactic rotation.

Viewer note: Trimesh's SceneViewer shifts GL_LINES in camera space by scene.scale / 1000
to reduce z-fighting. Adding the ~150 AU galactic orbit inflates scene.scale, so the
default offset visibly displaces orbit lines while meshes (sun, earth) stay put. Pass
offset_lines=False so orbit paths align with the bodies they describe.

Camera note: Trimesh's default z_near is 0.01 AU. Exaggerated Earth is much smaller, so
zooming in clips the planet unless z_near is reduced (see CAMERA_Z_NEAR_AU).
"""

import numpy as np
import trimesh
from astropy.time import Time
from scipy.spatial.transform import Rotation
from trimesh.visual.color import ColorVisuals
from trimesh.viewer.trackball import Trackball
from trimesh.viewer.windowed import SceneViewer

from similarity.astronomy.earth_mesh import create_earth
from similarity.astronomy.ephemeris import (
    current_time,
    earth_heliocentric_ecliptic_au,
    earth_orbit_ecliptic_au,
    earth_orientation_matrix,
    earth_spin_axis_ecliptic,
    ecliptic_to_galactocentric_rotation,
    netherlands_direction_ecliptic,
    sun_galactic_orbit_kpc,
    sun_galactocentric_kpc,
)

REAL_SUN_RADIUS_AU = 695_700 / 149_597_870.7
REAL_EARTH_RADIUS_AU = 6_371 / 149_597_870.7

SUN_SIZE_EXAGGERATION = 20
EARTH_SIZE_EXAGGERATION = 20
# SUN_SIZE_EXAGGERATION = 1
# EARTH_SIZE_EXAGGERATION = 1

SUN_RADIUS_AU = REAL_SUN_RADIUS_AU * SUN_SIZE_EXAGGERATION
EARTH_RADIUS_AU = REAL_EARTH_RADIUS_AU * EARTH_SIZE_EXAGGERATION

AXIS_HALF_LENGTH_EARTH_RADII = 2.8
NETHERLANDS_MARKER_EARTH_RADII = 0.16
LINE_RADIUS_EARTH_RADII = 0.06
CAMERA_DISTANCE_EARTH_RADII = 80
GALACTIC_ORBIT_VISUAL_RADIUS_AU = 150.0
GALACTIC_AXIS_HALF_LENGTH_AU = 170.0
GALACTIC_CENTER_RADIUS_AU = SUN_RADIUS_AU * 3
GALACTIC_LINE_RADIUS_AU = SUN_RADIUS_AU * 0.4

# Default Trimesh z_near is 0.01 AU, larger than exaggerated Earth (~0.0009 AU).
CAMERA_Z_NEAR_EARTH_RADII = 0.01
CAMERA_Z_NEAR_AU = EARTH_RADIUS_AU * CAMERA_Z_NEAR_EARTH_RADII
CAMERA_Z_FAR_AU = GALACTIC_AXIS_HALF_LENGTH_AU * 2

ORBIT_COLOR = [180, 180, 200, 255]
SUN_COLOR = [255, 210, 60, 255]
AXIS_COLOR = [220, 60, 60, 255]
NETHERLANDS_COLOR = [255, 80, 40, 255]
GALACTIC_ORBIT_COLOR = [120, 80, 180, 255]
GALACTIC_AXIS_COLOR = [180, 120, 255, 255]
GALACTIC_CENTER_COLOR = [240, 200, 255, 255]

MILKY_WAY_FRAME = "milky_way"
SOLAR_SYSTEM_FRAME = "solar_system"


def show_earth_sun(time: Time | None = None) -> None:
    if time is None:
        time = current_time()

    scene = build_earth_sun_scene(time)
    earth_transform, _ = scene.graph.get("earth", "world")
    earth_position = earth_transform[:3, 3]
    scene.apply_translation(-earth_position)
    # Orbit paths are GL_LINES; see module docstring for why offset_lines must be False.
    scene.show(
        viewer=_EarthCenteredViewer,
        earth_center=np.zeros(3),
        camera_distance=CAMERA_DISTANCE_EARTH_RADII * EARTH_RADIUS_AU,
        offset_lines=False,
    )


def build_earth_sun_scene(time: Time | None = None) -> trimesh.Scene:
    if time is None:
        time = current_time()

    earth_position = earth_heliocentric_ecliptic_au(time)
    orbit = earth_orbit_ecliptic_au(time)
    spin_axis = earth_spin_axis_ecliptic(time)
    netherlands_dir = netherlands_direction_ecliptic(time)
    earth_rotation = earth_orientation_matrix(time)

    sun_galactic_kpc = sun_galactocentric_kpc(time)
    sun_galactic_distance_kpc = np.linalg.norm(sun_galactic_kpc)
    galactic_scale = GALACTIC_ORBIT_VISUAL_RADIUS_AU / sun_galactic_distance_kpc
    sun_galactic_position = sun_galactic_kpc * galactic_scale
    galactic_rotation = ecliptic_to_galactocentric_rotation(time)
    galactic_orbit = sun_galactic_orbit_kpc(time) * galactic_scale

    scene = trimesh.Scene()
    scene.graph.update(MILKY_WAY_FRAME, "world", matrix=np.eye(4))
    scene.graph.update(
        SOLAR_SYSTEM_FRAME,
        MILKY_WAY_FRAME,
        matrix=_transform_matrix(galactic_rotation, sun_galactic_position),
    )

    galactic_center = _color_mesh(
        trimesh.creation.icosphere(radius=GALACTIC_CENTER_RADIUS_AU, subdivisions=3),
        GALACTIC_CENTER_COLOR,
    )
    scene.add_geometry(
        galactic_center,
        geom_name="galactic_center",
        node_name="galactic_center",
        parent_node_name=MILKY_WAY_FRAME,
    )

    galactic_orbit_path = trimesh.load_path(galactic_orbit)
    galactic_orbit_path.colors = np.tile(
        GALACTIC_ORBIT_COLOR, (len(galactic_orbit_path.entities), 1)
    )
    scene.add_geometry(
        galactic_orbit_path,
        geom_name="galactic_orbit",
        node_name="galactic_orbit",
        parent_node_name=MILKY_WAY_FRAME,
    )

    scene.add_geometry(
        _line_mesh(
            np.array([0.0, 0.0, -GALACTIC_AXIS_HALF_LENGTH_AU]),
            np.array([0.0, 0.0, GALACTIC_AXIS_HALF_LENGTH_AU]),
            GALACTIC_LINE_RADIUS_AU,
            GALACTIC_AXIS_COLOR,
        ),
        geom_name="galactic_axis",
        node_name="galactic_axis",
        parent_node_name=MILKY_WAY_FRAME,
    )

    sun = _color_mesh(
        trimesh.creation.icosphere(radius=SUN_RADIUS_AU, subdivisions=4),
        SUN_COLOR,
    )
    scene.add_geometry(
        sun,
        geom_name="sun",
        node_name="sun",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    earth = create_earth(EARTH_RADIUS_AU)
    scene.add_geometry(
        earth,
        transform=_transform_matrix(earth_rotation, earth_position),
        geom_name="earth",
        node_name="earth",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    orbit_path = trimesh.load_path(orbit)
    orbit_path.colors = np.tile(ORBIT_COLOR, (len(orbit_path.entities), 1))
    scene.add_geometry(
        orbit_path,
        geom_name="orbit",
        node_name="orbit",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    axis_start = (
        earth_position - spin_axis * AXIS_HALF_LENGTH_EARTH_RADII * EARTH_RADIUS_AU
    )
    axis_end = (
        earth_position + spin_axis * AXIS_HALF_LENGTH_EARTH_RADII * EARTH_RADIUS_AU
    )
    scene.add_geometry(
        _line_mesh(
            axis_start, axis_end, LINE_RADIUS_EARTH_RADII * EARTH_RADIUS_AU, AXIS_COLOR
        ),
        geom_name="earth_axis",
        node_name="earth_axis",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    netherlands_position = earth_position + netherlands_dir * EARTH_RADIUS_AU
    netherlands_marker = _color_mesh(
        trimesh.creation.icosphere(
            radius=NETHERLANDS_MARKER_EARTH_RADII * EARTH_RADIUS_AU,
            subdivisions=2,
        ),
        NETHERLANDS_COLOR,
    )
    scene.add_geometry(
        netherlands_marker,
        transform=_transform_matrix(np.eye(3), netherlands_position),
        geom_name="netherlands",
        node_name="netherlands",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    return scene


class _EarthCenteredViewer(SceneViewer):
    def __init__(
        self,
        scene: trimesh.Scene,
        earth_center: np.ndarray,
        camera_distance: float,
        **kwargs,
    ):
        self._earth_center = np.asarray(earth_center, dtype=float)
        scene.set_camera(center=self._earth_center, distance=camera_distance)
        scene.camera.z_near = CAMERA_Z_NEAR_AU
        scene.camera.z_far = CAMERA_Z_FAR_AU
        super().__init__(scene, **kwargs)

    def reset_view(self, flags=None):
        self.view = {
            "cull": True,
            "axis": False,
            "grid": False,
            "fullscreen": False,
            "wireframe": False,
            "ball": Trackball(
                pose=self._initial_camera_transform,
                size=self.scene.camera.resolution,
                scale=self.scene.scale,
                target=self._earth_center,
            ),
        }
        self.scene.camera_transform = self.view["ball"].pose
        if isinstance(flags, dict):
            for key, value in flags.items():
                if key in self.view:
                    self.view[key] = value
            self.update_flags()


def _line_mesh(
    start: np.ndarray,
    end: np.ndarray,
    radius: float,
    color: list[int],
) -> trimesh.Trimesh:
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    direction = end - start
    length = np.linalg.norm(direction)
    cylinder = trimesh.creation.cylinder(radius=radius, height=length, sections=10)
    direction /= length
    rotation = Rotation.align_vectors([direction], [np.array([0.0, 0.0, 1.0])])[0]
    transform = _transform_matrix(rotation.as_matrix(), (start + end) / 2)
    cylinder.apply_transform(transform)
    return _color_mesh(cylinder, color)


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
