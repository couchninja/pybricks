import numpy as np
import trimesh
from astropy.time import Time
from scipy.spatial.transform import Rotation
from trimesh.visual.color import ColorVisuals
from trimesh.viewer.trackball import Trackball
from trimesh.viewer.windowed import SceneViewer

from similarity.astronomy.ephemeris import (
    current_time,
    earth_heliocentric_ecliptic_au,
    earth_orbit_ecliptic_au,
    earth_orientation_matrix,
    earth_spin_axis_ecliptic,
    netherlands_direction_ecliptic,
)

REAL_SUN_RADIUS_AU = 695_700 / 149_597_870.7
REAL_EARTH_RADIUS_AU = 6_371 / 149_597_870.7

SUN_SIZE_EXAGGERATION = 10
EARTH_SIZE_EXAGGERATION = 50
# SUN_SIZE_EXAGGERATION = 1
# EARTH_SIZE_EXAGGERATION = 1

SUN_RADIUS_AU = REAL_SUN_RADIUS_AU * SUN_SIZE_EXAGGERATION
EARTH_RADIUS_AU = REAL_EARTH_RADIUS_AU * EARTH_SIZE_EXAGGERATION

AXIS_HALF_LENGTH_EARTH_RADII = 2.8
NETHERLANDS_MARKER_EARTH_RADII = 0.16
LINE_RADIUS_EARTH_RADII = 0.06
CAMERA_DISTANCE_EARTH_RADII = 80
ORBIT_COLOR = [180, 180, 200, 255]
SUN_COLOR = [255, 210, 60, 255]
EARTH_COLOR = [40, 90, 180, 255]
AXIS_COLOR = [220, 60, 60, 255]
NETHERLANDS_COLOR = [255, 80, 40, 255]


def show_earth_sun(time: Time | None = None) -> None:
    if time is None:
        time = current_time()

    scene = build_earth_sun_scene(time)
    earth_position = earth_heliocentric_ecliptic_au(time)
    scene.apply_translation(-earth_position)
    scene.show(
        viewer=_EarthCenteredViewer,
        earth_center=np.zeros(3),
        camera_distance=CAMERA_DISTANCE_EARTH_RADII * EARTH_RADIUS_AU,
    )


def build_earth_sun_scene(time: Time | None = None) -> trimesh.Scene:
    if time is None:
        time = current_time()

    earth_position = earth_heliocentric_ecliptic_au(time)
    orbit = earth_orbit_ecliptic_au(time)
    spin_axis = earth_spin_axis_ecliptic(time)
    netherlands_dir = netherlands_direction_ecliptic(time)
    earth_rotation = earth_orientation_matrix(time)

    scene = trimesh.Scene()

    sun = _color_mesh(
        trimesh.creation.icosphere(radius=SUN_RADIUS_AU, subdivisions=4),
        SUN_COLOR,
    )
    scene.add_geometry(sun, geom_name="sun")

    earth = _color_mesh(
        trimesh.creation.icosphere(radius=EARTH_RADIUS_AU, subdivisions=4),
        EARTH_COLOR,
    )
    scene.add_geometry(
        earth,
        transform=_transform_matrix(earth_rotation, earth_position),
        geom_name="earth",
    )

    orbit_path = trimesh.load_path(orbit)
    orbit_path.colors = np.tile(ORBIT_COLOR, (len(orbit_path.entities), 1))
    scene.add_geometry(orbit_path, geom_name="orbit")

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
