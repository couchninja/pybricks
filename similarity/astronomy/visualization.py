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

from ctypes import byref
from time import perf_counter
from typing import Any, Literal, NamedTuple, TypedDict, cast, overload
import warnings

import pyglet
from pyglet import gl, text
from pyglet.gl.glu import gluProject

import numpy as np
import trimesh
from astropy import units as u
from astropy.time import Time
from astropy.utils.exceptions import ErfaWarning
from scipy.spatial.transform import Rotation
from trimesh.visual.color import ColorVisuals
from trimesh.viewer.trackball import Trackball
from trimesh.viewer.windowed import SceneViewer

from similarity.astronomy.earth_mesh import create_earth
from similarity.astronomy.ephemeris import (
    current_time,
    earth_heliocentric_ecliptic_au,
    earth_orbit_ecliptic_au,
    earth_year_boundary_positions_ecliptic_au,
    earth_orientation_matrix,
    earth_spin_axis_ecliptic,
    ecliptic_to_galactocentric_rotation,
    netherlands_direction_ecliptic,
    sun_galactic_orbit_kpc,
    sun_galactocentric_kpc,
)

try:
    # macOS focuses inactive windows without delivering the first mouse click.
    # Teach the Cocoa view to accept the first click so click-drag rotation works
    # immediately (scroll already works because it doesn't require mouse capture).
    from pyglet.window.cocoa.pyglet_view import PygletView_Implementation

    @PygletView_Implementation.PygletView.method("B@")
    def acceptsFirstMouse_(self, _nsevent):
        return True
except Exception:
    pass

KPC_TO_AU = (1 * u.kpc).to_value(u.au)

# ARON get this from the astropy constants?
REAL_SUN_RADIUS_AU = 695_700 / 149_597_870.7
REAL_EARTH_RADIUS_AU = 6_371 / 149_597_870.7

# SUN_SIZE_EXAGGERATION = 20
# EARTH_SIZE_EXAGGERATION = 20
SUN_SIZE_EXAGGERATION = 1
EARTH_SIZE_EXAGGERATION = 1
GALACTIC_ORBIT_DISTANCE_SCALE = 1e-7
# GALACTIC_ORBIT_DISTANCE_SCALE = 1e-9

SUN_RADIUS_AU = REAL_SUN_RADIUS_AU * SUN_SIZE_EXAGGERATION
EARTH_RADIUS_AU = REAL_EARTH_RADIUS_AU * EARTH_SIZE_EXAGGERATION

AXIS_HALF_LENGTH_EARTH_RADII = 2.8
NETHERLANDS_MARKER_EARTH_RADII = 0.16
LINE_RADIUS_EARTH_DIAMETERS = 0.03
CAMERA_DISTANCE_EARTH_RADII = 80
AXIS_HALF_LENGTH_CAMERA_DISTANCE_FRACTION = (
    AXIS_HALF_LENGTH_EARTH_RADII / CAMERA_DISTANCE_EARTH_RADII
)
AXIS_RADIUS_CAMERA_DISTANCE_FRACTION = (
    LINE_RADIUS_EARTH_DIAMETERS * 2 / CAMERA_DISTANCE_EARTH_RADII
)
AXIS_MIN_TOTAL_LENGTH_EARTH_DIAMETERS = 2
# Pan/zoom sensitivity; keep near the solar-system scale, not galactic bounds.
TRACKBALL_SCALE_AU = 2.0
GALACTIC_AXIS_HALF_LENGTH_ORBIT_FRACTION = 0.05
GALACTIC_CENTER_RADIUS_ORBIT_FRACTION = 0.003
GALACTIC_LINE_RADIUS_ORBIT_FRACTION = 0.0005

# Default Trimesh z_near is 0.01 AU, larger than exaggerated Earth (~0.0009 AU).
CAMERA_Z_NEAR_EARTH_RADII = 0.01
# z_far = camera distance + scene.scale * this multiplier (see _camera_clip_planes).
CAMERA_Z_FAR_SCENE_SCALE_MULTIPLIER = 2.0
# gluPerspective rejects extreme z_far / z_near ratios (~1e8 at true scale).
MAX_DEPTH_RATIO = 5e7
# macOS OpenGL rejects z_near below ~1e-5 AU.
OPENGL_Z_NEAR_MIN_AU = 1e-5

ORBIT_COLOR = [180, 180, 200, 255]
YEAR_BOUNDARY_COLOR = [255, 220, 80, 255]
SUN_COLOR = [255, 210, 60, 255]
AXIS_COLOR = [220, 60, 60, 255]
NETHERLANDS_COLOR = [255, 80, 40, 255]
GALACTIC_ORBIT_COLOR = [120, 80, 180, 255]
GALACTIC_AXIS_COLOR = [180, 120, 255, 255]
GALACTIC_CENTER_COLOR = [240, 200, 255, 255]
LABEL_OFFSET_BODY_RADII = 2.5
LABEL_FONT_SIZE = 18

MILKY_WAY_FRAME = "milky_way"
SOLAR_SYSTEM_FRAME = "solar_system"
EARTH_VIEW_FRAME = "earth_view"
EARTH_VIEW_ORIGIN = np.zeros(3)

ANIMATION_CALLBACK_PERIOD = 1.0 / 60.0
ORBIT_UPDATE_INTERVAL = 1 * u.day


class EarthSunState(TypedDict):
    earth_position: np.ndarray
    earth_rotation: np.ndarray
    spin_axis: np.ndarray
    netherlands_position: np.ndarray
    galactic_rotation: np.ndarray
    sun_galactic_position: np.ndarray
    galactic_axis_half_length_au: float
    galactic_center_radius_au: float
    galactic_line_radius_au: float


class EarthSunStateWithOrbit(EarthSunState):
    orbit: np.ndarray
    galactic_orbit: np.ndarray


class _BodyScreenLabel(NamedTuple):
    node_name: str
    caption: str
    color: tuple[int, int, int, int]


_BODY_SCREEN_LABELS = (
    _BodyScreenLabel("sun", "Sun", (255, 210, 60, 255)),
    _BodyScreenLabel("earth", "Earth", (255, 255, 255, 255)),
    _BodyScreenLabel("galactic_center", "Milky Way center", (240, 200, 255, 255)),
)


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
    # Orbit paths are GL_LINES; see module docstring for why offset_lines must be False.
    scene.show(
        viewer=_EarthCenteredViewer,
        camera_distance=CAMERA_DISTANCE_EARTH_RADII * EARTH_RADIUS_AU,
        offset_lines=False,
        start_time=time,
        time_scaling=time_scaling,
    )


def build_earth_sun_scene(time: Time | None = None) -> trimesh.Scene:
    if time is None:
        time = current_time()

    state = _earth_sun_state(time)
    scene = trimesh.Scene()
    scene.graph.update(EARTH_VIEW_FRAME, "world", matrix=np.eye(4))
    scene.graph.update(MILKY_WAY_FRAME, EARTH_VIEW_FRAME, matrix=np.eye(4))
    scene.graph.update(
        SOLAR_SYSTEM_FRAME,
        MILKY_WAY_FRAME,
        matrix=_transform_matrix(
            state["galactic_rotation"], state["sun_galactic_position"]
        ),
    )

    galactic_center = _color_mesh(
        trimesh.creation.icosphere(
            radius=state["galactic_center_radius_au"], subdivisions=3
        ),
        GALACTIC_CENTER_COLOR,
    )
    scene.add_geometry(
        galactic_center,
        geom_name="galactic_center",
        node_name="galactic_center",
        parent_node_name=MILKY_WAY_FRAME,
    )

    galactic_orbit_path = trimesh.load_path(state["galactic_orbit"])
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
            np.array([0.0, 0.0, -state["galactic_axis_half_length_au"]]),
            np.array([0.0, 0.0, state["galactic_axis_half_length_au"]]),
            state["galactic_line_radius_au"],
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
        transform=_transform_matrix(state["earth_rotation"], state["earth_position"]),
        geom_name="earth",
        node_name="earth",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    orbit_path = trimesh.load_path(state["orbit"])
    orbit_path.colors = np.tile(ORBIT_COLOR, (len(orbit_path.entities), 1))
    scene.add_geometry(
        orbit_path,
        geom_name="orbit",
        node_name="orbit",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    scene.add_geometry(
        _year_boundary_path(time),
        geom_name="year_boundaries",
        node_name="year_boundaries",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    scene.add_geometry(
        _earth_axis_mesh(state, CAMERA_DISTANCE_EARTH_RADII * EARTH_RADIUS_AU),
        geom_name="earth_axis",
        node_name="earth_axis",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    netherlands_marker = _color_mesh(
        trimesh.creation.icosphere(
            radius=NETHERLANDS_MARKER_EARTH_RADII * EARTH_RADIUS_AU,
            subdivisions=2,
        ),
        NETHERLANDS_COLOR,
    )
    scene.add_geometry(
        netherlands_marker,
        transform=_transform_matrix(np.eye(3), state["netherlands_position"]),
        geom_name="netherlands",
        node_name="netherlands",
        parent_node_name=SOLAR_SYSTEM_FRAME,
    )

    _sync_earth_view_frame(scene, state)
    return scene


def update_earth_sun_scene(
    scene: trimesh.Scene,
    time: Time,
    last_orbit_time: Time | None,
    camera_distance_au: float,
) -> Time:
    state = _earth_sun_state(time, include_orbit=False)
    scene.graph.update(
        SOLAR_SYSTEM_FRAME,
        MILKY_WAY_FRAME,
        matrix=_transform_matrix(
            state["galactic_rotation"], state["sun_galactic_position"]
        ),
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
    scene.geometry["earth_axis"] = _earth_axis_mesh(state, camera_distance_au)
    _sync_earth_view_frame(scene, state)

    if last_orbit_time is None or abs(time - last_orbit_time) >= ORBIT_UPDATE_INTERVAL:
        orbit_state = _earth_sun_state(time, include_orbit=True)
        orbit_path = trimesh.load_path(orbit_state["orbit"])
        orbit_path.colors = np.tile(ORBIT_COLOR, (len(orbit_path.entities), 1))
        scene.geometry["orbit"] = orbit_path

        scene.geometry["year_boundaries"] = _year_boundary_path(time)

        galactic_orbit_path = trimesh.load_path(orbit_state["galactic_orbit"])
        galactic_orbit_path.colors = np.tile(
            GALACTIC_ORBIT_COLOR, (len(galactic_orbit_path.entities), 1)
        )
        scene.geometry["galactic_orbit"] = galactic_orbit_path
        return time

    return last_orbit_time


class _EarthCenteredViewer(SceneViewer):
    def __init__(
        self,
        scene: trimesh.Scene,
        camera_distance: float,
        start_time: Time,
        *,
        time_scaling: float = 1.0,
        **kwargs,
    ):
        self._start_time = start_time
        self._last_orbit_time: Time | None = None
        self._screen_labels: list[text.Label] | None = None
        self._time_scaling = time_scaling
        scene.set_camera(center=EARTH_VIEW_ORIGIN, distance=camera_distance)

        # Without this, the viewer will show a black screen until any mouse or keyboard input.
        kwargs["start_loop"] = False
        kwargs["callback"] = self._animate
        kwargs["callback_period"] = ANIMATION_CALLBACK_PERIOD
        kwargs["caption"] = f"Earth-sun ({start_time.iso})"
        super().__init__(scene, **kwargs)
        self._sync_camera_clip_planes()
        self._animation_start = perf_counter()
        pyglet.clock.schedule_once(self._initial_draw, 0)
        pyglet.app.run()

    def on_show(self) -> None:
        self.activate()

    def on_draw(self) -> None:
        super().on_draw()
        self._draw_screen_labels()

    def on_mouse_scroll(self, x: int, y: int, dx: float, dy: float) -> None:
        super().on_mouse_scroll(x, y, dx, dy)
        self._sync_camera_clip_planes()

    def reset_view(self, flags: dict[str, Any] | None = None) -> None:
        self.view = {
            "cull": True,
            "axis": False,
            "grid": False,
            "fullscreen": False,
            "wireframe": False,
            "ball": Trackball(
                pose=self._initial_camera_transform,
                size=self.scene.camera.resolution,
                scale=TRACKBALL_SCALE_AU,
                target=EARTH_VIEW_ORIGIN,
            ),
        }
        self.scene.camera_transform = self.view["ball"].pose
        if isinstance(flags, dict):
            for key, value in flags.items():
                if key in self.view:
                    self.view[key] = value
            self.update_flags()
        self._sync_camera_clip_planes()

    def _sync_camera_clip_planes(self) -> None:
        """Update ``z_near``/``z_far`` from camera distance and scene scale.

        Galactic geometry extends well beyond the solar system. A fixed far plane
        clipped distant objects when zooming out. Recompute the clip planes on
        zoom, animation, and view reset so the full scene stays visible.

        Perspective is refreshed only after the Cocoa window exists; during
        ``reset_view`` in ``super().__init__`` the clip values are set but
        ``gluPerspective`` waits until the first real resize/draw.
        """
        z_near, z_far = _camera_clip_planes(self.scene)
        if z_near == self.scene.camera.z_near and z_far == self.scene.camera.z_far:
            return
        self.scene.camera.z_near = z_near
        self.scene.camera.z_far = z_far
        if getattr(self, "_nswindow", None) is None:
            return
        width, height = self.get_size()
        if width > 0 and height > 0:
            self._update_perspective(width, height)

    def _initial_draw(self, _dt: float) -> None:
        """Draw the first frame after the window and OpenGL context exist.

        ``start_loop=False`` keeps pyglet from rendering until input arrives,
        which leaves a black screen on startup. This callback runs once on the
        next clock tick to set the viewport, create screen labels (they need an
        active GL context), and present the scene immediately.
        """
        self.activate()
        self.switch_to()
        if self.width > 0 and self.height > 0:
            self.dispatch_event("on_resize", self.width, self.height)
        self._sync_camera_clip_planes()
        self._ensure_screen_labels()
        self.dispatch_event("on_draw")
        self.flip()

    def _animate(self, scene: trimesh.Scene) -> None:
        elapsed = perf_counter() - self._animation_start
        time = self._start_time + elapsed * self._time_scaling * u.second
        self._last_orbit_time = update_earth_sun_scene(
            scene,
            time,
            self._last_orbit_time,
            _camera_distance_au(scene),
        )
        self._sync_camera_clip_planes()
        self.set_caption(f"Earth-sun ({time.iso})")

    def _ensure_screen_labels(self) -> list[text.Label]:
        if self._screen_labels is None:
            self._screen_labels = [
                text.Label(
                    body.caption,
                    font_size=LABEL_FONT_SIZE,
                    color=body.color,
                    anchor_x="center",
                    anchor_y="bottom",
                )
                for body in _BODY_SCREEN_LABELS
            ]
        return self._screen_labels

    def _draw_screen_labels(self) -> None:
        width, height = self.get_viewport_size()
        gl.glDisable(gl.GL_DEPTH_TEST)

        screen_positions = [
            _world_to_screen_gl(_label_world_position(self.scene, body.node_name))
            for body in _BODY_SCREEN_LABELS
        ]

        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPushMatrix()
        gl.glLoadIdentity()
        gl.gluOrtho2D(0, width, 0, height)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glPushMatrix()
        gl.glLoadIdentity()

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        for label, screen in zip(
            self._ensure_screen_labels(), screen_positions, strict=True
        ):
            if screen is None:
                label.visible = False
                continue
            label.visible = True
            label.x = int(screen[0])
            label.y = int(screen[1])
            label.draw()
        gl.glDisable(gl.GL_BLEND)

        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glEnable(gl.GL_DEPTH_TEST)


@overload
def _earth_sun_state(
    time: Time, *, include_orbit: Literal[True] = True
) -> EarthSunStateWithOrbit: ...


@overload
def _earth_sun_state(time: Time, *, include_orbit: Literal[False]) -> EarthSunState: ...


def _earth_sun_state(
    time: Time, *, include_orbit: bool = True
) -> EarthSunState | EarthSunStateWithOrbit:
    earth_position = earth_heliocentric_ecliptic_au(time)
    spin_axis = earth_spin_axis_ecliptic(time)
    netherlands_dir = netherlands_direction_ecliptic(time)

    sun_galactic_kpc = sun_galactocentric_kpc(time)
    sun_galactic_distance_kpc = np.linalg.norm(sun_galactic_kpc)
    galactic_scale = KPC_TO_AU * GALACTIC_ORBIT_DISTANCE_SCALE
    sun_galactic_distance_au = sun_galactic_distance_kpc * galactic_scale
    sun_galactic_position = sun_galactic_kpc * galactic_scale

    state: EarthSunState = {
        "earth_position": earth_position,
        "earth_rotation": earth_orientation_matrix(time),
        "spin_axis": spin_axis,
        "netherlands_position": earth_position + netherlands_dir * EARTH_RADIUS_AU,
        "galactic_rotation": ecliptic_to_galactocentric_rotation(time),
        "sun_galactic_position": sun_galactic_position,
        "galactic_axis_half_length_au": (
            sun_galactic_distance_au * GALACTIC_AXIS_HALF_LENGTH_ORBIT_FRACTION
        ),
        "galactic_center_radius_au": (
            sun_galactic_distance_au * GALACTIC_CENTER_RADIUS_ORBIT_FRACTION
        ),
        "galactic_line_radius_au": (
            sun_galactic_distance_au * GALACTIC_LINE_RADIUS_ORBIT_FRACTION
        ),
    }
    if include_orbit:
        orbit_state = cast(
            EarthSunStateWithOrbit,
            {
                **state,
                "orbit": earth_orbit_ecliptic_au(time),
                "galactic_orbit": sun_galactic_orbit_kpc(time) * galactic_scale,
            },
        )
        return orbit_state
    return state


def _sync_earth_view_frame(scene: trimesh.Scene, state: EarthSunState) -> None:
    earth_world = _earth_world_position(state)
    scene.graph.update(
        frame_to=EARTH_VIEW_FRAME,
        frame_from="world",
        matrix=_transform_matrix(np.eye(3), -earth_world),
    )


def _earth_world_position(state: EarthSunState) -> np.ndarray:
    solar_system = _transform_matrix(
        state["galactic_rotation"], state["sun_galactic_position"]
    )
    earth = _transform_matrix(state["earth_rotation"], state["earth_position"])
    return (solar_system @ earth)[:3, 3]


def _earth_axis_mesh(
    state: EarthSunState, camera_distance_au: float
) -> trimesh.Trimesh:
    earth_position = state["earth_position"]
    spin_axis = state["spin_axis"]
    axis_half_length = max(
        camera_distance_au * AXIS_HALF_LENGTH_CAMERA_DISTANCE_FRACTION,
        AXIS_MIN_TOTAL_LENGTH_EARTH_DIAMETERS * EARTH_RADIUS_AU,
    )
    axis_radius = camera_distance_au * AXIS_RADIUS_CAMERA_DISTANCE_FRACTION
    axis_start = earth_position - spin_axis * axis_half_length
    axis_end = earth_position + spin_axis * axis_half_length
    return _line_mesh(
        axis_start,
        axis_end,
        axis_radius,
        AXIS_COLOR,
    )


def _label_world_position(scene: trimesh.Scene, node_name: str) -> np.ndarray:
    transform, geometry_name = scene.graph.get(node_name, "world")
    mesh = scene.geometry[geometry_name]
    radius = mesh.bounding_sphere.primitive.radius
    offset = np.array([0.0, 0.0, radius * LABEL_OFFSET_BODY_RADII, 1.0])
    return (transform @ offset)[:3]


def _camera_distance_au(scene: trimesh.Scene) -> float:
    eye = scene.camera_transform[:3, 3]
    return float(np.linalg.norm(eye - EARTH_VIEW_ORIGIN))


def _camera_clip_planes(scene: trimesh.Scene) -> tuple[float, float]:
    camera_distance = _camera_distance_au(scene)
    try:
        scene_scale = float(scene.scale)
    except Exception:
        scene_scale = 1.0
    z_far = camera_distance + scene_scale * CAMERA_Z_FAR_SCENE_SCALE_MULTIPLIER
    z_near = max(
        EARTH_RADIUS_AU * CAMERA_Z_NEAR_EARTH_RADII,
        z_far / MAX_DEPTH_RATIO,
        OPENGL_Z_NEAR_MIN_AU,
    )
    return z_near, z_far


def _world_to_screen_gl(world: np.ndarray) -> tuple[float, float] | None:
    modelview = (gl.GLdouble * 16)()
    projection = (gl.GLdouble * 16)()
    viewport = (gl.GLint * 4)()
    gl.glGetDoublev(gl.GL_MODELVIEW_MATRIX, modelview)
    gl.glGetDoublev(gl.GL_PROJECTION_MATRIX, projection)
    gl.glGetIntegerv(gl.GL_VIEWPORT, viewport)

    win_x = gl.GLdouble()
    win_y = gl.GLdouble()
    win_z = gl.GLdouble()
    if not gluProject(
        gl.GLdouble(world[0]),
        gl.GLdouble(world[1]),
        gl.GLdouble(world[2]),
        modelview,
        projection,
        viewport,
        byref(win_x),
        byref(win_y),
        byref(win_z),
    ):
        return None
    if win_z.value < 0.0 or win_z.value > 1.0:
        return None
    return float(win_x.value), float(win_y.value)


def _year_boundary_path(time: Time) -> trimesh.path.Path3D:
    positions = earth_year_boundary_positions_ecliptic_au(time)
    if len(positions) == 0:
        # Trimesh's Scene.scale/bounds computation assumes geometry bounds exist.
        # Return a degenerate (zero-length) segment so the geometry is valid but invisible.
        sun = np.zeros(3, dtype=float)
        path = trimesh.load_path(np.array([[sun, sun]]))
        path.colors = np.tile(YEAR_BOUNDARY_COLOR, (len(path.entities), 1))
        return path
    segments = np.array(
        [[np.zeros(3, dtype=float), position] for position in positions]
    )
    path = trimesh.load_path(segments)
    path.colors = np.tile(YEAR_BOUNDARY_COLOR, (len(path.entities), 1))
    return path


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
    direction /= length
    rotation = Rotation.align_vectors([direction], [np.array([0.0, 0.0, 1.0])])[0]
    # Build at unit size then scale; trimesh drops faces on AU-scale cylinders.
    cylinder = trimesh.creation.cylinder(radius=1.0, height=1.0, sections=10)
    scale = np.diag([radius, radius, length, 1.0])
    transform = _transform_matrix(rotation.as_matrix(), (start + end) / 2) @ scale
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
