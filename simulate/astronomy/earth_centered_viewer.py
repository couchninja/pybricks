from ctypes import byref
from time import perf_counter
from typing import Any, NamedTuple, TypedDict, override

import numpy as np
import pyglet
import trimesh
from pyglet import gl, text
from pyglet.gl.glu import gluProject
from trimesh.viewer.trackball import Trackball
from trimesh.viewer.windowed import SceneViewer

from simulate.astronomy.constants import (
    CAMERA_Z_FAR_SCENE_SCALE_MULTIPLIER,
    CAMERA_Z_NEAR_EARTH_RADII,
    EARTH_RADIUS_AU,
    EARTH_VIEW_ORIGIN,
    FPS_LABEL_MARGIN,
    LABEL_FONT_SIZE,
    LABEL_OFFSET_BODY_RADII,
    MAX_DEPTH_RATIO,
    OPENGL_Z_NEAR_MIN_AU,
    TRACKBALL_SCALE_AU,
)

try:
    # macOS focuses inactive windows without delivering the first mouse click.
    # Teach the Cocoa view to accept the first click so click-drag rotation works
    # immediately (scroll already works because it doesn't require mouse capture).
    from pyglet.window.cocoa.pyglet_view import PygletView_Implementation

    @PygletView_Implementation.PygletView.method("B@")
    def acceptsFirstMouse_(self, _nsevent) -> bool:
        return True
except Exception:
    pass


class _EarthCenteredViewerState(TypedDict):
    screen_labels: list[text.Label]
    fps_label: text.Label
    fps_frames: int
    fps_interval_start: float
    fps_display: float


class _BodyScreenLabel(NamedTuple):
    node_name: str
    caption: str
    color: tuple[int, int, int, int]


_BODY_SCREEN_LABELS = (
    _BodyScreenLabel("sun", "Sun", (255, 210, 60, 255)),
    _BodyScreenLabel("earth", "Earth", (255, 255, 255, 255)),
    _BodyScreenLabel("galactic_center", "Milky Way center", (240, 200, 255, 255)),
)


class EarthCenteredViewer(SceneViewer):
    _state: _EarthCenteredViewerState

    @override
    def __init__(
        self,
        scene: trimesh.Scene,
        camera_distance: float,
        **kwargs,
    ):
        self._state: _EarthCenteredViewerState = {
            "screen_labels": [
                text.Label(
                    text=body.caption,
                    font_size=LABEL_FONT_SIZE,
                    color=body.color,
                    anchor_x="center",
                    anchor_y="bottom",
                )
                for body in _BODY_SCREEN_LABELS
            ],
            "fps_label": text.Label(
                "0 FPS",
                font_size=LABEL_FONT_SIZE,
                color=(255, 255, 255, 200),
                anchor_x="left",
                anchor_y="top",
            ),
            "fps_frames": 0,
            "fps_interval_start": perf_counter(),
            "fps_display": 0.0,
        }
        scene.set_camera(center=EARTH_VIEW_ORIGIN, distance=camera_distance)

        # Without this, the viewer will show a black screen until any mouse or keyboard input.
        kwargs["start_loop"] = False
        super().__init__(scene, **kwargs)
        self._sync_camera_clip_planes()
        pyglet.clock.schedule_once(self._initial_draw, 0)
        pyglet.app.run()

    @override
    def on_show(self) -> None:
        self.activate()

    @override
    def on_draw(self) -> None:
        super().on_draw()
        self._update_caption()
        self._update_fps()
        self._draw_screen_labels()
        self._sync_camera_clip_planes()

    @override
    def on_mouse_scroll(self, x: int, y: int, dx: float, dy: float) -> None:
        super().on_mouse_scroll(x, y, dx, dy)
        self._sync_camera_clip_planes()

    @override
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

        self.dispatch_event("on_draw")
        self.flip()

    def _update_caption(self) -> None:
        animation = self.scene.metadata.get("earth_sun_animation")
        if animation is None:
            return
        current_time = animation["current_time"]
        if current_time is None:
            return
        self.set_caption(f"Earth-sun ({current_time.iso})")

    def _update_fps(self) -> None:
        self._state["fps_frames"] += 1
        elapsed = perf_counter() - self._state["fps_interval_start"]
        if elapsed < 1.0:
            return
        self._state["fps_display"] = self._state["fps_frames"] / elapsed
        self._state["fps_frames"] = 0
        self._state["fps_interval_start"] = perf_counter()

    def _draw_screen_labels(self) -> None:
        width, height = self.get_viewport_size()
        gl.glDisable(gl.GL_DEPTH_TEST)

        screen_positions = [
            _world_to_screen_gl(_label_world_position(self.scene, body.node_name)) for body in _BODY_SCREEN_LABELS
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
        for label, screen in zip(self._state["screen_labels"], screen_positions, strict=True):
            if screen is None:
                label.visible = False
                continue
            label.visible = True
            label.x = int(screen[0])
            label.y = int(screen[1])
            label.draw()
        fps_label = self._state["fps_label"]
        fps_label.text = f"{self._state['fps_display']:.0f} FPS"
        fps_label.x = FPS_LABEL_MARGIN
        fps_label.y = height - FPS_LABEL_MARGIN
        fps_label.draw()
        gl.glDisable(gl.GL_BLEND)

        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glEnable(gl.GL_DEPTH_TEST)


def _camera_clip_planes(scene: trimesh.Scene) -> tuple[float, float]:
    eye = scene.camera_transform[:3, 3]
    camera_distance = float(np.linalg.norm(eye - EARTH_VIEW_ORIGIN))
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


def _label_world_position(scene: trimesh.Scene, node_name: str) -> np.ndarray:
    transform, geometry_name = scene.graph.get(node_name, "world")
    mesh = scene.geometry[geometry_name]
    radius = mesh.bounding_sphere.primitive.radius
    offset = np.array([0.0, 0.0, radius * LABEL_OFFSET_BODY_RADII, 1.0])
    return (transform @ offset)[:3]


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
