import numpy as np
import trimesh

from simulate.astronomy.constants import (
    CAMERA_Z_FAR_SCENE_SCALE_MULTIPLIER,
    CAMERA_Z_NEAR_EARTH_RADII,
    EARTH_CENTER_ORIGIN,
    EARTH_RADIUS_AU,
    MAX_DEPTH_RATIO,
    OPENGL_Z_NEAR_MIN_AU,
)


def camera_distance_au(scene: trimesh.Scene) -> float:
    return camera_distance_to_point_au(scene, EARTH_CENTER_ORIGIN)


def camera_distance_to_point_au(scene: trimesh.Scene, point: np.ndarray) -> float:
    eye = scene.camera_transform[:3, 3]
    return float(np.linalg.norm(eye - point))


def camera_clip_planes(
    scene: trimesh.Scene,
    *,
    camera_distance: float | None = None,
) -> tuple[float, float]:
    if camera_distance is None:
        camera_distance = camera_distance_au(scene)
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
