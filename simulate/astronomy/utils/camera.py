import numpy as np
import trimesh

from simulate.astronomy.constants import EARTH_CENTER_ORIGIN


def camera_distance_au(scene: trimesh.Scene) -> float:
    return camera_distance_to_point_au(scene, EARTH_CENTER_ORIGIN)


def camera_distance_to_point_au(scene: trimesh.Scene, point: np.ndarray) -> float:
    eye = scene.camera_transform[:3, 3]
    return float(np.linalg.norm(eye - point))
