import numpy as np
import trimesh
from trimesh.visual.color import ColorVisuals

OCEAN_COLOR = np.array([20, 70, 150, 255], dtype=np.uint8)
LAND_COLOR = np.array([45, 130, 55, 255], dtype=np.uint8)
ICE_COLOR = np.array([235, 240, 245, 255], dtype=np.uint8)
DESERT_COLOR = np.array([170, 150, 90, 255], dtype=np.uint8)

UV_SPHERE_COUNT = (128, 64)


def create_earth(radius: float) -> trimesh.Trimesh:
    # uv_sphere drops faces at AU-scale radii; build at unit size then scale.
    mesh = trimesh.creation.uv_sphere(radius=1.0, count=list(UV_SPHERE_COUNT))
    centroids = mesh.triangles_center
    directions = centroids / np.linalg.norm(centroids, axis=1, keepdims=True)
    lat = np.arcsin(directions[:, 2])
    lon = np.arctan2(directions[:, 1], directions[:, 0])
    colors = np.asarray(_earth_face_colors(lat, lon), dtype=np.uint8)
    mesh.visual = ColorVisuals(mesh=mesh, face_colors=colors)
    mesh.apply_scale(radius)
    return mesh


def _earth_face_colors(lat: np.ndarray, lon: np.ndarray) -> np.ndarray:
    lat_deg = np.degrees(lat)
    lon_deg = np.degrees(lon)
    colors = np.tile(OCEAN_COLOR, (len(lat), 1))
    land = _land_mask(lat_deg, lon_deg)
    colors[land] = LAND_COLOR
    colors[_desert_mask(lat_deg, lon_deg)] = DESERT_COLOR
    colors[lat_deg >= 66] = ICE_COLOR
    colors[lat_deg <= -60] = ICE_COLOR
    return np.asarray(colors, dtype=np.uint8)


def _land_mask(lat_deg: np.ndarray, lon_deg: np.ndarray) -> np.ndarray:
    lon = _wrap_longitude(lon_deg)
    land = np.zeros(len(lat_deg), dtype=bool)
    land |= _region(lat_deg, lon, 49, 72, -170, -55)
    land |= _region(lat_deg, lon, 25, 49, -125, -65)
    land |= _region(lat_deg, lon, 15, 30, -115, -80)
    land |= _region(lat_deg, lon, 7, 20, -92, -77)
    land |= _region(lat_deg, lon, -56, 15, -82, -34)
    land |= _region(lat_deg, lon, 36, 72, -25, 45)
    land |= _region(lat_deg, lon, -35, 37, -18, 52)
    land |= _region(lat_deg, lon, 5, 77, 40, 145)
    land |= _region(lat_deg, lon, 5, 77, 145, 180)
    land |= _region(lat_deg, lon, 5, 77, -180, -168)
    land |= _region(lat_deg, lon, -45, -10, 112, 154)
    land |= _region(lat_deg, lon, 60, 84, -58, -20)
    land |= _region(lat_deg, lon, -47, -34, 166, 179)
    land |= _region(lat_deg, lon, -6, 6, 95, 141)
    land |= _region(lat_deg, lon, -26, -12, 43, 50)
    land |= _region(lat_deg, lon, 50, 59, -11, 2)
    land |= _region(lat_deg, lon, 30, 46, 129, 146)
    land |= _region(lat_deg, lon, 5, 22, 99, 109)
    land |= _region(lat_deg, lon, 6, 13, -5, 2)
    land |= _region(lat_deg, lon, 62, 67, 20, 32)
    land |= _region(lat_deg, lon, 76, 82, -70, -12)
    land |= lat_deg <= -62
    land &= ~_region(lat_deg, lon, 18, 30, -98, -82)
    land &= ~_region(lat_deg, lon, 53, 66, -90, -60)
    land &= ~_region(lat_deg, lon, 56, 66, 20, 45)
    land &= ~_region(lat_deg, lon, 12, 28, 32, 44)
    land &= ~_region(lat_deg, lon, 41, 46, 26, 42)
    return land


def _desert_mask(lat_deg: np.ndarray, lon_deg: np.ndarray) -> np.ndarray:
    lon = _wrap_longitude(lon_deg)
    desert = np.zeros(len(lat_deg), dtype=bool)
    desert |= _region(lat_deg, lon, 15, 35, -17, 40)
    desert |= _region(lat_deg, lon, 12, 32, 35, 55)
    desert |= _region(lat_deg, lon, 25, 37, 70, 90)
    desert |= _region(lat_deg, lon, 18, 30, 110, 125)
    desert |= _region(lat_deg, lon, -30, -20, 115, 130)
    return desert


def _region(
    lat_deg: np.ndarray,
    lon_deg: np.ndarray,
    lat_min: float,
    lat_max: float,
    lon_min: float,
    lon_max: float,
) -> np.ndarray:
    in_lat = (lat_deg >= lat_min) & (lat_deg <= lat_max)
    in_lon = (lon_deg >= lon_min) & (lon_deg <= lon_max)
    return in_lat & in_lon


def _wrap_longitude(lon_deg: np.ndarray) -> np.ndarray:
    return ((lon_deg + 180) % 360) - 180
