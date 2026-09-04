from enum import StrEnum

import numpy as np
from astropy import units as u

# FF
OBSERVER_LAT = 49.5 * u.deg
OBSERVER_LON = 6.6 * u.deg

# Netherlands
# OBSERVER_LAT = 52.1326 * u.deg
# OBSERVER_LON = 5.2913 * u.deg

KPC_TO_AU = (1 * u.kpc).to_value(u.au)

# ARON get this from the astropy constants?
REAL_SUN_RADIUS_AU = 695_700 / 149_597_870.7
REAL_EARTH_RADIUS_AU = 6_371 / 149_597_870.7
REAL_MOON_RADIUS_AU = 1_737.4 / 149_597_870.7

# SUN_SIZE_EXAGGERATION = 20
# EARTH_SIZE_EXAGGERATION = 20
SUN_SIZE_EXAGGERATION = 1
EARTH_SIZE_EXAGGERATION = 1
MOON_SIZE_EXAGGERATION = 1
GALACTIC_ORBIT_DISTANCE_SCALE = 1e-7
# GALACTIC_ORBIT_DISTANCE_SCALE = 1e-9

EARTH_ORBIT_SEGMENTS = 360 * 5

SUN_RADIUS_AU = REAL_SUN_RADIUS_AU * SUN_SIZE_EXAGGERATION
EARTH_RADIUS_AU = REAL_EARTH_RADIUS_AU * EARTH_SIZE_EXAGGERATION
MOON_RADIUS_AU = REAL_MOON_RADIUS_AU * MOON_SIZE_EXAGGERATION

AXIS_HALF_LENGTH_EARTH_RADII = 2.8
OBSERVER_MARKER_EARTH_RADII = 0.16
CAMERA_DISTANCE_EARTH_RADII = 80
AXIS_HALF_LENGTH_CAMERA_DISTANCE_FRACTION = AXIS_HALF_LENGTH_EARTH_RADII / CAMERA_DISTANCE_EARTH_RADII
AXIS_MIN_TOTAL_LENGTH_EARTH_DIAMETERS = 2
# Pan/zoom sensitivity; keep near the solar-system scale, not galactic bounds.
TRACKBALL_SCALE_AU = 2.0
GALACTIC_AXIS_HALF_LENGTH_ORBIT_FRACTION = 0.05
GALACTIC_CENTER_RADIUS_ORBIT_FRACTION = 0.003

# Default Trimesh z_near is 0.01 AU, larger than exaggerated Earth (~0.0009 AU).
CAMERA_Z_NEAR_EARTH_RADII = 0.01
# z_far = camera distance + scene.scale * this multiplier (see viewer clip planes).
CAMERA_Z_FAR_SCENE_SCALE_MULTIPLIER = 2.0
# gluPerspective rejects extreme z_far / z_near ratios (~1e8 at true scale).
MAX_DEPTH_RATIO = 5e7
# macOS OpenGL rejects z_near below ~1e-5 AU.
OPENGL_Z_NEAR_MIN_AU = 1e-5

EARTH_ORBIT_COLOR = [180, 180, 200, 255]
YEAR_BOUNDARY_COLOR = [255, 220, 80, 255]
SUN_COLOR = [255, 210, 60, 255]
MOON_COLOR = [210, 210, 205, 255]
AXIS_COLOR = [220, 60, 60, 255]
OBSERVER_COLOR = [255, 80, 40, 255]
OBSERVER_VELOCITY_ARROW_COLOR = [80, 220, 255, 255]
OBSERVER_VELOCITY_ARROW_LENGTH_EARTH_RADII = 4.0
OBSERVER_VELOCITY_ARROW_LENGTH_CAMERA_DISTANCE_FRACTION = (
    OBSERVER_VELOCITY_ARROW_LENGTH_EARTH_RADII / CAMERA_DISTANCE_EARTH_RADII
)
OBSERVER_VELOCITY_ARROW_MIN_TOTAL_LENGTH_EARTH_DIAMETERS = 2
OBSERVER_VELOCITY_ARROW_SHAFT_RADIUS_EARTH_RADII = 0.04
OBSERVER_VELOCITY_ARROW_HEAD_RADIUS_EARTH_RADII = 0.12
OBSERVER_VELOCITY_ARROW_HEAD_LENGTH_FRACTION = 0.35
SOLAR_GALACTIC_ORBITAL_SPEED = 220 * u.km / u.s
SIDEREAL_DAY = 86164.0905 * u.s

# Solar-system velocity relative to the CMB rest frame (Planck 2018 dipole).
# This already includes the orbit of the sun around the galactic center, so watch out for double-counting.
CMB_DIPOLE_SPEED = 369.82 * u.km / u.s
CMB_DIPOLE_L = 264.021 * u.deg
CMB_DIPOLE_B = 48.253 * u.deg


class PointingTarget(StrEnum):
    EARTH_ROTATION = "earth_rotation"
    SUN_ORBIT = "sun_orbit"
    MILKY_WAY_ORBIT = "milky_way_orbit"
    CMB_DIPOLE = "cmb_dipole"
    SUN = "sun"
    MOON = "moon"
    MILKY_WAY_CENTER = "milky_way_center"

    @property
    def label(self) -> str:
        return _POINTING_TARGET_LABELS[self]


_POINTING_TARGET_LABELS = {
    PointingTarget.EARTH_ROTATION: "Earth rotation",
    PointingTarget.SUN_ORBIT: "Sun orbit",
    PointingTarget.MILKY_WAY_ORBIT: "Milky Way orbit",
    PointingTarget.CMB_DIPOLE: "CMB dipole",
    PointingTarget.SUN: "Sun",
    PointingTarget.MOON: "Moon",
    PointingTarget.MILKY_WAY_CENTER: "Milky Way center",
}
POINTING_TARGET_BUTTON_MARGIN = 8
POINTING_TARGET_BUTTON_WIDTH = 200
POINTING_TARGET_BUTTON_HEIGHT = 32
GALACTIC_ORBIT_COLOR = [120, 80, 180, 255]
GALACTIC_AXIS_COLOR = [180, 120, 255, 255]
GALACTIC_CENTER_COLOR = [240, 200, 255, 255]
CMB_DIPOLE_ARROW_COLOR = [255, 200, 80, 255]
LABEL_OFFSET_BODY_RADII = 2.5
LABEL_FONT_SIZE = 18
FPS_LABEL_MARGIN = 8
LINE_WIDTH_PIXELS = 1

MILKY_WAY_FRAME = "milky_way"
SOLAR_SYSTEM_FRAME = "solar_system"
EARTH_CENTER_FRAME = "earth_center"
# Trimesh scene base/world frame: fixed display origin for the camera and trackball.
# No astronomy coordinates live here; earth_center is its only child and recenters
# the subtree so Earth stays at EARTH_CENTER_ORIGIN.
ROOT_FRAME = "root_frame"
EARTH_CENTER_ORIGIN = np.zeros(3)

ANIMATION_CALLBACK_PERIOD = 1.0 / 60.0
ORBIT_UPDATE_INTERVAL = 10 * u.day
