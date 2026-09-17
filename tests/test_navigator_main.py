"""Navigator pointing helpers without hardware."""

from pybricks.parameters import Port

from navigator import navigator_main as nav
from simulate.astronomy.constants import PointingTarget
from simulate.astronomy.utils.ephemeris import (
    current_time,
    observer_surface_vector_and_euler_angles_for_target,
)


def test_pointing_would_move_when_no_prior_angles() -> None:
    nav._last_target_angles.clear()
    assert nav.pointing_would_move(PointingTarget.EARTH_ROTATION)


def test_pointing_would_move_false_when_within_delta() -> None:
    nav._last_target_angles.clear()
    target = PointingTarget.EARTH_ROTATION
    _surface, (yaw, pitch, _roll), _speed = observer_surface_vector_and_euler_angles_for_target(
        current_time(), target
    )
    yaw = nav.clamp_yaw(yaw)
    pitch = nav.clamp_pitch(pitch)
    nav._last_target_angles[Port.A.name] = yaw
    nav._last_target_angles[nav.EXTERNAL_MOTOR_PORT.name] = pitch
    assert not nav.pointing_would_move(target)


def test_pointing_would_move_when_pan_drift_exceeds_delta() -> None:
    nav._last_target_angles.clear()
    target = PointingTarget.EARTH_ROTATION
    _surface, (yaw, pitch, _roll), _speed = observer_surface_vector_and_euler_angles_for_target(
        current_time(), target
    )
    yaw = nav.clamp_yaw(yaw)
    pitch = nav.clamp_pitch(pitch)
    nav._last_target_angles[Port.A.name] = yaw - nav.MIN_TARGET_ANGLE_DELTA
    nav._last_target_angles[nav.EXTERNAL_MOTOR_PORT.name] = pitch
    assert nav.pointing_would_move(target)
