"""Navigator pointing helpers without hardware."""

from contextlib import nullcontext
from unittest.mock import AsyncMock, MagicMock, call, patch

import pytest
from pybricks.parameters import Color, Port

from navigator import navigator_main as nav
from navigator import system_clock
from simulate.astronomy.constants import PointingTarget
from simulate.astronomy.utils.ephemeris import (
    current_time,
    observer_surface_vector_and_euler_angles_for_target,
)


class LoopStopped(Exception):
    """Raised from a fake pointing call to break out of the endless selection loop."""


def fake_hub() -> MagicMock:
    hub = MagicMock()
    hub.color_distance_sensor.return_value = AsyncMock()
    return hub


def fake_buttons(target: PointingTarget) -> MagicMock:
    buttons = MagicMock()
    buttons.selected_button = {
        "button_pin": 27,
        "led_pin": 17,
        "target": target,
        "color": Color.RED,
    }
    buttons.wait_for_selection = AsyncMock()
    buttons.blinking_selected = MagicMock(side_effect=nullcontext)
    buttons.set_inputs_enabled = MagicMock()
    buttons.reset = MagicMock()
    buttons.panel_status = MagicMock(side_effect=nullcontext)
    return buttons


def wait_timeouts(buttons: MagicMock) -> list[float]:
    return [call.kwargs["timeout_s"] for call in buttons.wait_for_selection.await_args_list]


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
    fixed = current_time()
    _surface, (yaw, pitch, _roll), _speed = observer_surface_vector_and_euler_angles_for_target(
        fixed, target
    )
    yaw = nav.clamp_yaw(yaw)
    pitch = nav.clamp_pitch(pitch)
    nav._last_target_angles[Port.A.name] = yaw - nav.MIN_TARGET_ANGLE_DELTA - 1.0
    nav._last_target_angles[nav.EXTERNAL_MOTOR_PORT.name] = pitch
    with patch.object(nav, "current_time", return_value=fixed):
        assert nav.pointing_would_move(target)


async def test_pointing_suspended_until_clock_is_synchronized() -> None:
    hub = fake_hub()
    buttons = fake_buttons(PointingTarget.SUN)
    sync_states = iter([False, False, True])
    pointed: list[PointingTarget] = []

    async def record_point(_hub: object, _sensor: object, button: nav.ButtonData) -> None:
        pointed.append(button["target"])
        raise LoopStopped

    with (
        patch.object(nav, "clock_is_synchronized", side_effect=lambda: next(sync_states)),
        patch.object(nav, "pointing_would_move", return_value=True),
        patch.object(nav, "point_selected", side_effect=record_point),
        patch.object(nav, "wait_for_clock_sync_panel", new=AsyncMock()) as wait_panel,
        patch.object(nav, "resume_after_clock_sync") as resume,
        pytest.raises(LoopStopped),
    ):
        await nav.run_selection_loop(hub, buttons)

    # Nothing moved during the unsynchronized wait, then it pointed once.
    assert pointed == [PointingTarget.SUN]
    assert resume.call_count == 1
    assert wait_panel.await_count == 2
    assert buttons.set_inputs_enabled.call_args_list == [call(False), call(False), call(True)]
    assert buttons.reset.call_count == 1
    assert wait_timeouts(buttons) == []


async def test_pointing_starts_immediately_when_clock_is_synchronized() -> None:
    hub = fake_hub()
    buttons = fake_buttons(PointingTarget.SUN)

    async def record_point(_hub: object, _sensor: object, _button: nav.ButtonData) -> None:
        raise LoopStopped

    with (
        patch.object(nav, "clock_is_synchronized", return_value=True),
        patch.object(nav, "pointing_would_move", return_value=True),
        patch.object(nav, "point_selected", side_effect=record_point),
        patch.object(nav, "resume_after_clock_sync") as resume,
        pytest.raises(LoopStopped),
    ):
        await nav.run_selection_loop(hub, buttons)

    assert resume.call_count == 0
    assert wait_timeouts(buttons) == []


def test_resume_after_clock_sync_rearms_time_dependent_state() -> None:
    nav._last_target_angles[Port.A.name] = 123.0
    with (
        patch.object(nav, "refresh_iss_tle") as refresh,
        patch.object(nav, "reset_network_retry") as reset_retry,
    ):
        nav.resume_after_clock_sync()
    assert nav._last_target_angles == {}
    assert refresh.call_count == 1
    assert reset_retry.call_count == 1


def test_clock_is_synchronized_matches_kernel_state() -> None:
    assert isinstance(system_clock.clock_is_synchronized(), bool)


def test_clock_is_not_synchronized_when_kernel_reports_unsync() -> None:
    def unsynced(timex_ref: object) -> int:
        timex_ref._obj.status = system_clock._STA_UNSYNC
        return system_clock._TIME_ERROR

    with patch.object(system_clock, "_adjtimex", side_effect=unsynced):
        assert not system_clock.clock_is_synchronized()


def test_clock_check_raises_when_adjtimex_fails() -> None:
    with (
        patch.object(system_clock, "_adjtimex", return_value=-1),
        pytest.raises(OSError),
    ):
        system_clock.clock_is_synchronized()
