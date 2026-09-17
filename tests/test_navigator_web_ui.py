import json
from collections.abc import Iterator

import pytest

from gpio.button_menu_host import HostButtonMenu
from navigator.web_ui import (
    LogBuffer,
    _status_payload,
    begin_navigator_session,
    capture_stdout,
    navigator_session_id,
    web_port_for_platform,
)
from simulate.astronomy.constants import PointingTarget


@pytest.fixture
def menu_without_gpio() -> Iterator[HostButtonMenu]:
    menu = HostButtonMenu()
    menu.__enter__()
    yield menu
    menu.__exit__(None, None, None)


def test_status_payload_includes_target_and_logs(menu_without_gpio: HostButtonMenu) -> None:
    log_buffer = LogBuffer()
    with capture_stdout(log_buffer):
        print("hello navigator")
    payload = _status_payload(menu_without_gpio, log_buffer)
    assert payload["target"] == PointingTarget.EARTH_ROTATION.value
    assert payload["target_label"] == "Earth rotation"
    assert payload["speed_km_h"] is not None
    logs = payload["logs"]
    assert isinstance(logs, list)
    assert "hello navigator" in logs


def test_status_payload_omits_speed_for_sun(menu_without_gpio: HostButtonMenu) -> None:
    menu_without_gpio.select_target(PointingTarget.SUN)
    payload = _status_payload(menu_without_gpio, LogBuffer())
    assert payload["speed_km_h"] is None


def test_status_payload_json_serializable(menu_without_gpio: HostButtonMenu) -> None:
    payload = _status_payload(menu_without_gpio, LogBuffer())
    json.dumps(payload)


def test_web_port_differs_on_darwin() -> None:
    assert web_port_for_platform("darwin") == 18765
    assert web_port_for_platform("linux") == 8765


def test_navigator_session_changes_each_run() -> None:
    first = begin_navigator_session()
    second = begin_navigator_session()
    assert len(first) == 32
    assert len(second) == 32
    assert first != second
    assert navigator_session_id() == second
