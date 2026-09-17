import json

import pytest

from gpio.button_menu import ButtonMenu
from navigator.web_ui import LogBuffer, _status_payload, capture_stdout
from simulate.astronomy.constants import PointingTarget


@pytest.fixture
def menu_without_gpio(monkeypatch) -> ButtonMenu:
    class FakeRequest:
        def set_value(self, pin: int, value: object) -> None:
            pass

        def release(self) -> None:
            pass

    monkeypatch.setattr(
        "gpio.button_menu.gpiod.request_lines",
        lambda *args, **kwargs: FakeRequest(),
    )
    menu = ButtonMenu()
    menu.__enter__()
    yield menu
    menu.__exit__(None, None, None)


def test_status_payload_includes_target_and_logs(menu_without_gpio: ButtonMenu) -> None:
    log_buffer = LogBuffer()
    with capture_stdout(log_buffer):
        print("hello navigator")
    payload = _status_payload(menu_without_gpio, log_buffer)
    assert payload["target"] == PointingTarget.EARTH_ROTATION.value
    assert payload["target_label"] == "Earth rotation"
    assert payload["speed_km_h"] is not None
    assert "hello navigator" in payload["logs"]


def test_status_payload_omits_speed_for_sun(menu_without_gpio: ButtonMenu) -> None:
    menu_without_gpio.select_target(PointingTarget.SUN)
    payload = _status_payload(menu_without_gpio, LogBuffer())
    assert payload["speed_km_h"] is None


def test_status_payload_json_serializable(menu_without_gpio: ButtonMenu) -> None:
    payload = _status_payload(menu_without_gpio, LogBuffer())
    json.dumps(payload)
