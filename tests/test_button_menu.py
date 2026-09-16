"""Button menu selection logic, exercised without GPIO hardware."""

import asyncio
from collections.abc import Iterator
from contextlib import contextmanager
from unittest.mock import patch

import gpiod
from gpiod.line import Value

from gpio.button_menu import BUTTONS, DEFAULT_BUTTON_INDEX, ButtonMenu
from simulate.astronomy.constants import PointingTarget

WAIT_TIMEOUT_S = 1.0
SUN_BUTTON_PIN = BUTTONS[0]["button_pin"]
SUN_LED_PIN = BUTTONS[0]["led_pin"]
DEFAULT_LED_PIN = BUTTONS[DEFAULT_BUTTON_INDEX]["led_pin"]


class FakeEvent:
    def __init__(self, line_offset: int) -> None:
        self.line_offset = line_offset


class FakeRequest:
    """Stand-in for gpiod.LineRequest, with a queue of falling edges to deliver."""

    def __init__(self) -> None:
        self.values: dict[int, Value] = {}
        self.queued: list[int] = []
        self.incoming: list[int] = []

    def set_value(self, pin: int, value: Value) -> None:
        self.values[pin] = value

    def wait_edge_events(self, timeout: float | None = None) -> bool:
        if timeout != 0:
            self.queued.extend(self.incoming)
            self.incoming.clear()
        return bool(self.queued)

    def read_edge_events(self) -> list[FakeEvent]:
        events = [FakeEvent(pin) for pin in self.queued]
        self.queued.clear()
        return events

    def release(self) -> None:
        self.values.clear()


@contextmanager
def fake_menu() -> Iterator[tuple[ButtonMenu, FakeRequest]]:
    request = FakeRequest()
    with patch.object(gpiod, "request_lines", return_value=request), ButtonMenu() as menu:
        yield menu, request


def lit_pins(request: FakeRequest) -> set[int]:
    return {pin for pin, value in request.values.items() if value == Value.ACTIVE}


def test_default_selection() -> None:
    with fake_menu() as (menu, request):
        assert menu.selected_button["target"] == PointingTarget.EARTH_ROTATION
        assert lit_pins(request) == {DEFAULT_LED_PIN}


async def test_first_press_points_at_sun() -> None:
    with fake_menu() as (menu, request):
        request.incoming.append(SUN_BUTTON_PIN)
        await menu.wait_for_selection(timeout_s=WAIT_TIMEOUT_S)
        assert menu.selected_button["target"] == PointingTarget.SUN
        assert lit_pins(request) == {SUN_LED_PIN}

        request.incoming.append(SUN_BUTTON_PIN)
        await menu.wait_for_selection(timeout_s=WAIT_TIMEOUT_S)
        assert menu.selected_button["target"] == PointingTarget.MOON

        request.incoming.append(SUN_BUTTON_PIN)
        await menu.wait_for_selection(timeout_s=WAIT_TIMEOUT_S)
        assert menu.selected_button["target"] == PointingTarget.MILKY_WAY_CENTER

        request.incoming.append(SUN_BUTTON_PIN)
        await menu.wait_for_selection(timeout_s=WAIT_TIMEOUT_S)
        assert menu.selected_button["target"] == PointingTarget.ISS


async def test_presses_while_busy_are_dropped() -> None:
    with fake_menu() as (menu, request):
        # Already in the queue when the menu starts waiting: pressed while moving.
        request.queued.append(SUN_BUTTON_PIN)
        await menu.wait_for_selection(timeout_s=WAIT_TIMEOUT_S)
        assert menu.selected_button["target"] == PointingTarget.EARTH_ROTATION
        assert lit_pins(request) == {DEFAULT_LED_PIN}


async def test_reset_restores_default() -> None:
    with fake_menu() as (menu, request):
        for _ in range(2):
            request.incoming.append(SUN_BUTTON_PIN)
            await menu.wait_for_selection(timeout_s=WAIT_TIMEOUT_S)
        assert menu.selected_button["target"] == PointingTarget.MOON

        menu.reset()
        assert menu.selected_button["target"] == PointingTarget.EARTH_ROTATION
        assert lit_pins(request) == {DEFAULT_LED_PIN}

        request.incoming.append(SUN_BUTTON_PIN)
        await menu.wait_for_selection(timeout_s=WAIT_TIMEOUT_S)
        assert menu.selected_button["target"] == PointingTarget.SUN


async def main() -> None:
    test_default_selection()
    await test_first_press_points_at_sun()
    await test_presses_while_busy_are_dropped()
    await test_reset_restores_default()
    print("button menu tests passed")


if __name__ == "__main__":
    asyncio.run(main())
