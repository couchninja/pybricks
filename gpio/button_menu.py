from __future__ import annotations

import asyncio
import time
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager, suppress
from typing import TypedDict

import gpiod
from gpiod.line import Bias, Direction, Edge, Value
from pybricks.parameters import Color

from simulate.astronomy.constants import PointingTarget

CHIP = "/dev/gpiochip0"
BLINK_PERIOD_S = 0.4


class ButtonMode(TypedDict):
    target: PointingTarget
    color: Color


class ButtonConfig(TypedDict):
    button_pin: int
    led_pin: int
    modes: tuple[ButtonMode, ...]


class ButtonData(TypedDict):
    button_pin: int
    led_pin: int
    target: PointingTarget
    color: Color


BUTTONS: tuple[ButtonConfig, ...] = (
    {
        "button_pin": 27,
        "led_pin": 17,
        "modes": (
            {"target": PointingTarget.SUN, "color": Color.RED},
            {"target": PointingTarget.MOON, "color": Color.BLUE},
            {"target": PointingTarget.MILKY_WAY_CENTER, "color": Color.GREEN},
        ),
    },
    {
        "button_pin": 22,
        "led_pin": 23,
        "modes": ({"target": PointingTarget.EARTH_ROTATION, "color": Color.BLUE},),
    },
    {
        "button_pin": 24,
        "led_pin": 25,
        "modes": ({"target": PointingTarget.SUN_ORBIT, "color": Color.RED},),
    },
    {
        "button_pin": 5,
        "led_pin": 6,
        "modes": ({"target": PointingTarget.MILKY_WAY_ORBIT, "color": Color.GREEN},),
    },
    {
        "button_pin": 12,
        "led_pin": 16,
        "modes": ({"target": PointingTarget.CMB_DIPOLE, "color": Color.RED},),
    },
)

DEFAULT_BUTTON_INDEX = 1


class ButtonMenu:
    """Single-selection button panel: one LED lit for the selected target."""

    def __init__(
        self,
        buttons: tuple[ButtonConfig, ...] = BUTTONS,
        *,
        chip: str = CHIP,
    ) -> None:
        self._buttons = buttons
        self._chip = chip
        self._index_by_pin = {button["button_pin"]: index for index, button in enumerate(buttons)}
        self._mode_indices = [0] * len(buttons)
        self._selected_index = DEFAULT_BUTTON_INDEX
        self._request: gpiod.LineRequest | None = None

    def __enter__(self) -> ButtonMenu:
        self._request = gpiod.request_lines(
            self._chip,
            consumer="navigator-buttons",
            config=self._line_config(),
        )
        self.reset()
        return self

    def __exit__(self, *exc: object) -> None:
        if self._request is not None:
            for button in self._buttons:
                self._set_led(button["led_pin"], False)
            self._request.release()
            self._request = None

    @property
    def selected_button(self) -> ButtonData:
        index = self._selected_index
        button = self._buttons[index]
        mode = button["modes"][self._mode_indices[index]]
        return {
            "button_pin": button["button_pin"],
            "led_pin": button["led_pin"],
            "target": mode["target"],
            "color": mode["color"],
        }

    def reset(self) -> None:
        """Select the default button and put every button back on its first mode."""
        self._selected_index = DEFAULT_BUTTON_INDEX
        self._mode_indices = [0] * len(self._buttons)
        self._show_selection()

    async def wait_for_selection(self, *, timeout_s: float) -> None:
        """Wait for a press or the timeout, discarding presses made before this call.

        A press on the selected button advances it to its next mode; a press on any
        other button selects it.
        """
        request = self._require_request()
        await asyncio.to_thread(self._drain_edge_events)
        deadline = time.monotonic() + timeout_s

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            if not await asyncio.to_thread(request.wait_edge_events, remaining):
                return
            for event in request.read_edge_events():
                index = self._index_by_pin.get(event.line_offset)
                if index is not None:
                    self._select(index)
                    return

    @asynccontextmanager
    async def blinking_selected(self) -> AsyncIterator[None]:
        """Blink the selected LED while the navigator is busy."""
        led_pin = self._buttons[self._selected_index]["led_pin"]
        task = asyncio.create_task(self._blink_led(led_pin))
        try:
            yield
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            self._show_selection()

    def _select(self, index: int) -> None:
        if index == self._selected_index:
            modes = self._buttons[index]["modes"]
            self._mode_indices[index] = (self._mode_indices[index] + 1) % len(modes)
        else:
            self._selected_index = index
        self._show_selection()

    def _drain_edge_events(self) -> None:
        request = self._require_request()
        while request.wait_edge_events(timeout=0):
            request.read_edge_events()

    async def _blink_led(self, led_pin: int) -> None:
        on = False
        while True:
            on = not on
            self._set_led(led_pin, on)
            await asyncio.sleep(BLINK_PERIOD_S)

    def _show_selection(self) -> None:
        for index, button in enumerate(self._buttons):
            self._set_led(button["led_pin"], index == self._selected_index)

    def _set_led(self, led_pin: int, on: bool) -> None:
        self._require_request().set_value(
            led_pin, Value.ACTIVE if on else Value.INACTIVE
        )

    def _require_request(self) -> gpiod.LineRequest:
        if self._request is None:
            raise RuntimeError("ButtonMenu must be used as a context manager")
        return self._request

    def _line_config(self) -> dict[int, gpiod.LineSettings]:
        config: dict[int, gpiod.LineSettings] = {}
        for button in self._buttons:
            config[button["button_pin"]] = gpiod.LineSettings(
                direction=Direction.INPUT,
                bias=Bias.PULL_UP,
                edge_detection=Edge.FALLING,
            )
            config[button["led_pin"]] = gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE,
            )
        return config
