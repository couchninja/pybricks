from __future__ import annotations

import asyncio
import time
from collections.abc import AsyncIterator, Awaitable, Callable
from contextlib import asynccontextmanager, suppress
from typing import TypedDict

import gpiod
from gpiod.line import Bias, Direction, Edge, Value
from pybricks.parameters import Color

from simulate.astronomy.constants import ObserverFrame

CHIP = "/dev/gpiochip0"
DEBOUNCE_S = 0.03
BLINK_PERIOD_S = 0.4


class ButtonData(TypedDict):
    button_pin: int
    led_pin: int
    frame: ObserverFrame
    color: Color


ButtonListener = Callable[[ButtonData], Awaitable[None]]


BUTTONS: tuple[ButtonData, ...] = (
    {
        "button_pin": 27,
        "led_pin": 17,
        "frame": ObserverFrame.EARTH_ROTATION,
        "color": Color.BLUE,
    },
    {
        "button_pin": 22,
        "led_pin": 23,
        "frame": ObserverFrame.SUN_ORBIT,
        "color": Color.RED,
    },
    {
        "button_pin": 24,
        "led_pin": 25,
        "frame": ObserverFrame.MILKY_WAY_ORBIT,
        "color": Color.GREEN,
    },
    {
        "button_pin": 5,
        "led_pin": 6,
        "frame": ObserverFrame.CMB_DIPOLE,
        "color": Color.RED,
    },
    {
        "button_pin": 12,
        "led_pin": 16,
        "frame": ObserverFrame.CMB_DIPOLE,
        "color": Color.WHITE,
    },
)


class ButtonMenu:
    def __init__(
        self,
        buttons: tuple[ButtonData, ...] = BUTTONS,
        *,
        chip: str = CHIP,
        debounce_s: float = DEBOUNCE_S,
    ) -> None:
        self._buttons = buttons
        self._chip = chip
        self._debounce_s = debounce_s
        self._button_index_by_pin = {
            button["button_pin"]: index for index, button in enumerate(buttons)
        }
        self._selected_index = 0
        self._selection_listeners: list[ButtonListener] = []
        self._request: gpiod.LineRequest | None = None

    def __enter__(self) -> ButtonMenu:
        self._request = gpiod.request_lines(
            self._chip,
            consumer="starpoint-buttons",
            config=self._line_config(),
        )
        return self

    def __exit__(self, *exc: object) -> None:
        if self._request is not None:
            self._request.release()
            self._request = None

    @property
    def buttons(self) -> tuple[ButtonData, ...]:
        return self._buttons

    @property
    def selected_index(self) -> int:
        return self._selected_index

    @property
    def selected_button(self) -> ButtonData:
        return self._buttons[self._selected_index]

    def on_selection_changed(self, listener: ButtonListener) -> None:
        self._selection_listeners.append(listener)

    @asynccontextmanager
    async def blinking(
        self, index: int, *, period_s: float = BLINK_PERIOD_S
    ) -> AsyncIterator[None]:
        led_pin = self._buttons[index]["led_pin"]
        task = asyncio.create_task(self._blink_led(led_pin, period_s))
        try:
            yield
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            self._set_led(led_pin, False)

    async def run(self) -> None:
        self._show_selection(self._selected_index)
        last_event_s = {button["button_pin"]: 0.0 for button in self._buttons}

        while True:
            request = self._require_request()
            events = await asyncio.to_thread(
                lambda: list(request.read_edge_events())
            )
            if not events:
                await asyncio.sleep(0.01)
                continue

            for event in events:
                await self._handle_event(event, last_event_s)

    async def _blink_led(self, led_pin: int, period_s: float) -> None:
        on = False
        while True:
            on = not on
            self._set_led(led_pin, on)
            await asyncio.sleep(period_s)

    async def _handle_event(
        self,
        event: gpiod.EdgeEvent,
        last_event_s: dict[int, float],
    ) -> None:
        button_pin = event.line_offset
        index = self._button_index_by_pin.get(button_pin)
        if index is None:
            return

        now = time.monotonic()
        if now - last_event_s[button_pin] < self._debounce_s:
            return
        last_event_s[button_pin] = now

        if index == self._selected_index:
            return

        self._selected_index = index
        self._show_selection(index)
        button = self._buttons[index]
        for listener in self._selection_listeners:
            await listener(button)

    def _show_selection(self, selected_index: int) -> None:
        for index, button in enumerate(self._buttons):
            self._set_led(button["led_pin"], index == selected_index)

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
