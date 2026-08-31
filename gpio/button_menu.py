from __future__ import annotations

import asyncio
import time
from collections.abc import Awaitable, Callable
from typing import TypedDict

import gpiod
from gpiod.line import Bias, Direction, Edge, Value
from pybricks.parameters import Color

from simulate.astronomy.constants import ObserverFrame

CHIP = "/dev/gpiochip0"
DEBOUNCE_S = 0.03


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

    async def run(self) -> None:
        with gpiod.request_lines(
            self._chip,
            consumer="starpoint-buttons",
            config=self._line_config(),
        ) as request:
            self._show_selection(request, self._selected_index)
            last_event_s = {
                button["button_pin"]: 0.0 for button in self._buttons
            }

            while True:
                events = await asyncio.to_thread(
                    lambda: list(request.read_edge_events())
                )
                if not events:
                    await asyncio.sleep(0.01)
                    continue

                for event in events:
                    await self._handle_event(request, event, last_event_s)

    async def _handle_event(
        self,
        request: gpiod.LineRequest,
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
        self._show_selection(request, index)
        await self._notify_selection_changed(index)

    async def _notify_selection_changed(self, index: int) -> None:
        button = self._buttons[index]
        for listener in self._selection_listeners:
            await listener(button)

    def _show_selection(self, request: gpiod.LineRequest, selected_index: int) -> None:
        for index, button in enumerate(self._buttons):
            request.set_value(
                button["led_pin"],
                Value.ACTIVE if index == selected_index else Value.INACTIVE,
            )

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
