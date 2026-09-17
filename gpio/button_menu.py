from __future__ import annotations

import asyncio
import time
from collections.abc import AsyncIterator, Iterable
from contextlib import asynccontextmanager, suppress

import gpiod
from gpiod.line import Bias, Direction, Edge, Value

from gpio.button_config import (
    BUTTONS,
    CLOCK_UNSYNC_LED_PIN,
    DEFAULT_BUTTON_INDEX,
    HUB_CALIBRATE_LED_PIN,
    HUB_SEARCH_LED_PIN,
    ButtonConfig,
    ButtonData,
    HubLinkPhase,
    PanelStatus,
)
from simulate.astronomy.constants import PointingTarget

__all__ = [
    "BUTTONS",
    "ButtonConfig",
    "ButtonData",
    "ButtonMenu",
    "CLOCK_UNSYNC_LED_PIN",
    "DEFAULT_BUTTON_INDEX",
    "HUB_CALIBRATE_LED_PIN",
    "HUB_SEARCH_LED_PIN",
    "HubLinkPhase",
    "PanelStatus",
]

CHIP = "/dev/gpiochip0"
BLINK_PERIOD_S = 0.4

GpiodLineConfig = dict[Iterable[int | str] | int | str, gpiod.LineSettings | None]

_ALL_LED_PINS = frozenset(
    {button["led_pin"] for button in BUTTONS} | {CLOCK_UNSYNC_LED_PIN, HUB_SEARCH_LED_PIN, HUB_CALIBRATE_LED_PIN}
)


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
        self._wake = asyncio.Event()
        self._inputs_enabled = False
        self._status_mode = False
        self._panel_status = PanelStatus()

    def __enter__(self) -> ButtonMenu:
        self._request = gpiod.request_lines(
            self._chip,
            consumer="navigator-buttons",
            config=self._line_config(),
        )
        self._all_leds_off()
        return self

    def __exit__(self, *exc: object) -> None:
        if self._request is not None:
            self._all_leds_off()
            self._request.release()
            self._request = None

    def set_inputs_enabled(self, enabled: bool) -> None:
        self._inputs_enabled = enabled
        if not enabled:
            self._drain_edge_events()

    @property
    def inputs_enabled(self) -> bool:
        return self._inputs_enabled

    @asynccontextmanager
    async def panel_status(self) -> AsyncIterator[PanelStatus]:
        """Blink white / blue / orange status LEDs until ``panel_status`` exits."""
        self._status_mode = True
        self._panel_status = PanelStatus()
        self._all_leds_off()
        task = asyncio.create_task(self._status_led_loop())
        try:
            yield self._panel_status
        finally:
            self._status_mode = False
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            self._all_leds_off()

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
        if not self._status_mode:
            self._show_selection()

    def cycle_pointing_target(self) -> None:
        """Advance to the next ``PointingTarget`` (same order as the astronomy viewer)."""
        targets = list(PointingTarget)
        current = self.selected_button["target"]
        next_target = targets[(targets.index(current) + 1) % len(targets)]
        self.select_target(next_target)

    def select_target(self, target: PointingTarget) -> None:
        for index, button in enumerate(self._buttons):
            for mode_index, mode in enumerate(button["modes"]):
                if mode["target"] == target:
                    self._selected_index = index
                    self._mode_indices[index] = mode_index
                    if not self._status_mode:
                        self._show_selection()
                    self._wake.set()
                    return
        raise ValueError(f"No button mode for {target!r}")

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
            if self._wake.is_set():
                self._wake.clear()
                return
            slice_s = min(remaining, 0.2)
            if not await asyncio.to_thread(request.wait_edge_events, slice_s):
                continue
            if not self._inputs_enabled:
                self._drain_edge_events()
                continue
            for event in request.read_edge_events():
                index = self._index_by_pin.get(event.line_offset)
                if index is not None:
                    self._select(index)
                    return

    @asynccontextmanager
    async def blinking_selected(self) -> AsyncIterator[None]:
        """Blink the selected LED while the navigator is busy."""
        if self._status_mode:
            raise RuntimeError("panel_status and blinking_selected cannot overlap")
        self._show_selection()
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
        if not self._status_mode:
            self._show_selection()

    def _drain_edge_events(self) -> None:
        request = self._require_request()
        while request.wait_edge_events(timeout=0):
            request.read_edge_events()

    async def _blink_led(self, led_pin: int) -> None:
        self._set_led(led_pin, True)
        on = True
        while True:
            await asyncio.sleep(BLINK_PERIOD_S)
            on = not on
            self._set_led(led_pin, on)

    async def _status_led_loop(self) -> None:
        while True:
            status_pins = self._active_status_pins(self._panel_status)
            if not status_pins:
                await asyncio.sleep(BLINK_PERIOD_S / 4)
                continue
            self._set_status_leds(status_pins, True)
            await asyncio.sleep(BLINK_PERIOD_S)
            self._set_status_leds(status_pins, False)
            await asyncio.sleep(BLINK_PERIOD_S)

    def _active_status_pins(self, status: PanelStatus) -> tuple[int, ...]:
        pins: list[int] = []
        if not status.clock_synchronized:
            pins.append(CLOCK_UNSYNC_LED_PIN)
        if status.hub == "disconnected":
            pins.append(HUB_SEARCH_LED_PIN)
        elif status.hub == "calibrating":
            pins.append(HUB_CALIBRATE_LED_PIN)
        return tuple(pins)

    def _set_status_leds(self, active_pins: Iterable[int], on: bool) -> None:
        active = set(active_pins)
        for pin in _ALL_LED_PINS:
            self._set_led(pin, pin in active and on)

    def _all_leds_off(self) -> None:
        for pin in _ALL_LED_PINS:
            self._set_led(pin, False)

    def _show_selection(self) -> None:
        for index, button in enumerate(self._buttons):
            self._set_led(button["led_pin"], index == self._selected_index)

    def _set_led(self, led_pin: int, on: bool) -> None:
        self._require_request().set_value(led_pin, Value.ACTIVE if on else Value.INACTIVE)

    def _require_request(self) -> gpiod.LineRequest:
        if self._request is None:
            raise RuntimeError("ButtonMenu must be used as a context manager")
        return self._request

    def _line_config(self) -> GpiodLineConfig:
        config: GpiodLineConfig = {}
        for button in self._buttons:
            config[button["button_pin"]] = gpiod.LineSettings(
                direction=Direction.INPUT,
                bias=Bias.PULL_UP,
                edge_detection=Edge.FALLING,
            )
        for pin in _ALL_LED_PINS:
            config[pin] = gpiod.LineSettings(
                direction=Direction.OUTPUT,
                output_value=Value.INACTIVE,
            )
        return config
