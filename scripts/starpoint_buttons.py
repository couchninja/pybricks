#!/usr/bin/env python3
"""Illuminate one button LED at a time (last press wins)."""

from __future__ import annotations

import time
from typing import TypedDict

import gpiod
from gpiod.line import Bias, Direction, Edge, Value
from pybricks.parameters import Color

from simulate.astronomy.constants import ObserverMotionMode

# BCM numbering. Button to GND (internal pull-up); LED on the paired output.
CHIP = "/dev/gpiochip0"
DEBOUNCE_S = 0.03


class ButtonData(TypedDict):
    button_pin: int
    led_pin: int
    mode: ObserverMotionMode
    color: Color


BUTTONS: tuple[ButtonData, ...] = (
    {
        "button_pin": 27,
        "led_pin": 17,
        "mode": ObserverMotionMode.EARTH_ROTATION,
        "color": Color.BLUE,
    },
    {
        "button_pin": 22,
        "led_pin": 23,
        "mode": ObserverMotionMode.SUN_ORBIT,
        "color": Color.ORANGE,
    },
    {
        "button_pin": 24,
        "led_pin": 25,
        "mode": ObserverMotionMode.MILKY_WAY_ORBIT,
        "color": Color.GREEN,
    },
    {
        "button_pin": 5,
        "led_pin": 6,
        "mode": ObserverMotionMode.CMB_DIPOLE,
        "color": Color.YELLOW,
    },
    {
        "button_pin": 12,
        "led_pin": 16,
        "mode": ObserverMotionMode.CMB_DIPOLE,
        "color": Color.WHITE,
    },
)


def main() -> None:
    button_to_led = {button["button_pin"]: button["led_pin"] for button in BUTTONS}
    config = {}
    for button in BUTTONS:
        config[button["button_pin"]] = gpiod.LineSettings(
            direction=Direction.INPUT,
            bias=Bias.PULL_UP,
            edge_detection=Edge.FALLING,
        )
        config[button["led_pin"]] = gpiod.LineSettings(
            direction=Direction.OUTPUT,
            output_value=Value.INACTIVE,
        )

    with gpiod.request_lines(
        CHIP,
        consumer="gpio-buttons",
        config=config,
    ) as request:
        for button in BUTTONS:
            print(f"  BCM {button['button_pin']} -> BCM {button['led_pin']}")

        last_event_s = {button["button_pin"]: 0.0 for button in BUTTONS}

        while True:
            for event in request.read_edge_events():
                button_pin = event.line_offset
                led_pin = button_to_led.get(button_pin)
                if led_pin is None:
                    continue
                now = time.monotonic()
                if now - last_event_s[button_pin] < DEBOUNCE_S:
                    continue
                last_event_s[button_pin] = now

                for button in BUTTONS:
                    request.set_value(
                        button["led_pin"],
                        Value.ACTIVE if button["led_pin"] == led_pin else Value.INACTIVE,
                    )
                print(f"GPIO {led_pin} HIGH")


if __name__ == "__main__":
    main()
