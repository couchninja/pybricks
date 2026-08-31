#!/usr/bin/env python3
"""Toggle each illuminated button's LED on press."""

from __future__ import annotations

import time

import gpiod
from gpiod.line import Bias, Direction, Edge, Value

# BCM numbering. Button to GND (internal pull-up); LED on the paired output.
CHIP = "/dev/gpiochip0"
# (button, led) — physical pins: 13/11, 15/16, 18/22, 29/31, 32/36
CHANNELS = (
    (27, 17),
    (22, 23),
    (24, 25),
    (5, 6),
    (12, 16),
)
DEBOUNCE_S = 0.03


def main() -> None:
    button_to_led = dict(CHANNELS)
    config = {}
    for button, led in CHANNELS:
        config[button] = gpiod.LineSettings(
            direction=Direction.INPUT,
            bias=Bias.PULL_UP,
            edge_detection=Edge.FALLING,
        )
        config[led] = gpiod.LineSettings(
            direction=Direction.OUTPUT,
            output_value=Value.INACTIVE,
        )

    with gpiod.request_lines(
        CHIP,
        consumer="gpio-buttons",
        config=config,
    ) as request:
        print("Each button toggles its LED (Ctrl+C to stop)")
        for button, led in CHANNELS:
            print(f"  BCM {button} -> BCM {led}")

        high = {led: False for _, led in CHANNELS}
        last_event_s = {button: 0.0 for button, _ in CHANNELS}

        while True:
            for event in request.read_edge_events():
                button = event.line_offset
                led = button_to_led.get(button)
                if led is None:
                    continue
                now = time.monotonic()
                if now - last_event_s[button] < DEBOUNCE_S:
                    continue
                last_event_s[button] = now

                high[led] = not high[led]
                request.set_value(led, Value.ACTIVE if high[led] else Value.INACTIVE)
                print(f"GPIO {led} {'HIGH' if high[led] else 'LOW'}")


if __name__ == "__main__":
    main()
