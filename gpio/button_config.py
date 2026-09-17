from __future__ import annotations

import colorsys
from dataclasses import dataclass
from typing import Literal, TypedDict

from pybricks.parameters import Color

from simulate.astronomy.constants import PointingTarget

# Panel status indicators reuse the first three button LEDs.
CLOCK_UNSYNC_BUTTON_INDEX = 0
HUB_SEARCH_BUTTON_INDEX = 1
HUB_CALIBRATE_BUTTON_INDEX = 2

HubLinkPhase = Literal["disconnected", "calibrating", "ready"]


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
            {"target": PointingTarget.ISS, "color": Color.BLUE},
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


def pybricks_color_rgba(color: Color) -> list[int]:
    red, green, blue = colorsys.hsv_to_rgb(color.h / 360.0, color.s / 100.0, color.v / 100.0)
    return [int(round(red * 255)), int(round(green * 255)), int(round(blue * 255)), 255]


def _pointing_target_button_colors() -> dict[PointingTarget, Color]:
    by_target: dict[PointingTarget, Color] = {}
    for button in BUTTONS:
        for mode in button["modes"]:
            by_target[mode["target"]] = mode["color"]
    return by_target


POINTING_TARGET_BUTTON_COLORS = _pointing_target_button_colors()


def pointing_target_button_rgba(target: PointingTarget) -> list[int]:
    return pybricks_color_rgba(POINTING_TARGET_BUTTON_COLORS[target])


@dataclass
class PanelStatus:
    clock_synchronized: bool = False
    hub: HubLinkPhase = "disconnected"
