from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, TypedDict

from pybricks.parameters import Color

from simulate.astronomy.constants import PointingTarget

# Status LEDs on the panel (white / blue / orange indicators).
CLOCK_UNSYNC_LED_PIN = 17
HUB_SEARCH_LED_PIN = 23
HUB_CALIBRATE_LED_PIN = 25

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


@dataclass
class PanelStatus:
    clock_synchronized: bool = False
    hub: HubLinkPhase = "disconnected"
