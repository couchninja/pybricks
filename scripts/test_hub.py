#!/usr/bin/env python3
"""Exercise motors, hub light, and sensors on a remote Move Hub."""

from __future__ import annotations

import argparse
import asyncio
import sys

from pybricks.parameters import Color, Direction, Port

from hub_client import MoveHub
from hub_client.client import _color_name


async def test_motors(hub: MoveHub) -> None:
    motors = [
        hub.motor(Port.A, Direction.CLOCKWISE),
        hub.motor(Port.B, Direction.COUNTERCLOCKWISE),
        hub.motor(Port.C, Direction.COUNTERCLOCKWISE),
    ]

    print("Motors: short forward pulse on A, B, and C")
    for motor in motors:
        angle_before = await motor.angle()
        await motor.dc(30)
        await asyncio.sleep(0.25)
        await motor.stop()
        await asyncio.sleep(0.1)
        angle_after = await motor.angle()
        delta = angle_after - angle_before
        print(
            f"  Port {motor.port.name}: angle {angle_before} -> {angle_after}"
            f" (delta={delta})"
        )
        if delta == 0:
            raise RuntimeError(
                f"Motor on port {motor.port.name} did not move (angle unchanged)"
            )


async def test_light(hub: MoveHub) -> None:
    colors = (Color.BLUE, Color.YELLOW, Color.GREEN, Color.RED)
    print("Hub light: blink through colors")
    for color in colors:
        await hub.light.on(color)
        print(f"  light.on({_color_name(color)})")
        await asyncio.sleep(0.35)
    await hub.light.off()
    print("  light.off()")


async def test_sensor(hub: MoveHub) -> None:
    sensor = hub.color_distance_sensor(Port.D)
    print("Sensor on port D:")
    distance = await sensor.distance()
    reflection = await sensor.reflection()
    ambient = await sensor.ambient()
    color = await sensor.color()
    print(f"  distance()   = {distance}")
    print(f"  reflection() = {reflection}")
    print(f"  ambient()    = {ambient}")
    print(f"  color()      = {_color_name(color)}")


async def run_tests(name: str | None) -> None:
    async with MoveHub.connect(name) as hub:
        await hub.ping()
        print("Hub responded to ping.\n")

        await test_light(hub)
        print()
        await test_motors(hub)
        print()
        await test_sensor(hub)

        print("\nAll tests completed.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-n",
        "--name",
        help="Hub Bluetooth name or address (optional)",
    )
    args = parser.parse_args()

    try:
        asyncio.run(run_tests(args.name))
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        sys.exit(130)


if __name__ == "__main__":
    main()
