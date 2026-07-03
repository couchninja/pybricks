#!/usr/bin/env python3
"""Drive the robot with arrow keys using pybricks-ble broadcast."""

from __future__ import annotations

import argparse
import asyncio
import curses
import sys
from typing import cast

from dbus_fast.aio import MessageBus
from dbus_fast.constants import BusType
from pb_ble.bluezdbus import (
    BlueZBroadcaster,
    PybricksBroadcastAdvertisement,
    get_adapter,
)
from pb_ble.constants import PybricksBroadcastData

REMOTE_CHANNEL = 7
DRIVE_SPEED = 50
TURN_SPEED = 50


def speeds_for_key(key: int) -> tuple[int, int]:
    if key == curses.KEY_UP:
        return DRIVE_SPEED, DRIVE_SPEED
    if key == curses.KEY_DOWN:
        return -DRIVE_SPEED, -DRIVE_SPEED
    if key == curses.KEY_LEFT:
        return TURN_SPEED, -TURN_SPEED
    if key == curses.KEY_RIGHT:
        return -TURN_SPEED, TURN_SPEED
    return 0, 0


class BroadcastRadio:
    """Broadcast-only BLE radio (no observer / passive scan required)."""

    def __init__(self, broadcaster: BlueZBroadcaster, channel: int):
        self._broadcaster = broadcaster
        self._adv = PybricksBroadcastAdvertisement(broadcaster.name, channel)

    async def broadcast(self, left: int, right: int) -> None:
        if not self._broadcaster.is_broadcasting(self._adv):
            await self._broadcaster.broadcast(self._adv)
        self._adv.message = cast(PybricksBroadcastData, (left, right))

    async def stop(self) -> None:
        await self.broadcast(0, 0)
        if self._broadcaster.is_broadcasting(self._adv):
            await self._broadcaster.stop_broadcast(self._adv)


async def drive(stdscr, channel: int) -> None:
    curses.cbreak()
    curses.noecho()
    stdscr.keypad(True)
    stdscr.nodelay(True)

    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    adapter = await get_adapter(bus)
    async with BlueZBroadcaster(bus, adapter, "remote") as broadcaster:
        radio = BroadcastRadio(broadcaster, channel)
        try:
            left = 0
            right = 0
            while True:
                key = stdscr.getch()
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key != -1:
                    left, right = speeds_for_key(key)
                else:
                    left, right = 0, 0

                await radio.broadcast(left, right)

                stdscr.erase()
                stdscr.addstr(0, 0, "Arrow keys: drive. Q or Esc: quit.")
                stdscr.addstr(2, 0, f"Broadcasting on channel {channel}")
                stdscr.addstr(3, 0, f"Left: {left:4d}   Right: {right:4d}")
                stdscr.refresh()
                await asyncio.sleep(0.1)
        finally:
            await radio.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--channel",
        type=int,
        default=REMOTE_CHANNEL,
        help=f"BLE broadcast channel (default: {REMOTE_CHANNEL})",
    )
    args = parser.parse_args()

    if not sys.stdin.isatty():
        print("Remote control needs an interactive terminal.", file=sys.stderr)
        sys.exit(1)

    try:
        curses.wrapper(lambda stdscr: asyncio.run(drive(stdscr, args.channel)))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
