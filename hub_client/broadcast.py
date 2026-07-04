"""BLE broadcast-only helpers (no scanning required)."""

from __future__ import annotations

from contextlib import asynccontextmanager
from typing import AsyncIterator, cast

from dbus_fast.aio import MessageBus
from dbus_fast.constants import BusType
from pb_ble.bluezdbus import BlueZBroadcaster, PybricksBroadcastAdvertisement, get_adapter
from pb_ble.constants import PybricksBroadcastData

from hub_client.constants import COMMAND_CHANNEL


class CommandBroadcaster:
    def __init__(self, broadcaster: BlueZBroadcaster, channel: int):
        self._broadcaster = broadcaster
        self._adv = PybricksBroadcastAdvertisement(broadcaster.name, channel)

    async def broadcast(self, message: str) -> None:
        if not self._broadcaster.is_broadcasting(self._adv):
            await self._broadcaster.broadcast(self._adv)
        self._adv.message = cast(PybricksBroadcastData, (message,))

    async def close(self) -> None:
        if self._broadcaster.is_broadcasting(self._adv):
            await self._broadcaster.stop_broadcast(self._adv)


@asynccontextmanager
async def open_broadcaster(
    channel: int = COMMAND_CHANNEL,
) -> AsyncIterator[CommandBroadcaster]:
    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    adapter = await get_adapter(bus)
    async with BlueZBroadcaster(bus, adapter, "remote") as broadcaster:
        yield CommandBroadcaster(broadcaster, channel)
