#!/usr/bin/env python3
"""Broadcast a light.on command (no GATT connection required)."""

import asyncio

from hub_client.broadcast import open_broadcaster


async def main() -> None:
    async with open_broadcaster() as broadcaster:
        print("Broadcasting light.on BLUE")
        await broadcaster.broadcast("1 light.on BLUE")
        await asyncio.sleep(2)
        print("Broadcasting light.off")
        await broadcaster.broadcast("2 light.off")


if __name__ == "__main__":
    asyncio.run(main())
