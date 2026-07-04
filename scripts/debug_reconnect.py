#!/usr/bin/env python3
"""Test: broadcast while disconnected, then reconnect GATT to read stdout."""

import asyncio

from hub_client.ble import connect, disconnect_hub
from hub_client.broadcast import open_broadcaster


async def main() -> None:
    async with open_broadcaster() as broadcaster:
        print("Broadcasting ping...")
        await broadcaster.broadcast("1 ping")

    print("Waiting for hub to process...")
    await asyncio.sleep(1.0)

    print("Reconnecting GATT to read stdout...")
    hub = await connect()
    try:
        hub._enable_line_handler = True
        hub._stdout_line_queue = asyncio.Queue()
        hub.print_output = False
        line = await asyncio.wait_for(hub.read_line(), timeout=5.0)
        print("Got line:", repr(line))
    finally:
        await disconnect_hub(hub)


if __name__ == "__main__":
    asyncio.run(main())
