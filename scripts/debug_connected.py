#!/usr/bin/env python3
"""Broadcast commands while GATT stays connected and watch stdout."""

import asyncio

from hub_client.ble import connect, disconnect_hub
from hub_client.broadcast import open_broadcaster


async def main() -> None:
    hub = await connect()
    try:
        await hub.stop_user_program()
        await asyncio.sleep(0.5)
        await hub.download("pybricks/main.py")
        await hub.run(wait=False, print_output=False, line_handler=True)
        await asyncio.sleep(0.5)

        while True:
            try:
                line = await asyncio.wait_for(hub.read_line(), timeout=0.5)
                print("stdout:", repr(line))
                if "ready" in line:
                    break
            except TimeoutError:
                pass

        async with open_broadcaster() as broadcaster:
            for i in range(30):
                await broadcaster.broadcast("1 ping")
                try:
                    line = await asyncio.wait_for(hub.read_line(), timeout=0.3)
                    print("response:", repr(line))
                except TimeoutError:
                    pass
                await asyncio.sleep(0.2)
    finally:
        try:
            await hub.stop_user_program()
        except Exception:
            pass
        await disconnect_hub(hub)


if __name__ == "__main__":
    asyncio.run(main())
