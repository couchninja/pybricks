#!/usr/bin/env python3
"""Check whether this machine can see its own Pybricks BLE broadcasts."""

import asyncio

from bleak import BleakScanner

from hub_client.broadcast import open_broadcaster

LEGO_CID = 0x0397


async def main() -> None:
    seen: list[str] = []

    def callback(_device, adv_data) -> None:
        if LEGO_CID in adv_data.manufacturer_data:
            seen.append(repr(adv_data.manufacturer_data[LEGO_CID]))

    scanner = BleakScanner(callback, scanning_mode="active")
    await scanner.start()
    await asyncio.sleep(1.0)

    async with open_broadcaster() as broadcaster:
        for i in range(10):
            await broadcaster.broadcast(f"{i} ping")
            await asyncio.sleep(0.3)

    await asyncio.sleep(2.0)
    await scanner.stop()
    print(f"Seen {len(seen)} LEGO advertisements while broadcasting")
    for item in seen[:10]:
        print(" ", item)


if __name__ == "__main__":
    asyncio.run(main())
