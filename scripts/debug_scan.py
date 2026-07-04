#!/usr/bin/env python3
"""Listen for LEGO/Pybricks BLE advertisements (active scan)."""

import asyncio

from bleak import BleakScanner

LEGO_CID = 0x0397


async def main() -> None:
    seen: set[str] = set()

    def callback(device, adv_data) -> None:
        if LEGO_CID not in adv_data.manufacturer_data:
            return
        payload = adv_data.manufacturer_data[LEGO_CID]
        key = f"{device.address}:{payload!r}"
        if key in seen:
            return
        seen.add(key)
        print(f"{device.name or device.address}: {payload!r} rssi={adv_data.rssi}")

    scanner = BleakScanner(callback, scanning_mode="active")
    await scanner.start()
    print("Scanning for 20s (active mode)...")
    await asyncio.sleep(20)
    await scanner.stop()
    print(f"Seen {len(seen)} unique LEGO advertisements")


if __name__ == "__main__":
    asyncio.run(main())
