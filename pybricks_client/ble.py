"""BLE connection helpers for Pybricks hubs."""

from __future__ import annotations

import asyncio
import shutil

from bleak.exc import BleakError, BleakGATTProtocolError
from pybricksdev.ble import find_device
from pybricksdev.connections.pybricks import PybricksHubBLE

RECOVERABLE_ERRORS = (
    asyncio.TimeoutError,
    BleakError,
    BleakGATTProtocolError,
    OSError,
    RuntimeError,
)


def format_error(exc: BaseException) -> str:
    message = str(exc).strip()
    if message:
        return f"{type(exc).__name__}: {message}"
    return type(exc).__name__


async def reset_bluez_device(address: str | None) -> None:
    """Drop any stale BlueZ connection before retrying."""
    if not address or not shutil.which("bluetoothctl"):
        return

    for command in (["disconnect", address], ["remove", address]):
        proc = await asyncio.create_subprocess_exec(
            "bluetoothctl",
            *command,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )
        await proc.wait()
    await asyncio.sleep(1.5)


async def connect(name: str | None = None, *, retries: int = 5) -> PybricksHubBLE:
    last_error: Exception | None = None
    address: str | None = None

    for attempt in range(1, retries + 1):
        try:
            print(f"Searching for {name or 'any hub with Pybricks service'}...")
            device = await find_device(name, timeout=15)
            address = device.address
            print(f"Found {device.name or '(unnamed)'} [{address}]")

            await asyncio.sleep(1.0)

            hub = PybricksHubBLE(device)
            await hub.connect()
            print(f"Connected (firmware {hub.fw_version})")
            return hub
        except RECOVERABLE_ERRORS as exc:
            last_error = exc
            if attempt < retries:
                print(
                    f"Connection failed ({format_error(exc)}); "
                    f"retrying ({attempt}/{retries})..."
                )
                await reset_bluez_device(address)
                await asyncio.sleep(2.0)

    if last_error is not None:
        raise last_error
    raise RuntimeError("connection failed")


async def disconnect_hub(hub: PybricksHubBLE) -> None:
    try:
        await hub.disconnect()
    except RECOVERABLE_ERRORS:
        pass
    await reset_bluez_device(hub._device.address)
