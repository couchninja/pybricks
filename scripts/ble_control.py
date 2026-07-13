#!/usr/bin/env python3
"""Control a Pybricks hub over BLE from this machine."""

from __future__ import annotations

import argparse
import asyncio
import sys

from pybricksdev.connections.pybricks import PybricksHubBLE

from hub_client.ble import (
    RECOVERABLE_ERRORS,
    connect,
    disconnect_hub,
    format_error,
)


async def cmd_scan(_: argparse.Namespace) -> None:
    from bleak import BleakScanner

    from pybricksdev.ble.pybricks import PYBRICKS_SERVICE_UUID

    print("Scanning for Pybricks hubs (15s)...")
    devices = await BleakScanner.discover(timeout=15.0, return_adv=True)
    found = False
    for address, (device, adv) in devices.items():
        if PYBRICKS_SERVICE_UUID.lower() not in [
            uuid.lower() for uuid in (adv.service_uuids or [])
        ]:
            continue
        found = True
        name = device.name or adv.local_name or "(unnamed)"
        print(f"  {name}  {address}")
    if not found:
        print("No Pybricks hubs found.")
        print(
            "Make sure the hub is on, running Pybricks firmware, and not connected elsewhere."
        )


async def cmd_stop(args: argparse.Namespace) -> None:
    hub = await connect(args.name, retries=args.retries)
    try:
        await hub.stop_user_program()
        print("Stopped user program.")
    finally:
        await disconnect_hub(hub)


async def cmd_run(args: argparse.Namespace) -> None:
    last_error: Exception | None = None
    for attempt in range(1, args.retries + 1):
        hub: PybricksHubBLE | None = None
        try:
            hub = await connect(args.name, retries=args.retries)
            if args.stop_first:
                try:
                    await hub.stop_user_program()
                    await asyncio.sleep(0.5)
                except RECOVERABLE_ERRORS:
                    pass
            await hub.run(args.program, wait=args.wait)
            return
        except RECOVERABLE_ERRORS as exc:
            last_error = exc
            if attempt < args.retries:
                print(
                    f"Failed ({format_error(exc)}); retrying ({attempt}/{args.retries})..."
                )
                await asyncio.sleep(2.0)
        finally:
            if hub is not None:
                await disconnect_hub(hub)

    if last_error is not None:
        raise last_error
    raise RuntimeError("run failed")


async def cmd_download(args: argparse.Namespace) -> None:
    last_error: Exception | None = None
    for attempt in range(1, args.retries + 1):
        hub: PybricksHubBLE | None = None
        try:
            hub = await connect(args.name, retries=args.retries)
            if args.stop_first:
                try:
                    await hub.stop_user_program()
                    await asyncio.sleep(0.5)
                except RECOVERABLE_ERRORS:
                    pass
            await hub.download(args.program)
            print("Program downloaded.")
            if args.start:
                await hub.start_user_program()
                print("Program started.")
            return
        except RECOVERABLE_ERRORS as exc:
            last_error = exc
            if attempt < args.retries:
                print(
                    f"Failed ({format_error(exc)}); retrying ({attempt}/{args.retries})..."
                )
                await asyncio.sleep(2.0)
        finally:
            if hub is not None:
                await disconnect_hub(hub)

    if last_error is not None:
        raise last_error
    raise RuntimeError("download failed")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-n",
        "--name",
        help="Hub Bluetooth name or address (optional)",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=5,
        help="number of attempts before giving up (default: 5)",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("scan", help="scan for nearby Pybricks hubs")

    subparsers.add_parser("stop", help="stop the running program on the hub")

    run_parser = subparsers.add_parser("run", help="compile, upload, and run a program")
    run_parser.add_argument("program", help="path to the .py program")
    run_parser.add_argument(
        "--no-stop-first",
        dest="stop_first",
        action="store_false",
        help="do not stop a running program before uploading",
    )
    run_parser.add_argument(
        "--no-wait",
        dest="wait",
        action="store_false",
        help="return after starting the program instead of waiting for it to finish",
    )
    run_parser.set_defaults(stop_first=True, wait=True)

    download_parser = subparsers.add_parser(
        "download", help="compile and upload a program without running it"
    )
    download_parser.add_argument("program", help="path to the .py program")
    download_parser.add_argument(
        "--start",
        action="store_true",
        help="start the program after downloading",
    )
    download_parser.add_argument(
        "--no-stop-first",
        dest="stop_first",
        action="store_false",
        help="do not stop a running program before uploading",
    )
    download_parser.set_defaults(stop_first=True)

    args = parser.parse_args()
    commands = {
        "scan": cmd_scan,
        "stop": cmd_stop,
        "run": cmd_run,
        "download": cmd_download,
    }
    try:
        asyncio.run(commands[args.command](args))
    except asyncio.TimeoutError:
        print("Hub not found.", file=sys.stderr)
        print(
            "Tip: turn the hub off and on, move it closer, and close the Pybricks phone app.",
            file=sys.stderr,
        )
        sys.exit(1)
    except RECOVERABLE_ERRORS as exc:
        print(f"Failed: {format_error(exc)}", file=sys.stderr)
        print(
            "Tip: if the hub is running a program, power-cycle it, then run `pixi run stop`.",
            file=sys.stderr,
        )
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
