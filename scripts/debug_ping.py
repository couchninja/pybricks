#!/usr/bin/env python3
"""Send one broadcast command and listen for a hub response."""

import asyncio

from pb_ble.vhub import get_virtual_ble

from hub_client.constants import COMMAND_CHANNEL, RESPONSE_CHANNEL


async def main() -> None:
    radio = await get_virtual_ble(
        broadcast_channel=COMMAND_CHANNEL,
        observe_channels=[RESPONSE_CHANNEL],
        scanning_mode="active",
    )
    async with radio:
        print("Broadcasting ping on channel", COMMAND_CHANNEL)
        await radio.broadcast("1 ping")
        for i in range(100):
            response = radio.observe(RESPONSE_CHANNEL)
            if response is not None:
                print("Response:", response)
                return
            await asyncio.sleep(0.1)
        print("No response after 10 seconds")


if __name__ == "__main__":
    asyncio.run(main())
