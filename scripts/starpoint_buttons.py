#!/usr/bin/env python3
"""Illuminate one button LED at a time (last press wins)."""

from __future__ import annotations

import asyncio

from hub_client.starpoint.buttons import ButtonData, StarpointButtons


async def main() -> None:
    buttons = StarpointButtons()

    async def on_selection_changed(button: ButtonData) -> None:
        print(f"Selected: {button['mode'].label} (BCM {button['button_pin']})")

    buttons.on_selection_changed(on_selection_changed)
    await buttons.run()


if __name__ == "__main__":
    asyncio.run(main())
