from unittest.mock import AsyncMock, patch

import pytest

from navigator import navigator_main as nav


def test_gpio_unavailable_on_darwin() -> None:
    with patch.object(nav.sys, "platform", "darwin"):
        assert not nav.gpio_available()
    with patch.object(nav.sys, "platform", "linux"):
        assert nav.gpio_available()


@pytest.mark.asyncio
async def test_navigator_main_runs_web_ui_only_on_darwin() -> None:
    with (
        patch.object(nav.sys, "platform", "darwin"),
        patch.object(nav, "run_web_ui_only", new=AsyncMock()) as web_only,
    ):
        await nav.navigator_main()
    web_only.assert_awaited_once()
