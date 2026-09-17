"""IERS refresh via Astropy cache without hitting the network in tests."""

import pytest
from unittest.mock import MagicMock, patch

from astropy.utils.iers import IERS_Auto, conf as iers_conf

from simulate.astronomy.utils import iers_refresh


@pytest.fixture(autouse=True)
def _restore_iers_table_after_test() -> None:
    yield
    IERS_Auto.iers_table = None
    iers_refresh.ensure_iers_table_loaded()


def test_configure_iers_disables_auto_download() -> None:
    iers_refresh.configure_iers()
    assert iers_conf.auto_download is False
    assert iers_conf.auto_max_age is None


def test_refresh_skips_download_when_cache_is_fresh() -> None:
    IERS_Auto.iers_table = None
    with (
        patch.object(iers_refresh, "_iers_cache_age_s", return_value=100.0),
        patch.object(iers_refresh, "_load_iers_table_from_cache", return_value=True) as load_cache,
        patch("simulate.astronomy.utils.iers_refresh.download_file") as download,
    ):
        iers_refresh.refresh_iers_table_if_needed(allow_network=True)
    load_cache.assert_called()
    download.assert_not_called()


def test_refresh_downloads_when_cache_is_stale() -> None:
    IERS_Auto.iers_table = None
    stale_age = iers_refresh.IERS_REFRESH_INTERVAL_S + 1.0
    fake_table = MagicMock()

    with (
        patch.object(iers_refresh, "_iers_cache_age_s", return_value=stale_age),
        patch.object(iers_refresh, "ensure_iers_table_loaded"),
        patch(
            "simulate.astronomy.utils.iers_refresh.download_file",
            return_value="/tmp/finals2000A.all",
        ) as download,
        patch.object(IERS_Auto, "read", return_value=fake_table),
        patch.object(IERS_Auto, "_substitute_iers_b"),
    ):
        iers_refresh.refresh_iers_table_if_needed(allow_network=True)
    download.assert_called_once()
    assert IERS_Auto.iers_table is fake_table


def test_refresh_never_downloads_when_network_not_allowed() -> None:
    with (
        patch.object(iers_refresh, "ensure_iers_table_loaded") as ensure,
        patch("simulate.astronomy.utils.iers_refresh.download_file") as download,
    ):
        iers_refresh.refresh_iers_table_if_needed(allow_network=False)
    ensure.assert_called_once()
    download.assert_not_called()


def test_negative_cache_age_treated_as_stale() -> None:
    IERS_Auto.iers_table = None
    fake_table = MagicMock()
    with (
        patch.object(iers_refresh, "_iers_cache_age_s", return_value=-10.0),
        patch.object(iers_refresh, "ensure_iers_table_loaded"),
        patch(
            "simulate.astronomy.utils.iers_refresh.download_file",
            return_value="/tmp/finals2000A.all",
        ) as download,
        patch.object(IERS_Auto, "read", return_value=fake_table),
        patch.object(IERS_Auto, "_substitute_iers_b"),
    ):
        iers_refresh.refresh_iers_table_if_needed(allow_network=True)
    download.assert_called_once()
