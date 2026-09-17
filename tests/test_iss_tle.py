"""ISS TLE resolution: CelesTrak fetch, cache, and offline fallback."""

import os
import tempfile
import time
import urllib.error
from email.message import Message
from io import BytesIO
from pathlib import Path
from unittest.mock import patch

from simulate.astronomy.utils import iss_tle

_VALID_TLE = """ISS (ZARYA)
1 25544U 98067A   26259.14303184  .00007008  00000+0  13461-3 0  9990
2 25544  51.6310 209.9325 0004907 145.2560 214.8750 15.49133683585852
"""


def _bundled_tle_path() -> Path:
    return Path(__file__).resolve().parent.parent / "simulate" / "astronomy" / "data" / "iss.tle"


def _reset_iss_tle_state() -> None:
    iss_tle._network_retry_after_monotonic = 0.0
    iss_tle._active_tle_lines = None


def test_fetch_disabled_uses_cache_without_network() -> None:
    _reset_iss_tle_state()
    with tempfile.TemporaryDirectory() as tmp:
        cache_path = Path(tmp) / "iss.tle"
        cache_path.write_text(_VALID_TLE)
        with (
            patch.object(iss_tle, "_CACHE_TLE_PATH", cache_path),
            patch.object(iss_tle, "ISS_TLE_FETCH_FROM_CELESTRAK", False),
            patch.object(
                iss_tle,
                "_fetch_iss_tle_text",
                side_effect=RuntimeError("fetch must not run"),
            ),
        ):
            _name, line1, _line2 = iss_tle.resolve_iss_tle_lines()
        assert "26259.14303184" in line1


def test_offline_uses_bundled_when_cache_missing() -> None:
    _reset_iss_tle_state()
    with tempfile.TemporaryDirectory() as tmp:
        cache_path = Path(tmp) / "iss.tle"
        with (
            patch.object(iss_tle, "_CACHE_TLE_PATH", cache_path),
            patch.object(iss_tle, "_BUNDLED_TLE_PATH", _bundled_tle_path()),
            patch.object(iss_tle, "ISS_TLE_FETCH_FROM_CELESTRAK", True),
            patch.object(
                iss_tle,
                "_fetch_iss_tle_text",
                side_effect=iss_tle.IssTleNetworkUnavailable("offline"),
            ),
        ):
            name, line1, line2 = iss_tle.resolve_iss_tle_lines()
        assert name.startswith("ISS")
        assert line1.startswith("1 ")
        assert line2.startswith("2 ")
        assert not cache_path.exists()


def test_offline_uses_cache_when_present() -> None:
    _reset_iss_tle_state()
    with tempfile.TemporaryDirectory() as tmp:
        cache_path = Path(tmp) / "iss.tle"
        cache_path.write_text(_VALID_TLE)
        with (
            patch.object(iss_tle, "_CACHE_TLE_PATH", cache_path),
            patch.object(iss_tle, "_BUNDLED_TLE_PATH", _bundled_tle_path()),
            patch.object(iss_tle, "ISS_TLE_FETCH_FROM_CELESTRAK", True),
            patch.object(
                iss_tle,
                "_fetch_iss_tle_text",
                side_effect=iss_tle.IssTleNetworkUnavailable("offline"),
            ),
        ):
            _name, line1, _line2 = iss_tle.resolve_iss_tle_lines()
        assert "26259.14303184" in line1


def test_celestrak_rejection_uses_fallback() -> None:
    _reset_iss_tle_state()
    with tempfile.TemporaryDirectory() as tmp:
        cache_path = Path(tmp) / "iss.tle"
        with (
            patch.object(iss_tle, "_CACHE_TLE_PATH", cache_path),
            patch.object(iss_tle, "_BUNDLED_TLE_PATH", _bundled_tle_path()),
            patch.object(iss_tle, "ISS_TLE_FETCH_FROM_CELESTRAK", True),
            patch("urllib.request.urlopen") as urlopen,
        ):
            urlopen.side_effect = urllib.error.HTTPError(
                iss_tle.ISS_TLE_CELESTRAK_URL,
                403,
                "Forbidden",
                hdrs=Message(),
                fp=BytesIO(b""),
            )
            name, line1, line2 = iss_tle.resolve_iss_tle_lines()
        assert name.startswith("ISS")
        assert line1.startswith("1 ")
        assert line2.startswith("2 ")


def test_http_error_raises() -> None:
    _reset_iss_tle_state()
    with tempfile.TemporaryDirectory() as tmp:
        cache_path = Path(tmp) / "missing.tle"
        with (
            patch.object(iss_tle, "_CACHE_TLE_PATH", cache_path),
            patch.object(iss_tle, "ISS_TLE_FETCH_FROM_CELESTRAK", True),
        ):
            try:
                with patch("urllib.request.urlopen") as urlopen:
                    urlopen.side_effect = urllib.error.HTTPError(
                        iss_tle.ISS_TLE_CELESTRAK_URL,
                        503,
                        "Unavailable",
                        hdrs=Message(),
                        fp=BytesIO(b""),
                    )
                    iss_tle.resolve_iss_tle_lines()
                raised = False
            except RuntimeError as exc:
                raised = True
                assert "HTTP 503" in str(exc)
        assert raised


def test_invalid_response_raises() -> None:
    _reset_iss_tle_state()
    with tempfile.TemporaryDirectory() as tmp:
        cache_path = Path(tmp) / "missing.tle"
        with (
            patch.object(iss_tle, "_CACHE_TLE_PATH", cache_path),
            patch.object(iss_tle, "ISS_TLE_FETCH_FROM_CELESTRAK", True),
            patch.object(iss_tle, "_fetch_iss_tle_text", return_value="not a tle"),
        ):
            try:
                iss_tle.resolve_iss_tle_lines()
                raised = False
            except RuntimeError as exc:
                raised = True
                assert "valid TLE" in str(exc) or "expected 3 lines" in str(exc)
        assert raised


def test_success_writes_cache() -> None:
    _reset_iss_tle_state()
    with tempfile.TemporaryDirectory() as tmp:
        cache_path = Path(tmp) / "iss.tle"
        with (
            patch.object(iss_tle, "_CACHE_TLE_PATH", cache_path),
            patch.object(iss_tle, "ISS_TLE_FETCH_FROM_CELESTRAK", True),
            patch.object(iss_tle, "_fetch_iss_tle_text", return_value=_VALID_TLE),
        ):
            iss_tle.resolve_iss_tle_lines()
        assert cache_path.is_file()
        assert "26259.14303184" in cache_path.read_text()


def test_skips_fetch_when_cache_is_fresh() -> None:
    _reset_iss_tle_state()
    with tempfile.TemporaryDirectory() as tmp:
        cache_path = Path(tmp) / "iss.tle"
        calls = {"count": 0}

        def fetch_ok(*_args, **_kwargs):
            calls["count"] += 1
            return _VALID_TLE

        with (
            patch.object(iss_tle, "_CACHE_TLE_PATH", cache_path),
            patch.object(iss_tle, "ISS_TLE_FETCH_FROM_CELESTRAK", True),
            patch.object(iss_tle, "_fetch_iss_tle_text", side_effect=fetch_ok),
        ):
            iss_tle.resolve_iss_tle_lines()
            iss_tle.resolve_iss_tle_lines()
        assert calls["count"] == 1


def test_refetches_when_cache_is_stale() -> None:
    _reset_iss_tle_state()
    with tempfile.TemporaryDirectory() as tmp:
        cache_path = Path(tmp) / "iss.tle"
        cache_path.write_text(_VALID_TLE)
        stale_mtime = time.time() - iss_tle.ISS_TLE_REFRESH_INTERVAL_S - 1.0
        os.utime(cache_path, (stale_mtime, stale_mtime))
        calls = {"count": 0}

        def fetch_ok(*_args, **_kwargs):
            calls["count"] += 1
            return _VALID_TLE

        with (
            patch.object(iss_tle, "_CACHE_TLE_PATH", cache_path),
            patch.object(iss_tle, "ISS_TLE_FETCH_FROM_CELESTRAK", True),
            patch.object(iss_tle, "_fetch_iss_tle_text", side_effect=fetch_ok),
        ):
            iss_tle.resolve_iss_tle_lines()
        assert calls["count"] == 1


def test_url_error_maps_to_network_unavailable() -> None:
    _reset_iss_tle_state()
    with patch("urllib.request.urlopen") as urlopen:
        urlopen.side_effect = urllib.error.URLError("no route to host")
        try:
            iss_tle._fetch_iss_tle_text()
            raised = False
        except iss_tle.IssTleNetworkUnavailable:
            raised = True
    assert raised
