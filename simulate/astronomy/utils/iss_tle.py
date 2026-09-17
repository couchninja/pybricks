from __future__ import annotations

import time
import urllib.error
import urllib.request
from pathlib import Path

ISS_TLE_FETCH_FROM_CELESTRAK = True
ISS_TLE_CELESTRAK_URL = "https://celestrak.org/NORAD/elements/gp.php?CATNR=25544&FORMAT=TLE"
ISS_TLE_FETCH_TIMEOUT_S = 15.0
ISS_TLE_USER_AGENT = "pybricks-iss-tle"
ISS_TLE_NETWORK_RETRY_INTERVAL_S = 60.0
ISS_TLE_REFRESH_INTERVAL_S = 3600.0

_BUNDLED_TLE_PATH = Path(__file__).resolve().parent.parent / "data" / "iss.tle"
_CACHE_TLE_PATH = Path.home() / ".cache" / "pybricks" / "iss.tle"

_network_retry_after_monotonic = 0.0
_active_tle_lines: tuple[str, str, str] | None = None


class IssTleNetworkUnavailable(Exception):
    """CelesTrak could not be reached (offline, DNS failure, timeout, etc.)."""

    def __init__(self, message: str, *, retry_after_s: float | None = None) -> None:
        super().__init__(message)
        self.retry_after_s = (
            retry_after_s if retry_after_s is not None else ISS_TLE_NETWORK_RETRY_INTERVAL_S
        )


def resolve_iss_tle_lines() -> tuple[str, str, str]:
    global _network_retry_after_monotonic, _active_tle_lines

    if not ISS_TLE_FETCH_FROM_CELESTRAK:
        return _load_fallback_tle_lines("CelesTrak fetch disabled")

    now = time.monotonic()
    if now < _network_retry_after_monotonic:
        return _load_fallback_tle_lines("CelesTrak unreachable; retry later")

    if _cached_tle_is_fresh():
        return _load_fallback_tle_lines("cached TLE within refresh interval")

    print("ISS TLE download starting from CelesTrak…")  # noqa: T201
    try:
        text = _fetch_iss_tle_text()
    except IssTleNetworkUnavailable as exc:
        _network_retry_after_monotonic = now + exc.retry_after_s
        return _load_fallback_tle_lines(str(exc))

    _network_retry_after_monotonic = 0.0
    name, line1, line2 = _parse_iss_tle_text(text)
    _write_cache_tle(name, line1, line2)
    _active_tle_lines = (name, line1, line2)
    print(f"ISS TLE downloaded from CelesTrak: {name.strip()}")  # noqa: T201
    return name, line1, line2


def reset_network_retry() -> None:
    """Drop the retry backoff so the next resolve tries CelesTrak again immediately."""
    global _network_retry_after_monotonic
    _network_retry_after_monotonic = 0.0


def _fetch_iss_tle_text() -> str:
    request = urllib.request.Request(
        ISS_TLE_CELESTRAK_URL,
        headers={
            "User-Agent": ISS_TLE_USER_AGENT,
            "Cache-Control": "no-cache",
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=ISS_TLE_FETCH_TIMEOUT_S) as response:
            charset = response.headers.get_content_charset() or "utf-8"
            return response.read().decode(charset)
    except urllib.error.HTTPError as exc:
        if exc.code in (403, 404):
            raise IssTleNetworkUnavailable(
                f"CelesTrak HTTP {exc.code}; using cached or bundled TLE",
                retry_after_s=ISS_TLE_REFRESH_INTERVAL_S,
            ) from exc
        raise RuntimeError(f"ISS TLE fetch failed: CelesTrak returned HTTP {exc.code}") from exc
    except urllib.error.URLError as exc:
        raise IssTleNetworkUnavailable(str(exc.reason)) from exc
    except TimeoutError as exc:
        raise IssTleNetworkUnavailable("request timed out") from exc


def _parse_iss_tle_text(text: str) -> tuple[str, str, str]:
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if len(lines) < 3:
        raise RuntimeError(f"ISS TLE fetch failed: expected 3 lines from CelesTrak, got {len(lines)}")
    name, line1, line2 = lines[0], lines[1], lines[2]
    if not line1.startswith("1 ") or not line2.startswith("2 "):
        raise RuntimeError("ISS TLE fetch failed: CelesTrak response is not a valid TLE")
    return name, line1, line2


def _load_fallback_tle_lines(reason: str) -> tuple[str, str, str]:
    global _active_tle_lines
    if _active_tle_lines is not None:
        print(f"ISS TLE: using TLE already loaded ({reason})")  # noqa: T201
        return _active_tle_lines
    if _CACHE_TLE_PATH.is_file():
        _active_tle_lines = _read_tle_file(_CACHE_TLE_PATH)
        print(f"ISS TLE: using cached file {_CACHE_TLE_PATH} ({reason})")  # noqa: T201
        return _active_tle_lines
    _active_tle_lines = _read_tle_file(_BUNDLED_TLE_PATH)
    print(f"ISS TLE: using bundled file {_BUNDLED_TLE_PATH} ({reason})")  # noqa: T201
    return _active_tle_lines


def _cached_tle_is_fresh() -> bool:
    if not _CACHE_TLE_PATH.is_file():
        return False
    age_s = time.time() - _CACHE_TLE_PATH.stat().st_mtime
    # A negative age means the clock is behind the cache file (the Pi boots without
    # an RTC), which says nothing about freshness: refetch instead of trusting it.
    return 0.0 <= age_s < ISS_TLE_REFRESH_INTERVAL_S


def _read_tle_file(path: Path) -> tuple[str, str, str]:
    return _parse_iss_tle_text(path.read_text())


def _write_cache_tle(name: str, line1: str, line2: str) -> None:
    _CACHE_TLE_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = f"{name}\n{line1}\n{line2}\n"
    temp_path = _CACHE_TLE_PATH.with_suffix(".tle.tmp")
    temp_path.write_text(payload)
    temp_path.replace(_CACHE_TLE_PATH)
