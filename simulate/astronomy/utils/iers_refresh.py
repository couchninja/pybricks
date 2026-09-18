"""IERS table loading and refresh via Astropy's download cache."""

from __future__ import annotations

import time
from pathlib import Path

from astropy.utils.data import download_file, is_url_in_cache
from astropy.utils.iers import IERS_Auto
from astropy.utils.iers import conf as iers_conf

IERS_REFRESH_INTERVAL_S = 3600.0


def configure_iers() -> None:
    """Do not download IERS data implicitly; callers refresh after clock sync."""
    iers_conf.auto_download = False
    iers_conf.auto_max_age = None


def ensure_iers_table_loaded() -> None:
    """Prefer Astropy's on-disk cache, then bundled tables. Never downloads."""
    if IERS_Auto.iers_table is not None:
        return
    if _load_iers_table_from_cache():
        return
    IERS_Auto.open()


def refresh_iers_table_if_needed(*, allow_network: bool) -> None:
    """Load IERS data and optionally download when cache is older than one hour."""
    ensure_iers_table_loaded()
    if not allow_network:
        return

    age_s = _iers_cache_age_s()
    if age_s is not None and 0.0 <= age_s < IERS_REFRESH_INTERVAL_S:
        print(f"IERS: using astropy cache (within {IERS_REFRESH_INTERVAL_S:.0f}s refresh interval)")
        _load_iers_table_from_cache()
        return

    urls = (iers_conf.iers_auto_url, iers_conf.iers_auto_url_mirror)
    print("IERS: downloading latest table from IERS…")
    filename = download_file(urls[0], cache="update", sources=list(urls))
    table = IERS_Auto.read(file=filename)
    IERS_Auto._substitute_iers_b(table)
    IERS_Auto.iers_table = table
    print(f"IERS: loaded from {filename}")


def _iers_cache_path() -> Path | None:
    url = iers_conf.iers_auto_url
    if not is_url_in_cache(url):
        return None
    return Path(download_file(url, cache=True))


def _iers_cache_age_s() -> float | None:
    path = _iers_cache_path()
    if path is None:
        return None
    return time.time() - path.stat().st_mtime


def _load_iers_table_from_cache() -> bool:
    path = _iers_cache_path()
    if path is None:
        return False
    table = IERS_Auto.read(file=str(path))
    IERS_Auto._substitute_iers_b(table)
    IERS_Auto.iers_table = table
    return True
