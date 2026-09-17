"""Network-backed astronomy data (ISS TLE, IERS) with a shared refresh policy."""

from __future__ import annotations

from simulate.astronomy.utils.iers_refresh import refresh_iers_table_if_needed
from simulate.astronomy.utils.iss import refresh_iss_tle
from simulate.astronomy.utils.iss_tle import reset_network_retry, set_celestrak_fetch_allowed


def refresh_astronomy_downloads(*, allow_network: bool) -> None:
    """Refresh ISS TLE and IERS when ``allow_network`` and the clock is trusted."""
    set_celestrak_fetch_allowed(allow_network)
    if allow_network:
        reset_network_retry()
    refresh_iss_tle()
    refresh_iers_table_if_needed(allow_network=allow_network)
