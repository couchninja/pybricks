"""Wall-clock-driven simulation time for the navigator and web scene."""

from __future__ import annotations

from threading import Lock
from time import perf_counter

from astropy import units as u
from astropy.time import Time

from simulate.astronomy.utils.ephemeris import current_time

TIME_SCALE_REALTIME = 1.0
TIME_SCALE_ONE_MINUTE_PER_SECOND = 60.0
TIME_SCALE_ONE_HOUR_PER_SECOND = 3600.0
TIME_SCALE_ONE_DAY_PER_SECOND = 86_400.0
TIME_SCALE_ONE_MONTH_PER_SECOND = 86_400.0 * 30.0

TIME_SCALE_PRESETS: dict[str, float] = {
    "realtime": TIME_SCALE_REALTIME,
    "minute": TIME_SCALE_ONE_MINUTE_PER_SECOND,
    "hour": TIME_SCALE_ONE_HOUR_PER_SECOND,
    "day": TIME_SCALE_ONE_DAY_PER_SECOND,
    "month": TIME_SCALE_ONE_MONTH_PER_SECOND,
}

TIME_SCALE_LABELS: dict[str, str] = {
    "realtime": "Realtime",
    "minute": "1 minute / second",
    "hour": "1 hour / second",
    "day": "1 day / second",
    "month": "1 month / second",
}

_lock = Lock()
_start_time: Time | None = None
_wall_start: float | None = None
_time_scaling = TIME_SCALE_REALTIME


def reset_simulation_clock() -> None:
    global _start_time, _wall_start, _time_scaling
    with _lock:
        _start_time = None
        _wall_start = None
        _time_scaling = TIME_SCALE_REALTIME


def time_scaling() -> float:
    return _time_scaling


def active_preset() -> str | None:
    scale = _time_scaling
    for name, preset_scale in TIME_SCALE_PRESETS.items():
        if scale == preset_scale:
            return name
    return None


def time_scale_label() -> str:
    preset = active_preset()
    if preset is not None:
        return TIME_SCALE_LABELS[preset]
    return f"{_time_scaling:g}×"


def simulation_time() -> Time:
    with _lock:
        _ensure_initialized_locked()
        assert _wall_start is not None
        assert _start_time is not None
        elapsed = perf_counter() - _wall_start
        return _start_time + elapsed * _time_scaling * u.second


def set_time_scaling(scale: float) -> None:
    if scale <= 0.0:
        raise ValueError("time scale must be positive")
    global _start_time, _wall_start, _time_scaling
    with _lock:
        _ensure_initialized_locked()
        assert _wall_start is not None
        assert _start_time is not None
        elapsed = perf_counter() - _wall_start
        now_sim = _start_time + elapsed * _time_scaling * u.second
        _start_time = now_sim
        _wall_start = perf_counter()
        _time_scaling = scale


def set_time_scale_preset(name: str) -> None:
    try:
        scale = TIME_SCALE_PRESETS[name]
    except KeyError as exc:
        raise ValueError(f"unknown time scale preset: {name}") from exc
    set_time_scaling(scale)


def sync_to_realtime() -> None:
    global _start_time, _wall_start, _time_scaling
    with _lock:
        _start_time = current_time()
        _wall_start = perf_counter()
        _time_scaling = TIME_SCALE_REALTIME


def reanchor_wall_clock() -> None:
    """Ignore wall time since the last anchor (e.g. after slow scene setup)."""
    global _wall_start
    with _lock:
        _ensure_initialized_locked()
        assert _wall_start is not None
        _wall_start = perf_counter()


def _ensure_initialized_locked() -> None:
    global _start_time, _wall_start
    if _start_time is None:
        _start_time = current_time()
        _wall_start = perf_counter()


def time_scale_status_payload() -> dict[str, object]:
    time = simulation_time()
    preset = active_preset()
    return {
        "time_iso": time.iso,
        "time_scaling": time_scaling(),
        "preset": preset,
        "label": time_scale_label(),
        "presets": list(TIME_SCALE_PRESETS.keys()),
    }
