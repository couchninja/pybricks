"""Kernel clock-synchronization state.

The Raspberry Pi 4 has no battery-backed RTC, so it boots with an arbitrary
clock until a time source is reached. Every pointing angle is derived from the
current time, so the navigator must know when the clock cannot be trusted.

On hosts without Linux ``adjtimex`` (e.g. macOS dev), the clock is assumed synchronized.
"""

from __future__ import annotations

import ctypes
from typing import Any

_STA_UNSYNC = 0x0040
_TIME_ERROR = 5

# ``False`` when adjtimex is unavailable; otherwise the ctypes function pointer.
_adjtimex: Any = None


def clock_is_synchronized() -> bool:
    """True when the kernel reports the system clock is steered by a time source."""
    adjtimex = _resolve_adjtimex()
    if adjtimex is False:
        return True
    timex = _Timex()
    state = adjtimex(ctypes.byref(timex))
    if state < 0:
        raise OSError(ctypes.get_errno(), "adjtimex failed")
    return state != _TIME_ERROR and not timex.status & _STA_UNSYNC


def _resolve_adjtimex() -> Any:
    global _adjtimex
    if _adjtimex is not None:
        return _adjtimex
    try:
        fn = ctypes.CDLL(None, use_errno=True).adjtimex
    except AttributeError:
        _adjtimex = False
        return _adjtimex
    fn.argtypes = (ctypes.POINTER(_Timex),)
    fn.restype = ctypes.c_int
    _adjtimex = fn
    return _adjtimex


class _Timeval(ctypes.Structure):
    _fields_ = (
        ("tv_sec", ctypes.c_long),
        ("tv_usec", ctypes.c_long),
    )


class _Timex(ctypes.Structure):
    _fields_ = (
        ("modes", ctypes.c_int),
        ("offset", ctypes.c_long),
        ("freq", ctypes.c_long),
        ("maxerror", ctypes.c_long),
        ("esterror", ctypes.c_long),
        ("status", ctypes.c_int),
        ("constant", ctypes.c_long),
        ("precision", ctypes.c_long),
        ("tolerance", ctypes.c_long),
        ("time", _Timeval),
        ("tick", ctypes.c_long),
        ("ppsfreq", ctypes.c_long),
        ("jitter", ctypes.c_long),
        ("shift", ctypes.c_int),
        ("stabil", ctypes.c_long),
        ("jitcnt", ctypes.c_long),
        ("calcnt", ctypes.c_long),
        ("errcnt", ctypes.c_long),
        ("stbcnt", ctypes.c_long),
        ("tai", ctypes.c_int),
        ("_padding", ctypes.c_int * 11),
    )
