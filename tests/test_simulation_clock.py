import json

import pytest

from simulate.astronomy.constants import PointingTarget
from simulate.astronomy.simulation_clock import (
    TIME_SCALE_ONE_DAY_PER_SECOND,
    TIME_SCALE_ONE_HOUR_PER_SECOND,
    TIME_SCALE_ONE_MINUTE_PER_SECOND,
    reset_simulation_clock,
    set_time_scale_preset,
    set_time_scaling,
    sync_to_realtime,
    time_scale_status_payload,
    time_scaling,
)
from simulate.astronomy.web_scene import reset_web_scene_cache, scene_snapshot_payload


def test_time_scale_status_json_serializable() -> None:
    reset_simulation_clock()
    json.dumps(time_scale_status_payload())


def test_set_preset_changes_scaling() -> None:
    reset_simulation_clock()
    set_time_scale_preset("day")
    assert time_scaling() == TIME_SCALE_ONE_DAY_PER_SECOND


def test_minute_preset_scaling() -> None:
    reset_simulation_clock()
    set_time_scale_preset("minute")
    assert time_scaling() == TIME_SCALE_ONE_MINUTE_PER_SECOND


def test_sync_to_realtime_resets_scale() -> None:
    reset_simulation_clock()
    set_time_scaling(3600.0)
    sync_to_realtime()
    assert time_scaling() == 1.0


def test_scene_snapshot_honors_time_scale_preset() -> None:
    reset_web_scene_cache()
    set_time_scale_preset("hour")
    json.dumps(scene_snapshot_payload(PointingTarget.EARTH_ROTATION))
    status = time_scale_status_payload()
    assert status["time_iso"]
    assert time_scaling() == TIME_SCALE_ONE_HOUR_PER_SECOND


def test_unknown_preset_raises() -> None:
    reset_simulation_clock()
    with pytest.raises(ValueError):
        set_time_scale_preset("invalid")
