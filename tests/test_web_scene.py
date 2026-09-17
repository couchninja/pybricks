import json

from simulate.astronomy.constants import PointingTarget
from simulate.astronomy.web_scene import reset_web_scene_cache, scene_snapshot_payload


def test_scene_snapshot_json_serializable() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.EARTH_ROTATION)
    json.dumps(payload)
    body_names = {body["name"] for body in payload["bodies"]}
    assert "earth" in body_names
    assert "sun" in body_names
    assert "observer" in body_names
    assert payload["time_iso"]
    assert payload["paths"]


def test_scene_snapshot_follows_pointing_target() -> None:
    reset_web_scene_cache()
    payload = scene_snapshot_payload(PointingTarget.MOON)
    assert payload["pointing_target_label"] == PointingTarget.MOON.label


def test_scene_snapshot_includes_arrow_mesh_length() -> None:
    reset_web_scene_cache()
    from simulate.astronomy.constants import EARTH_RADIUS_AU, OBSERVER_VELOCITY_ARROW_LENGTH_EARTH_RADII

    payload = scene_snapshot_payload(PointingTarget.EARTH_ROTATION)
    assert payload["arrow_mesh_length_au"] == OBSERVER_VELOCITY_ARROW_LENGTH_EARTH_RADII * EARTH_RADIUS_AU
    observer_arrow = next(a for a in payload["arrows"] if a["name"] == "observer_velocity_arrow")
    assert observer_arrow["distance_anchor"] == "earth_center"
    assert len(observer_arrow["base"]) == 3
    assert len(observer_arrow["direction"]) == 3
