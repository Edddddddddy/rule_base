"""Purpose: Resolve per-station configs through the deployment router using STATION_TYPE.

Usage: source ros2_ws/setup_ros_env.sh && python -m pytest tests/unit/test_station_router.py
"""

from pathlib import Path

import pytest

from rule_base.config.load import resolve_config_path, station_type

ROOT = Path(__file__).resolve().parents[2]
ROUTER = ROOT / "configs/live_router.yaml"


def test_station_type_accepts_only_the_ported_station():
    assert station_type({"STATION_TYPE": "pick_place"}) == "pick_place"
    for env in (
        {},
        {"STATION_TYPE": ""},
        {"STATION_TYPE": "drawer"},
        {"STATION_TYPE": "press"},
        {"STATION_TYPE": "station_01"},
        {"STATION_TYPE": "Pick_Place"},
    ):
        with pytest.raises(ValueError, match="STATION_TYPE"):
            station_type(env)


def test_router_resolves_pick_place():
    path, resolved = resolve_config_path(ROUTER, {"STATION_TYPE": "pick_place"})
    assert path == ROOT / "configs/pick_place/live.yaml"
    assert resolved == "pick_place"


@pytest.mark.parametrize("value", [None, "", " ", "invalid", "Press"])
def test_router_requires_valid_station_type(value):
    environ = {} if value is None else {"STATION_TYPE": value}
    with pytest.raises(ValueError, match="STATION_TYPE must be one of"):
        resolve_config_path(ROUTER, environ)


def test_router_rejects_unmapped_station(tmp_path):
    router = tmp_path / "router.yaml"
    router.write_text("by_station: {}\n")
    with pytest.raises(ValueError, match="no config path mapped"):
        resolve_config_path(router, {"STATION_TYPE": "pick_place"})


def test_router_rejects_extra_keys_and_missing_target(tmp_path):
    router = tmp_path / "router.yaml"
    router.write_text("by_station:\n  pick_place: pick.yaml\nextra: true\n")
    with pytest.raises(ValueError, match="only a by_station"):
        resolve_config_path(router, {"STATION_TYPE": "pick_place"})
    router.write_text("by_station:\n  pick_place: missing.yaml\n")
    with pytest.raises(ValueError, match="router target missing"):
        resolve_config_path(router, {"STATION_TYPE": "pick_place"})


def test_router_rejects_target_task_mismatch(tmp_path):
    router = tmp_path / "router.yaml"
    (tmp_path / "press.yaml").write_text("task: press\n")
    router.write_text("by_station:\n  pick_place: press.yaml\n")
    with pytest.raises(ValueError, match="task does not match"):
        resolve_config_path(router, {"STATION_TYPE": "pick_place"})


@pytest.mark.parametrize("environ", [{}, {"STATION_TYPE": "station_01"}])
def test_plain_config_passes_through_and_derives_station(environ):
    config = ROOT / "configs/pick_place/live.yaml"
    path, resolved = resolve_config_path(config, environ)
    assert path == config
    assert resolved == "pick_place"
