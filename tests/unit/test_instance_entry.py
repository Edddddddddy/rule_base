"""Purpose: unit-test entry CLI/env failure paths (exit 2, JSON logs) without ROS.

The happy path needs a live ROS graph and is covered by
tests/integration/test_instance_entry.py. Usage: python3 -m pytest
tests/unit/test_instance_entry.py -q
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

from rule_base.instance import entry

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG = REPO_ROOT / "configs/pick_place/mock_ros2_instance.yaml"

VALID_ENV = {
    "INFERENCE_NAMESPACE": "",
    "DOMAIN_ID": "229",
    "STATION_TYPE": "pick_place",
    "CLIENT_ID": "client-1",
    "LEASE_ID": "7",
    "INFERENCE_ID": "3",
}


def apply_env(monkeypatch, overrides: dict[str, str] | None = None) -> None:
    """Isolate the instance env: only VALID_ENV (+ overrides, None = unset)."""
    for key in (
        "INFERENCE_NAMESPACE",
        "DOMAIN_ID",
        "STATION_TYPE",
        "CLIENT_ID",
        "LEASE_ID",
        "INFERENCE_ID",
        "ROS_DOMAIN_ID",
    ):
        monkeypatch.delenv(key, raising=False)
    values = dict(VALID_ENV)
    values.update(overrides or {})
    for key, value in values.items():
        if value is None:
            monkeypatch.delenv(key, raising=False)
        else:
            monkeypatch.setenv(key, value)


def json_lines(capsys) -> list[dict]:
    out = capsys.readouterr().out
    lines = [line for line in out.splitlines() if line.strip()]
    assert lines, "entry printed no structured log lines"
    return [json.loads(line) for line in lines]


def force_ros_unavailable(monkeypatch) -> None:
    """Make ``import rclpy`` fail so the entry stops before touching any ROS graph."""
    monkeypatch.setitem(sys.modules, "rclpy", None)


# --- CLI ------------------------------------------------------------------
def test_config_argument_is_required_with_exit_code_2(capsys):
    with pytest.raises(SystemExit) as excinfo:
        entry.parse_args([])
    assert excinfo.value.code == 2
    assert "--config" in capsys.readouterr().err


def test_help_works_without_ros(capsys):
    with pytest.raises(SystemExit) as excinfo:
        entry.parse_args(["--help"])
    assert excinfo.value.code == 0
    assert "rule_base 026 instance entry" in capsys.readouterr().out


@pytest.mark.parametrize("argv", [["--hold-hz", "1.0"], ["--hold-hz", "5.5"]])
def test_hold_hz_outside_contract_band_is_rejected(argv):
    with pytest.raises(SystemExit) as excinfo:
        entry.parse_args(["--config", str(CONFIG), *argv])
    assert excinfo.value.code == 2


def test_negative_and_zero_timeouts_are_rejected():
    for argv in (["--warmup-timeout", "0"], ["--round-timeout", "-1"]):
        with pytest.raises(SystemExit) as excinfo:
            entry.parse_args(["--config", str(CONFIG), *argv])
        assert excinfo.value.code == 2


def test_defaults_match_the_frozen_protocol():
    args = entry.parse_args(["--config", str(CONFIG)])
    assert args.dry_run is False
    assert args.warmup_timeout == 30.0
    assert args.round_timeout == 600.0
    assert args.hold_hz == 4.0
    assert args.stop_grace == 5.0


# --- config failures ------------------------------------------------------
def test_missing_config_file_exits_2_with_config_stage(monkeypatch, capsys):
    apply_env(monkeypatch)
    assert entry.main(["--config", str(REPO_ROOT / "does-not-exist.yaml")]) == 2
    lines = json_lines(capsys)
    assert lines[0]["event"] == "instance_failed"
    assert lines[0]["stage"] == "config"


def test_wrong_task_config_exits_2(tmp_path, monkeypatch, capsys):
    apply_env(monkeypatch)
    bad = tmp_path / "bad.yaml"
    bad.write_text("task: drawer\n", encoding="utf-8")
    assert entry.main(["--config", str(bad)]) == 2
    assert json_lines(capsys)[0]["stage"] == "config"


# --- environment failures (contracts §1.2) --------------------------------
def test_missing_domain_id_exits_2(monkeypatch, capsys):
    apply_env(monkeypatch, {"DOMAIN_ID": None})
    assert entry.main(["--config", str(CONFIG)]) == 2
    lines = json_lines(capsys)
    assert lines[0]["stage"] == "environment"
    assert "DOMAIN_ID" in lines[0]["error"]


def test_missing_station_type_exits_2(monkeypatch, capsys):
    apply_env(monkeypatch, {"STATION_TYPE": None})
    assert entry.main(["--config", str(CONFIG)]) == 2
    assert json_lines(capsys)[0]["stage"] == "environment"


def test_non_pick_place_station_type_exits_2(monkeypatch, capsys):
    apply_env(monkeypatch, {"STATION_TYPE": "drawer"})
    assert entry.main(["--config", str(CONFIG)]) == 2
    assert json_lines(capsys)[0]["stage"] == "environment"


def test_domain_id_mismatch_exits_2(monkeypatch, capsys):
    apply_env(monkeypatch, {"DOMAIN_ID": "229", "ROS_DOMAIN_ID": "193"})
    assert entry.main(["--config", str(CONFIG)]) == 2
    lines = json_lines(capsys)
    assert lines[0]["stage"] == "environment"
    assert "ROS_DOMAIN_ID" in lines[0]["error"]


def test_relative_namespace_exits_2(monkeypatch, capsys):
    apply_env(monkeypatch, {"INFERENCE_NAMESPACE": "station1"})
    assert entry.main(["--config", str(CONFIG)]) == 2
    assert json_lines(capsys)[0]["stage"] == "environment"


def test_domain_id_is_exported_as_ros_domain_id(monkeypatch, capsys):
    """contracts §1.2: derive ROS_DOMAIN_ID from DOMAIN_ID before rclpy.init."""
    apply_env(monkeypatch, {"DOMAIN_ID": "229", "ROS_DOMAIN_ID": None})
    force_ros_unavailable(monkeypatch)
    assert entry.main(["--config", str(CONFIG)]) == 2
    import os

    assert os.environ["ROS_DOMAIN_ID"] == "229"
    lines = json_lines(capsys)
    assert lines[0]["event"] == "instance_starting"
    assert lines[-1]["event"] == "instance_failed"
    assert lines[-1]["stage"] == "ros2_init"


def test_missing_client_and_lease_only_warn(monkeypatch, capsys):
    apply_env(monkeypatch, {"CLIENT_ID": "", "LEASE_ID": ""})
    force_ros_unavailable(monkeypatch)
    assert entry.main(["--config", str(CONFIG)]) == 2
    lines = json_lines(capsys)
    stages = [line for line in lines if line["event"] == "instance_warning"]
    assert any("CLIENT_ID" in warning["warning"] for warning in stages)
    assert any("LEASE_ID" in warning["warning"] for warning in stages)
    # The failure is the missing ROS runtime, not the optional identifiers.
    assert lines[-1]["stage"] == "ros2_init"


def test_missing_rclpy_exits_2_with_ros2_init_stage(monkeypatch, capsys):
    apply_env(monkeypatch)
    force_ros_unavailable(monkeypatch)
    assert entry.main(["--config", str(CONFIG)]) == 2
    assert json_lines(capsys)[-1]["stage"] == "ros2_init"


def test_structured_startup_log_fields(monkeypatch, capsys):
    apply_env(
        monkeypatch,
        {"INFERENCE_NAMESPACE": "/station1", "DOMAIN_ID": "229", "INFERENCE_ID": "42"},
    )
    force_ros_unavailable(monkeypatch)
    assert entry.main(["--config", str(CONFIG)]) == 2
    starting = json_lines(capsys)[0]
    assert starting["event"] == "instance_starting"
    assert starting["namespace"] == "/station1"
    assert starting["domain_id"] == 229
    assert starting["station_type"] == "pick_place"
    assert starting["client_id"] == "client-1"
    assert starting["lease_id"] == "7"
    assert starting["inference_id"] == 42
    assert starting["dry_run"] is False
