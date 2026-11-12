"""Purpose: Check the public process contract, hardware-free default and CLI gates.

Usage: source ros2_ws/setup_ros_env.sh && python -m pytest tests/smoke/test_entrypoint.py
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from rule_base.interfaces import node as node_module

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "configs/pick_place/offline.yaml"


def run_cli(*args: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    process_env = {
        key: value
        for key, value in os.environ.items()
        if key not in {"PYTHONPATH", "INFERENCE_NAMESPACE", "DOMAIN_ID", "PICK_HOLD_DURATION_S"}
    }
    if env is not None:
        process_env.update(env)
    return subprocess.run(
        [sys.executable, "-m", "rule_base.interfaces.node", *args],
        cwd=ROOT,
        env=process_env,
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )


def json_events(output: str) -> list[dict[str, object]]:
    events = []
    for line in output.splitlines():
        line = line.strip()
        if line.startswith("{"):
            events.append(json.loads(line))
    return events


def test_help_lists_port_transport_and_rounds():
    result = run_cli("--help")
    assert result.returncode == 0
    for option in ("--config", "--dry-run", "--run", "--transport", "--rounds"):
        assert option in result.stdout
    assert "ros2" in result.stdout


def test_default_dry_run_uses_fake_transport_and_one_round():
    result = run_cli("--config", str(CONFIG), "--dry-run")
    assert result.returncode == 0, result.stderr
    events = json_events(result.stdout + result.stderr)
    startup = next(event for event in events if event["event"] == "startup")
    assert startup["transport"] == "fake"
    assert startup["live"] is False
    assert [event["event"] for event in events].count("round_result") == 1
    shutdown = next(event for event in events if event["event"] == "shutdown")
    assert shutdown["cleanup_confirmed"] is True


def test_dry_run_rounds_emit_one_result_per_round():
    result = run_cli("--config", str(CONFIG), "--dry-run", "--rounds", "2")
    assert result.returncode == 0, result.stderr
    results = [
        event
        for event in json_events(result.stdout + result.stderr)
        if event["event"] == "round_result"
    ]
    assert len(results) == 2
    assert all(event["status"] == "completed" for event in results)


def test_cli_round_id_is_used_for_round_result():
    """S2-5：`--round-id` 透传到 round_result.round_id（capture 与 recorder metadata 同源）。"""
    result = run_cli("--config", str(CONFIG), "--dry-run", "--round-id", "c" * 32)
    assert result.returncode == 0, result.stderr
    events = json_events(result.stdout + result.stderr)
    round_result = next(event for event in events if event["event"] == "round_result")
    assert round_result["round_id"] == "c" * 32


def test_cli_round_id_defaults_to_internal_uuid_hex():
    result = run_cli("--config", str(CONFIG), "--dry-run")
    assert result.returncode == 0, result.stderr
    events = json_events(result.stdout + result.stderr)
    round_result = next(event for event in events if event["event"] == "round_result")
    round_id = round_result["round_id"]
    assert len(round_id) == 32
    int(round_id, 16)


def test_invalid_rounds_value_is_rejected():
    result = run_cli("--config", str(CONFIG), "--dry-run", "--rounds", "0")
    assert result.returncode == 2
    assert "--rounds must be a positive integer" in result.stderr


def test_run_requires_the_ros2_transport():
    result = run_cli("--config", str(CONFIG), "--run", "--transport", "fake")
    assert result.returncode == 2
    assert "requires the ROS 2 transport" in result.stderr


def test_run_without_transport_defaults_to_ros2(monkeypatch):
    """S6d：--run 省略 --transport 时默认 ros2，不再回落 fake 后 fatal。"""
    captured: dict[str, object] = {}

    async def fake_serve(config_path, **kwargs):
        captured["config_path"] = config_path
        captured.update(kwargs)
        return 0

    monkeypatch.setattr(node_module, "serve", fake_serve)
    monkeypatch.setattr(sys, "argv", ["rule_base", "--config", str(CONFIG), "--run"])
    assert node_module.main() == 0
    assert captured["live"] is True
    assert captured["transport"] == "ros2"
    assert captured["rounds"] == 1


def test_retired_press_target_is_rejected():
    result = run_cli("--config", str(CONFIG), "--dry-run", "--target", "244")
    assert result.returncode == 1
    assert "retired Press station" in result.stdout + result.stderr


@pytest.mark.parametrize("station", ["drawer", "press", "station_01"])
def test_router_rejects_unknown_station_type(station):
    result = run_cli(
        "--config",
        str(ROOT / "configs/live_router.yaml"),
        "--dry-run",
        env={"STATION_TYPE": station},
    )
    assert result.returncode == 1
    assert "STATION_TYPE" in result.stdout + result.stderr


def test_dry_run_fake_transport_does_not_import_rclpy():
    code = (
        "import sys\n"
        "from rule_base.application.runner import InstanceRunner\n"
        "assert 'rclpy' not in sys.modules, 'fake transport must not import rclpy'\n"
        "assert 'rule_base.infrastructure.ros2' not in sys.modules\n"
        "print('ok')\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout
