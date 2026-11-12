"""Purpose: End-to-end loopback of infrastructure/ros2 against a minimal arm double.

Runs the real CLI (`--dry-run --transport ros2`) against tests/integration/
ros2_mock_arm.py to prove the adapter wiring without CORA or hardware.

Usage:
    source ros2_ws/setup_ros_env.sh
    python -m pytest tests/integration/test_ros2_loopback.py -q -s
"""

from __future__ import annotations

import json
import select
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
CONFIG = Path(__file__).resolve().parent / "config_ros2_loopback.yaml"
MOCK = Path(__file__).resolve().parent / "ros2_mock_arm.py"

pytestmark = pytest.mark.integration


def _require_ros2() -> None:
    try:
        import arm_msgs.srv  # noqa: F401
        import rclpy  # noqa: F401
    except ImportError:  # pragma: no cover - depends on the shell environment
        pytest.skip("source ros2_ws/setup_ros_env.sh before running integration tests")


def _read_ready(process: subprocess.Popen[str], timeout_s: float = 15) -> None:
    deadline = time.monotonic() + timeout_s
    assert process.stdout is not None
    while time.monotonic() < deadline:
        ready, _, _ = select.select([process.stdout], [], [], 0.2)
        if not ready:
            continue
        line = process.stdout.readline()
        if not line:
            break
        if '"mock_ready"' in line:
            return
    stderr = process.stderr.read() if process.stderr is not None else ""
    pytest.fail(f"mock arm server did not become ready: {stderr}")


def _events(output: str) -> list[dict[str, object]]:
    return [json.loads(line) for line in output.splitlines() if line.strip().startswith("{")]


def test_ros2_transport_runs_two_completed_rounds():
    _require_ros2()
    server = subprocess.Popen(
        [sys.executable, str(MOCK), "--prefix", "/arm"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        _read_ready(server)
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "rule_base.interfaces.node",
                "--config",
                str(CONFIG),
                "--dry-run",
                "--transport",
                "ros2",
                "--rounds",
                "2",
            ],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=240,
            check=False,
        )
        output = result.stdout + result.stderr
        assert result.returncode == 0, output
        events = _events(output)
        startup = next(event for event in events if event["event"] == "startup")
        assert startup["transport"] == "ros2"
        assert startup["live"] is False
        assert any(event["event"] == "ros2_transport_open" for event in events)
        results = [event for event in events if event["event"] == "round_result"]
        assert len(results) == 2, output
        assert all(event["status"] == "completed" for event in results), results
        assert all(event["stage"] == "home_end" for event in results), results
        # D-094: every round streams home -> observation -> pick -> place -> home.
        stages = [event["stage"] for event in events if event["event"] == "stage"]
        for stage in ("home_start", "move_to_observation", "lower_to_place", "home_end"):
            assert stage in stages, stages
        assert (
            stages.index("home_start")
            < stages.index("move_to_observation")
            < stages.index("lower_to_place")
            < stages.index("home_end")
        ), stages
        assert stages.count("home_end") == 2
        assert len([event for event in events if event["event"] == "home_complete"]) == 4
        shutdown = next(event for event in events if event["event"] == "shutdown")
        assert shutdown["cleanup_confirmed"] is True
        assert server.poll() is None, "mock arm server exited during the rounds"
    finally:
        server.terminate()
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:  # pragma: no cover
            server.kill()
            server.wait(timeout=5)
