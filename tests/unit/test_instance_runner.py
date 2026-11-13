"""Purpose: unit-test the one-round child runner (argv/env/SIGTERM/timeout/exit codes).

Uses tiny fake child processes (never ROS): the real rule_base CLI is only constructed,
not executed. Usage: python3 -m pytest tests/unit/test_instance_runner.py -q
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from rule_base.instance import runner
from rule_base.instance.protocol import (
    CHILD_FAILED,
    CHILD_INTERRUPTED,
    CHILD_OK,
    CHILD_SPAWN_ERROR,
    CHILD_TIMEOUT,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG = REPO_ROOT / "configs/pick_place/mock_ros2_instance.yaml"

#: A child that handles SIGTERM gracefully (exits 0) while sleeping.
GRACEFUL_CHILD = (
    "import signal, sys, time\n"
    "signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))\n"
    "time.sleep(30)\n"
)
#: A child that ignores SIGTERM (forces the SIGKILL escalation path).
STUBBORN_CHILD = (
    "import signal, time\n"
    "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
    "time.sleep(30)\n"
)


def run_fake_child(
    code: str,
    *,
    timeout_s: float = 5.0,
    stop_grace_s: float = 1.0,
    interrupt_after: float | None = None,
    on_tick=None,
) -> tuple[runner.ChildOutcome, float, threading.Event]:
    interrupt = threading.Event()
    if interrupt_after is not None:
        timer = threading.Timer(interrupt_after, interrupt.set)
        timer.daemon = True
        timer.start()
    started = time.monotonic()
    outcome = runner.run_child(
        [sys.executable, "-c", code],
        env=os.environ.copy(),
        cwd=REPO_ROOT,
        timeout_s=timeout_s,
        stop_grace_s=stop_grace_s,
        interrupt=interrupt,
        on_tick=on_tick,
        poll_interval_s=0.02,
    )
    return outcome, time.monotonic() - started, interrupt


# --- frozen argv/env contract ---------------------------------------------
def test_child_argv_live_is_frozen():
    argv = runner.build_child_argv(CONFIG, dry_run=False)
    assert argv[:4] == [sys.executable, "-m", "rule_base.interfaces.node", "--config"]
    assert argv[4] == str(CONFIG.resolve())
    assert argv[5:] == ["--rounds", "1", "--run"]


def test_child_argv_mock_is_frozen():
    argv = runner.build_child_argv(CONFIG, dry_run=True)
    assert argv[:4] == [sys.executable, "-m", "rule_base.interfaces.node", "--config"]
    assert argv[4] == str(CONFIG.resolve())
    assert argv[5:] == ["--rounds", "1", "--dry-run", "--transport", "ros2"]


def test_child_argv_relative_config_is_made_absolute():
    argv = runner.build_child_argv("configs/pick_place/mock_ros2_instance.yaml", dry_run=True)
    assert Path(argv[4]).is_absolute()
    assert argv[4].endswith("rule_base/configs/pick_place/mock_ros2_instance.yaml")


def test_child_argv_custom_executable():
    argv = runner.build_child_argv(CONFIG, dry_run=True, executable="/opt/x/python3")
    assert argv[0] == "/opt/x/python3"


def test_child_env_prepends_this_checkout_src_to_pythonpath():
    env = runner.build_child_env({"PYTHONPATH": "/existing/path", "FOO": "bar"})
    entries = env["PYTHONPATH"].split(os.pathsep)
    assert entries[0] == str(runner.rule_base_src_dir())
    assert entries[1] == "/existing/path"
    assert env["FOO"] == "bar"


def test_child_env_without_existing_pythonpath():
    env = runner.build_child_env({})
    assert env["PYTHONPATH"] == str(runner.rule_base_src_dir())


def test_child_env_uses_explicit_src_dir(tmp_path):
    src = tmp_path / "src"
    src.mkdir()
    env = runner.build_child_env({"PYTHONPATH": ""}, src_dir=src)
    assert env["PYTHONPATH"] == str(src.resolve())


def test_repo_root_is_the_src_parent():
    assert runner.rule_base_src_dir().name == "src"
    assert runner.rule_base_repo_root() == runner.rule_base_src_dir().parent
    assert runner.rule_base_repo_root() == REPO_ROOT


# --- child outcome classification -----------------------------------------
def test_child_ok_exit_zero():
    outcome, elapsed, _ = run_fake_child("pass")
    assert outcome.status == CHILD_OK
    assert outcome.returncode == 0
    assert outcome.ok is True
    assert elapsed < 10


def test_child_failed_nonzero_exit_is_classified():
    outcome, _, _ = run_fake_child("raise SystemExit(3)")
    assert outcome.status == CHILD_FAILED
    assert outcome.returncode == 3
    assert outcome.stage == "round"
    assert "3" in (outcome.error or "")


def test_child_timeout_terminates_the_process():
    outcome, elapsed, _ = run_fake_child("import time; time.sleep(30)", timeout_s=0.3)
    assert outcome.status == CHILD_TIMEOUT
    assert outcome.timed_out is True
    assert outcome.returncode is not None
    assert elapsed < 5.0


def test_round_runner_timeout_and_grace_are_configurable():
    round_runner = runner.RoundRunner(
        CONFIG,
        dry_run=True,
        timeout_s=123.0,
        stop_grace_s=1.5,
        executable=sys.executable,
    )
    assert round_runner.timeout_s == 123.0
    assert round_runner.stop_grace_s == 1.5
    assert round_runner.argv == runner.build_child_argv(CONFIG, dry_run=True)
    assert round_runner.cwd == REPO_ROOT
    assert round_runner.interrupted is False


def test_spawn_error_is_reported_not_raised():
    outcome = runner.run_child(
        ["/definitely/not/a/real/program"],
        env={"PATH": "/usr/bin"},
        cwd=REPO_ROOT,
        timeout_s=1.0,
        stop_grace_s=0.1,
        interrupt=threading.Event(),
    )
    assert outcome.status == CHILD_SPAWN_ERROR
    assert outcome.stage == "spawn"
    assert outcome.error


def test_round_runner_spawn_error_via_bad_executable():
    round_runner = runner.RoundRunner(
        CONFIG, dry_run=True, executable="/definitely/not/a/real/program"
    )
    outcome = round_runner.run()
    assert outcome.status == CHILD_SPAWN_ERROR
    assert outcome.stage == "spawn"


# --- SIGTERM forwarding ----------------------------------------------------
def test_interrupt_forwards_sigterm_to_child():
    outcome, elapsed, _ = run_fake_child(GRACEFUL_CHILD, interrupt_after=0.25)
    assert outcome.status == CHILD_INTERRUPTED
    assert outcome.interrupted is True
    # Graceful child exits 0 on SIGTERM (no SIGKILL escalation needed).
    assert outcome.returncode == 0
    assert elapsed < 5.0


def test_interrupt_escalates_to_sigkill_after_grace():
    outcome, elapsed, _ = run_fake_child(
        STUBBORN_CHILD, stop_grace_s=0.2, interrupt_after=0.25
    )
    assert outcome.status == CHILD_INTERRUPTED
    assert outcome.returncode is not None and outcome.returncode < 0  # killed by signal
    assert elapsed < 5.0


def test_interrupt_before_spawn_does_not_start_a_child():
    interrupt = threading.Event()
    interrupt.set()
    started = time.monotonic()
    outcome = runner.run_child(
        [sys.executable, "-c", "import time; time.sleep(10)"],
        env=os.environ.copy(),
        cwd=REPO_ROOT,
        timeout_s=5.0,
        stop_grace_s=0.5,
        interrupt=interrupt,
    )
    assert outcome.status == CHILD_INTERRUPTED
    assert outcome.returncode is None
    assert time.monotonic() - started < 2.0


def test_on_tick_is_called_while_the_child_runs():
    ticks = []
    outcome, _, _ = run_fake_child(
        "import time; time.sleep(0.3)", timeout_s=5.0, on_tick=lambda: ticks.append(1)
    )
    assert outcome.status == CHILD_OK
    assert len(ticks) >= 3


def test_round_runner_interrupt_is_idempotent():
    round_runner = runner.RoundRunner(CONFIG, dry_run=True)
    assert round_runner.interrupted is False
    round_runner.interrupt()
    round_runner.interrupt()
    assert round_runner.interrupted is True
    outcome = round_runner.run()
    assert outcome.status == CHILD_INTERRUPTED


def test_timeout_must_be_positive():
    with pytest.raises(ValueError, match="timeout_s"):
        runner.run_child(
            [sys.executable, "-c", "pass"],
            env={},
            cwd=REPO_ROOT,
            timeout_s=0,
            stop_grace_s=0.1,
            interrupt=threading.Event(),
        )


def test_no_child_is_left_running_after_timeout():
    """A timed-out fake child must be reaped (no zombie / lingering sleeper)."""
    marker = f"rb_instance_unit_{os.getpid()}_{time.monotonic_ns()}"
    code = f"import time; time.sleep(30)  # {marker}"
    outcome, _, _ = run_fake_child(code, timeout_s=0.3)
    assert outcome.status == CHILD_TIMEOUT
    # The process is gone: a second SIGTERM probe must not find it.
    probe = subprocess.run(
        ["pgrep", "-f", marker], capture_output=True, text=True, check=False
    )
    assert probe.stdout.strip() == "", f"lingering child: {probe.stdout}"
