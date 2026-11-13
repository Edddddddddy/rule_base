"""One-round child runner for the rule_base instance (contracts §2).

The 026 instance entry does not import the business core: every round is a fresh
``python3 -m rule_base.interfaces.node --config <yaml> --rounds 1`` child process
(``--run`` for live, ``--dry-run --transport ros2`` for mock/test). This module owns
the argv/env contract, the single-round timeout, SIGTERM forwarding (SIGTERM then
SIGKILL after a grace period) and exit-code classification. It has no ROS imports.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import rule_base
from rule_base.instance.protocol import (
    CHILD_FAILED,
    CHILD_INTERRUPTED,
    CHILD_OK,
    CHILD_SPAWN_ERROR,
    CHILD_TIMEOUT,
)

CHILD_MODULE = "rule_base.interfaces.node"

#: contracts §4: one round's ``stopped`` wait budget is 600 s; the child is killed with
#: SIGTERM -> (grace) -> SIGKILL when it overruns.
DEFAULT_ROUND_TIMEOUT_S = 600.0
DEFAULT_STOP_GRACE_S = 5.0
DEFAULT_POLL_INTERVAL_S = 0.05
KILL_WAIT_S = 2.0


@dataclass(frozen=True)
class ChildOutcome:
    """Classified result of one child round."""

    status: str
    returncode: int | None
    stage: str
    error: str | None = None

    @property
    def ok(self) -> bool:
        return self.status == CHILD_OK

    @property
    def interrupted(self) -> bool:
        return self.status == CHILD_INTERRUPTED

    @property
    def timed_out(self) -> bool:
        return self.status == CHILD_TIMEOUT


def rule_base_src_dir() -> Path:
    """Locate this checkout's ``src`` directory (works for editable installs too)."""
    package_file = getattr(rule_base, "__file__", None)
    if package_file is None:  # pragma: no cover - namespace package corner case
        raise RuntimeError("cannot locate the rule_base package")
    return Path(package_file).resolve().parents[1]


def rule_base_repo_root() -> Path:
    """Repository root (= parent of ``src``), used as the child working directory."""
    return rule_base_src_dir().parent


def build_child_argv(
    config_path: Path | str,
    *,
    dry_run: bool,
    executable: str | None = None,
) -> list[str]:
    """Build the frozen rule_base CLI argv for one round (Lead-frozen contract)."""
    absolute_config = str(Path(config_path).expanduser().resolve())
    python = executable or sys.executable
    if dry_run:
        return [
            python,
            "-m",
            CHILD_MODULE,
            "--config",
            absolute_config,
            "--rounds",
            "1",
            "--dry-run",
            "--transport",
            "ros2",
        ]
    return [
        python,
        "-m",
        CHILD_MODULE,
        "--config",
        absolute_config,
        "--rounds",
        "1",
        "--run",
    ]


def build_child_env(
    base_env: Mapping[str, str] | None = None,
    *,
    src_dir: Path | str | None = None,
) -> dict[str, str]:
    """Inherit the current env and prepend this checkout's ``src`` to ``PYTHONPATH``.

    The machine has an editable ``so101-rule-base`` install pointing at another copy
    of the repository; prepending the running package's own ``src`` guarantees the
    child imports the same code as the entry (contracts §2 / S1 environment note).
    """
    env = dict(os.environ if base_env is None else base_env)
    source = Path(src_dir) if src_dir is not None else rule_base_src_dir()
    prefix = str(source.resolve())
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = prefix if not existing else prefix + os.pathsep + existing
    return env


def _kill(proc: subprocess.Popen, wait_s: float = KILL_WAIT_S) -> None:
    try:
        proc.kill()
    except ProcessLookupError:  # pragma: no cover - already reaped
        return
    try:
        proc.wait(timeout=wait_s)
    except subprocess.TimeoutExpired:  # pragma: no cover - uninterruptible child
        pass


def _terminate(
    proc: subprocess.Popen,
    *,
    status: str,
    stage: str,
    error: str,
    stop_grace_s: float,
) -> ChildOutcome:
    """SIGTERM the child, escalate to SIGKILL after ``stop_grace_s``; return outcome."""
    try:
        proc.send_signal(signal.SIGTERM)
    except ProcessLookupError:  # pragma: no cover - raced with natural exit
        pass
    try:
        proc.wait(timeout=max(0.0, stop_grace_s))
    except subprocess.TimeoutExpired:
        _kill(proc)
    return ChildOutcome(status=status, returncode=proc.poll(), stage=stage, error=error)


def run_child(
    argv: Sequence[str],
    *,
    env: Mapping[str, str],
    cwd: Path | str | None,
    timeout_s: float,
    stop_grace_s: float,
    interrupt: threading.Event,
    on_tick: Callable[[], None] | None = None,
    poll_interval_s: float = DEFAULT_POLL_INTERVAL_S,
    popen_factory: Callable[..., subprocess.Popen] = subprocess.Popen,
) -> ChildOutcome:
    """Run one child round to completion, classifying its outcome (contracts §2)."""
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive")
    if interrupt.is_set():
        return ChildOutcome(
            status=CHILD_INTERRUPTED,
            returncode=None,
            stage="round",
            error="interrupted before the round started",
        )
    try:
        proc = popen_factory(
            list(argv),
            env=dict(env),
            cwd=None if cwd is None else str(cwd),
        )
    except OSError as exc:
        return ChildOutcome(
            status=CHILD_SPAWN_ERROR,
            returncode=None,
            stage="spawn",
            error=f"{type(exc).__name__}: {exc}",
        )
    deadline = time.monotonic() + timeout_s
    try:
        while True:
            if interrupt.is_set():
                return _terminate(
                    proc,
                    status=CHILD_INTERRUPTED,
                    stage="round",
                    error="interrupted (SIGTERM/SIGINT) while the round was running",
                    stop_grace_s=stop_grace_s,
                )
            returncode = proc.poll()
            if returncode is not None:
                if returncode == 0:
                    return ChildOutcome(status=CHILD_OK, returncode=0, stage="round")
                return ChildOutcome(
                    status=CHILD_FAILED,
                    returncode=returncode,
                    stage="round",
                    error=f"child exited with code {returncode}",
                )
            if time.monotonic() >= deadline:
                return _terminate(
                    proc,
                    status=CHILD_TIMEOUT,
                    stage="timeout",
                    error=f"round exceeded the {timeout_s:g}s timeout",
                    stop_grace_s=stop_grace_s,
                )
            if on_tick is not None:
                on_tick()
            try:
                proc.wait(timeout=poll_interval_s)
            except subprocess.TimeoutExpired:
                pass
    finally:
        if proc.poll() is None:  # pragma: no cover - defensive: never leak a child
            _kill(proc)


class RoundRunner:
    """Configure one-round child execution for the instance entry."""

    def __init__(
        self,
        config_path: Path | str,
        *,
        dry_run: bool,
        timeout_s: float = DEFAULT_ROUND_TIMEOUT_S,
        stop_grace_s: float = DEFAULT_STOP_GRACE_S,
        base_env: Mapping[str, str] | None = None,
        src_dir: Path | str | None = None,
        cwd: Path | str | None = None,
        executable: str | None = None,
        poll_interval_s: float = DEFAULT_POLL_INTERVAL_S,
    ) -> None:
        self.config_path = Path(config_path).expanduser().resolve()
        self.dry_run = bool(dry_run)
        self.timeout_s = float(timeout_s)
        self.stop_grace_s = float(stop_grace_s)
        self.src_dir = Path(src_dir).resolve() if src_dir is not None else rule_base_src_dir()
        self.cwd = Path(cwd).resolve() if cwd is not None else self.src_dir.parent
        self.argv = build_child_argv(
            self.config_path, dry_run=self.dry_run, executable=executable
        )
        self.env = build_child_env(base_env, src_dir=self.src_dir)
        self.poll_interval_s = float(poll_interval_s)
        self._interrupt = threading.Event()

    @property
    def interrupted(self) -> bool:
        return self._interrupt.is_set()

    def interrupt(self) -> None:
        """Request shutdown; the run loop SIGTERMs the current child at its next poll."""
        self._interrupt.set()

    def run(self, on_tick: Callable[[], None] | None = None) -> ChildOutcome:
        return run_child(
            self.argv,
            env=self.env,
            cwd=self.cwd,
            timeout_s=self.timeout_s,
            stop_grace_s=self.stop_grace_s,
            interrupt=self._interrupt,
            on_tick=on_tick,
            poll_interval_s=self.poll_interval_s,
        )
