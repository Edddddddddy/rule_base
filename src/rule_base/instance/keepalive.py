"""Lease-keepalive hold heartbeat for the rule_base instance (contracts §2 / D4).

The inference/factory manager owns the arm authority; while the rule_base instance is
idle (warmup finished until a child round starts, and between rounds) it must publish
``<prefix>/joint_command`` traffic so arm_app keeps the lease alive. The hold target is
always the latest measured ``<prefix>/joint_states`` position (no attitude change).

Publishing is strictly paused while a rule_base child round runs: the child owns the
arm traffic then and two writers would fight.

This module only holds the pure command/state logic (no ROS imports) so it is unit
testable offline; :mod:`rule_base.instance.entry` wires the actual ROS publisher and
timer.
"""

from __future__ import annotations

import math
import time
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

#: contracts §2 D4: hold heartbeat must stay within 2–5 Hz (4 Hz is a safe default:
#: arm_app drops the lease after 5 s without traffic, so 4 Hz leaves >4 s of margin).
DEFAULT_HOLD_HZ = 4.0
MIN_HOLD_HZ = 2.0
MAX_HOLD_HZ = 5.0


@dataclass(frozen=True)
class HoldCommand:
    """A ``sensor_msgs/JointState`` hold target in the configured joint order."""

    names: tuple[str, ...]
    positions: tuple[float, ...]


def build_hold_command(
    joint_names: Sequence[str],
    names: Iterable[str],
    positions: Iterable[float],
) -> HoldCommand | None:
    """Build a hold command from one measured ``joint_states`` sample.

    Returns ``None`` when the sample is incomplete/non-finite, so the caller simply
    skips that period instead of publishing a malformed command. Output order always
    follows the configured ``joint_names`` (the frozen six-joint order).
    """
    expected = tuple(joint_names)
    sample_names = tuple(names)
    sample_positions = tuple(positions)
    if len(sample_names) != len(sample_positions):
        return None
    measured: dict[str, float] = {}
    for name, value in zip(sample_names, sample_positions, strict=True):
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(numeric):
            return None
        measured[name] = numeric
    try:
        return HoldCommand(expected, tuple(measured[name] for name in expected))
    except KeyError:
        return None


class Keepalive:
    """Pure start/stop state plus the latest measured sample for hold commands."""

    def __init__(self, joint_names: Sequence[str], *, hz: float = DEFAULT_HOLD_HZ) -> None:
        names = tuple(joint_names)
        if not names:
            raise ValueError("joint_names must not be empty")
        if len(set(names)) != len(names):
            raise ValueError("joint_names must be distinct")
        if not MIN_HOLD_HZ <= float(hz) <= MAX_HOLD_HZ:
            raise ValueError(
                f"hold frequency {hz} must be within [{MIN_HOLD_HZ}, {MAX_HOLD_HZ}] Hz "
                "(contracts §2 D4)"
            )
        self.joint_names = names
        self.hz = float(hz)
        self._running = False
        self._measured: dict[str, float] | None = None
        self._last_sample_at: float | None = None

    # -- state ------------------------------------------------------------
    @property
    def running(self) -> bool:
        return self._running

    @property
    def period_s(self) -> float:
        return 1.0 / self.hz

    @property
    def has_sample(self) -> bool:
        return self._measured is not None

    @property
    def last_sample_at(self) -> float | None:
        return self._last_sample_at

    def start(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False

    # -- samples ----------------------------------------------------------
    def on_joint_states(
        self,
        names: Iterable[str],
        positions: Iterable[float],
        *,
        now: float | None = None,
    ) -> bool:
        """Store one measured sample; return True when it is usable for hold commands."""
        command = build_hold_command(self.joint_names, names, positions)
        if command is None:
            return False
        self._measured = dict(zip(command.names, command.positions, strict=True))
        self._last_sample_at = time.monotonic() if now is None else now
        return True

    def next_command(self) -> HoldCommand | None:
        """Return the next hold command (or None when stopped / no usable sample)."""
        if not self._running or self._measured is None:
            return None
        try:
            positions = tuple(self._measured[name] for name in self.joint_names)
        except KeyError:  # pragma: no cover - guarded by on_joint_states
            return None
        return HoldCommand(self.joint_names, positions)
