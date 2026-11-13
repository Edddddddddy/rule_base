"""Purpose: unit-test the hold keepalive command builder/state (contracts §2 D4).

Usage: python3 -m pytest tests/unit/test_instance_keepalive.py -q
"""

import pytest

from rule_base.instance.keepalive import (
    DEFAULT_HOLD_HZ,
    MAX_HOLD_HZ,
    MIN_HOLD_HZ,
    HoldCommand,
    Keepalive,
    build_hold_command,
)

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


# --- command construction --------------------------------------------------
def test_build_hold_command_uses_measured_positions_in_configured_order():
    command = build_hold_command(JOINTS, JOINTS, (0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    assert command == HoldCommand(JOINTS, (0.1, 0.2, 0.3, 0.4, 0.5, 0.6))


def test_build_hold_command_reorders_scrambled_sample_names():
    scrambled = tuple(reversed(JOINTS))
    command = build_hold_command(JOINTS, scrambled, (6.0, 5.0, 4.0, 3.0, 2.0, 1.0))
    assert command is not None
    assert command.names == JOINTS
    assert command.positions == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


def test_build_hold_command_holds_measured_gripper_position():
    # D4: the hold target equals the measured positions for all six joints (gripper
    # included: never widen/close the gripper while idle).
    command = build_hold_command(JOINTS, JOINTS, (0.0, -1.5, 1.4, 0.7, 0.1, 0.123))
    assert command is not None
    assert command.positions[-1] == pytest.approx(0.123)


def test_build_hold_command_returns_none_without_a_complete_sample():
    partial = JOINTS[:-1]
    assert build_hold_command(JOINTS, partial, (0.0,) * 5) is None


def test_build_hold_command_returns_none_for_non_finite_values():
    bad = (0.0, 0.0, 0.0, 0.0, 0.0, float("nan"))
    assert build_hold_command(JOINTS, JOINTS, bad) is None
    inf = (0.0, 0.0, 0.0, 0.0, 0.0, float("inf"))
    assert build_hold_command(JOINTS, JOINTS, inf) is None


def test_build_hold_command_returns_none_for_mismatched_lengths():
    assert build_hold_command(JOINTS, JOINTS, (0.0,) * 5) is None


# --- frequency contract ----------------------------------------------------
def test_default_hold_rate_is_within_the_contract_band():
    assert MIN_HOLD_HZ <= DEFAULT_HOLD_HZ <= MAX_HOLD_HZ
    keepalive = Keepalive(JOINTS)
    assert keepalive.hz == 4.0
    assert keepalive.period_s == pytest.approx(0.25)


@pytest.mark.parametrize("hz", [2.0, 3.0, 4.0, 5.0])
def test_hold_rate_accepts_2_to_5_hz(hz):
    assert Keepalive(JOINTS, hz=hz).hz == hz


@pytest.mark.parametrize("hz", [0.5, 1.0, 1.99, 5.01, 10.0])
def test_hold_rate_rejects_out_of_band(hz):
    with pytest.raises(ValueError, match="2-5|2.0, 5.0"):
        Keepalive(JOINTS, hz=hz)


# --- start/stop state ------------------------------------------------------
def test_keepalive_does_not_publish_before_start_or_without_sample():
    keepalive = Keepalive(JOINTS)
    assert keepalive.running is False
    assert keepalive.next_command() is None
    keepalive.start()
    assert keepalive.running is True
    assert keepalive.next_command() is None  # no measured sample yet
    assert keepalive.has_sample is False


def test_keepalive_publishes_after_start_and_sample():
    keepalive = Keepalive(JOINTS)
    assert keepalive.on_joint_states(JOINTS, (1, 2, 3, 4, 5, 6)) is True
    assert keepalive.has_sample is True
    keepalive.start()
    command = keepalive.next_command()
    assert command == HoldCommand(JOINTS, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))


def test_keepalive_stop_suspends_hold_commands():
    keepalive = Keepalive(JOINTS)
    keepalive.on_joint_states(JOINTS, (1, 2, 3, 4, 5, 6))
    keepalive.start()
    assert keepalive.next_command() is not None
    keepalive.stop()
    assert keepalive.running is False
    assert keepalive.next_command() is None  # strictly paused while a child round runs
    keepalive.start()
    assert keepalive.next_command() is not None


def test_keepalive_tracks_latest_sample():
    keepalive = Keepalive(JOINTS)
    keepalive.on_joint_states(JOINTS, (1, 2, 3, 4, 5, 6))
    keepalive.on_joint_states(JOINTS, (6, 5, 4, 3, 2, 1), now=123.0)
    keepalive.start()
    assert keepalive.next_command() == HoldCommand(JOINTS, (6.0, 5.0, 4.0, 3.0, 2.0, 1.0))
    assert keepalive.last_sample_at == 123.0


def test_keepalive_ignores_malformed_samples():
    keepalive = Keepalive(JOINTS)
    assert keepalive.on_joint_states(JOINTS, (1, 2, 3)) is False
    assert keepalive.has_sample is False
    keepalive.on_joint_states(JOINTS, (1, 2, 3, 4, 5, 6))
    assert keepalive.on_joint_states(("wrong",), (0.0,)) is False
    assert keepalive.has_sample is True


def test_keepalive_requires_distinct_non_empty_joint_names():
    with pytest.raises(ValueError, match="distinct"):
        Keepalive(("a", "a"))
    with pytest.raises(ValueError, match="empty"):
        Keepalive(())
