"""Purpose: Guard grasp feedback, gripper refresh, hold loss and stage timeouts.

Usage: source ros2_ws/setup_ros_env.sh && python -m pytest tests/unit/test_pick_place_feedback.py
"""

from dataclasses import replace
from threading import Event

import pytest

from pick_place_support import offline_settings, record_events, recorded
from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import ContractError
from rule_base.infrastructure.fakes.runtime import FakePickArm, FakePickEnvironment
from rule_base.infrastructure.transform.math import errors
from rule_base.stations.pick_place import execution as module
from rule_base.stations.pick_place.execution import PickRound


@pytest.fixture
def settings():
    return offline_settings()


def test_pick_lost_grasp_returns_home_before_releasing_object(settings):
    environment = FakePickEnvironment(settings)

    class Lost(FakePickArm):
        def move(self, target):
            super().move(target)
            if (
                cycle.stage == "move_to_post_grasp"
                and self.current.gripper_width_m >= settings.profile.grasp_min_width_m
                and errors(target, cycle.plan.grasp)[0] > 0.05
            ):
                self.current = replace(self.current, gripper_width_m=0.07)

    arm = Lost(settings)
    cycle = PickRound(settings, environment, lambda: arm, Event())
    result = cycle.run("lost")
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted"
    assert "grasp evidence lost while holding" in result.message
    assert "lost_for_s=" in result.message
    assert "grasp_feedback=" in result.message
    assert arm.current.pose.position == settings.profile.post_grasp_position_arm_m
    assert arm.current.gripper_width_m == settings.profile.gripper_open_m
    assert arm.calls[-3:] == ["stop", "release", "close"]
    assert arm.calls.count("gripper") >= 2


def test_pick_tolerates_one_bad_holding_sample_before_lift(settings):
    environment = FakePickEnvironment(settings)

    class TransientFeedback(FakePickArm):
        injected = False

        def feedback(self):
            feedback = super().feedback()
            if cycle.stage == "move_to_post_grasp" and cycle.holding and not self.injected:
                self.injected = True
                return replace(feedback, gripper_width_m=0.07)
            return feedback

    arm = TransientFeedback(settings)
    cycle = PickRound(settings, environment, lambda: arm, Event())
    result = cycle.run("transient-holding-feedback")
    assert arm.injected
    assert result.status == "completed", result
    assert arm.targets[-1].position == settings.profile.post_grasp_position_arm_m


def test_pick_tolerates_one_bad_sample_at_end_of_hold(settings):
    settings = settings.model_copy(
        update={"profile": settings.profile.model_copy(update={"hold_duration_s": 0})}
    )
    environment = FakePickEnvironment(settings)

    class TransientFeedback(FakePickArm):
        injected = False

        def feedback(self):
            feedback = super().feedback()
            if cycle.stage == "hold_object" and not self.injected:
                self.injected = True
                return replace(feedback, gripper_width_m=0.07)
            return feedback

    arm = TransientFeedback(settings)
    cycle = PickRound(settings, environment, lambda: arm, Event())
    result = cycle.run("transient-end-of-hold")
    assert arm.injected
    assert result.status == "completed", result


def test_pick_persistent_grasp_loss_reports_diagnostics(settings, monkeypatch):
    environment = FakePickEnvironment(settings)
    events = record_events(monkeypatch, module)

    class Lost(FakePickArm):
        def move(self, target):
            super().move(target)
            if cycle.stage == "move_to_post_grasp":
                self.current = replace(self.current, gripper_width_m=0.07)

    arm = Lost(settings)
    cycle = PickRound(settings, environment, lambda: arm, Event())
    result = cycle.run("persistent-loss")
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted"
    assert "lost_for_s=" in result.message and "'width_ok': False" in result.message
    samples = recorded(events, "grasp_feedback")
    assert samples[-1]["stage"] == "move_to_post_grasp"
    assert samples[-1]["width_ok"] is False
    assert samples[-1]["lost_for_s"] >= settings.limits.stable_time_s


@pytest.mark.parametrize(
    "feedback_update",
    [
        {"gripper_effort": 0.0},
        {"gripper_velocity_m_s": 0.1},
    ],
)
def test_pick_grasp_uses_stable_width_not_effort_or_velocity(settings, feedback_update):
    environment = FakePickEnvironment(settings)

    class WidthOnlyGrasp(FakePickArm):
        def gripper(self, width_m, effort):
            super().gripper(width_m, effort)
            if self.current.gripper_width_m >= settings.profile.grasp_min_width_m:
                self.current = replace(self.current, **feedback_update)

    result = PickRound(settings, environment, lambda: WidthOnlyGrasp(settings), Event()).run(
        "width-only-grasp"
    )
    assert result.status == "completed", result


@pytest.mark.parametrize("dropped_target", ["open", "close"])
def test_pick_refreshes_gripper_target_when_first_command_is_ignored(settings, dropped_target):
    settings = settings.model_copy(
        update={"profile": settings.profile.model_copy(update={"hold_duration_s": 0})}
    )
    profile = settings.profile
    dropped_width = profile.gripper_open_m if dropped_target == "open" else profile.gripper_closed_m
    commands = []

    class DroppedCommand(FakePickArm):
        def gripper(self, width_m, effort):
            first = width_m not in commands
            commands.append(width_m)
            if first and width_m == dropped_width:
                return
            super().gripper(width_m, effort)

    arm = DroppedCommand(settings)
    # Start closed, as on a second run after the previous round held an object.
    arm.current = replace(arm.current, gripper_width_m=profile.gripper_closed_m)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event())
    result = cycle.run("dropped-gripper-command")
    assert result.status == "completed", result.message
    assert result.cleanup_confirmed
    assert commands.count(dropped_width) >= 2
    first_close = commands.index(profile.gripper_closed_m)
    first_release = commands.index(profile.gripper_open_m, first_close)
    assert set(commands[first_close:first_release]) == {profile.gripper_closed_m}
    assert set(commands[first_release:]) == {profile.gripper_open_m}
    assert cycle.plan.grasp in arm.targets


def test_pick_open_timeout_reports_width_and_does_not_advance(settings, monkeypatch):
    settings = settings.model_copy(
        update={"limits": settings.limits.model_copy(update={"stage_timeout_s": 0.25})}
    )
    events = record_events(monkeypatch, module)
    commands = []

    class StuckGripper(FakePickArm):
        def gripper(self, width_m, effort):
            commands.append(width_m)

    arm = StuckGripper(settings)
    arm.current = replace(arm.current, gripper_width_m=0.03)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event())
    result = cycle.run("open-timeout")
    assert result.status == "failed" and result.stage == "retry_open_gripper"
    assert arm.current.pose.position == settings.profile.post_grasp_position_arm_m
    assert "stage timed out: wait_open_gripper; gripper_feedback=" in result.message
    samples = recorded(events, "gripper_feedback")
    assert samples[-1]["width_m"] == pytest.approx(0.03)
    assert samples[-1]["target_width_m"] == pytest.approx(0.08)
    assert samples[-1]["width_error_m"] == pytest.approx(0.05)
    assert samples[-1]["width_ok"] is False
    assert len(commands) >= 2 and set(commands) == {0.08}
    assert cycle.plan.grasp not in arm.targets
    assert arm.calls[-3:] == ["stop", "release", "close"]
    assert result.cleanup_confirmed


def test_pick_grasp_timeout_reports_width_feedback(settings, monkeypatch):
    limits = settings.limits.model_copy(update={"stage_timeout_s": 0.25})
    settings = settings.model_copy(update={"limits": limits})
    environment = FakePickEnvironment(settings)
    events = record_events(monkeypatch, module)

    class UnconfirmedGrasp(FakePickArm):
        def gripper(self, width_m, effort):
            super().gripper(width_m, effort)
            if width_m <= settings.profile.gripper_closed_m:
                # Jaws never move: the measured width stays at the open target, so
                # the live contact gate (width < open - margin) must keep failing.
                self.current = replace(
                    self.current, gripper_width_m=settings.profile.gripper_open_m
                )

    arm = UnconfirmedGrasp(settings)
    result = PickRound(settings, environment, lambda: arm, Event()).run("grasp-timeout")
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted"
    assert "stage timed out: close_gripper; grasp_feedback=" in result.message
    samples = recorded(events, "grasp_feedback")
    assert len(samples) == 3
    assert samples[-1]["width_ok"] is False
    assert samples[-1]["width_m"] == pytest.approx(settings.profile.gripper_open_m)
    assert samples[-1]["width_span_m"] == pytest.approx(0)
    assert arm.calls[-3:] == ["stop", "release", "close"]


def test_pick_confirmation_event_reports_measured_contact_width(settings, monkeypatch):
    """The close stage publishes one active grasp_feedback with the contact width.

    L6 ``judge_scripted`` reads the last ``grasp_feedback`` width/width_ok; the
    historical run only recorded the closing-start (open) sample and judged a held
    object as task_success=false.
    """
    events = record_events(monkeypatch, module)
    arm = FakePickArm(settings)
    result = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event()).run(
        "confirmation-event"
    )
    assert result.status == "completed", result
    contact_width = arm.object_width_m
    samples = recorded(events, "grasp_feedback")
    confirmation = samples[-1]
    assert confirmation["stage"] == "close_gripper"
    assert confirmation["width_m"] == pytest.approx(contact_width)
    assert confirmation["width_ok"] is True
    assert confirmation["width_in_expected_interval"] is True
    assert confirmation["contact_width_m"] == pytest.approx(contact_width)
    assert confirmation["contact_threshold_m"] == pytest.approx(
        settings.profile.gripper_closed_m + settings.profile.grasp_contact_margin_m
    )
    assert confirmation["contact_ceiling_m"] == pytest.approx(
        settings.profile.gripper_open_m - settings.profile.grasp_contact_margin_m
    )
    assert confirmation["required_stable_s"] == pytest.approx(settings.limits.stable_time_s)
    assert confirmation["stable_for_s"] >= settings.limits.stable_time_s
    assert confirmation["elapsed_s"] >= confirmation["stable_for_s"]
    # Inside the calibrated cube interval: no diagnostic deviation event.
    assert not recorded(events, "grasp_width_outside_expected")


def test_pick_real_measured_width_00782_is_confirmed_and_kept(settings, monkeypatch):
    """2026-11-12 field regression: 7.82 mm measured grip is a real grasp.

    The fixed 1.5 cm cube interval [0.010, 0.022] used to fail this round (then the
    recovery lifted and opened at home, dropping the object).  The live measured
    width now confirms the grasp, continues the round and the final grasp_feedback
    carries width_ok=True with the measured width so L6 judging agrees.
    """
    profile = settings.profile.model_copy(
        update={
            "gripper_closed_m": 0.005,
            "grasp_min_width_m": 0.010,
            "grasp_max_width_m": 0.022,
            "cube_side_m": 0.015,
            "gripper_open_m": 0.08,
        }
    )
    settings = settings.model_copy(update={"profile": profile})
    events = record_events(monkeypatch, module)
    arm = FakePickArm(settings, object_width_m=0.00782)
    result = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event()).run(
        "field-width-00782"
    )
    assert result.status == "completed", result
    assert result.stage == "home_end"
    assert arm.calls.count("gripper") >= 2
    samples = recorded(events, "grasp_feedback")
    confirmation = samples[-1]
    assert confirmation["stage"] == "close_gripper"
    assert confirmation["width_m"] == pytest.approx(0.00782)
    assert confirmation["width_ok"] is True
    assert confirmation["width_in_expected_interval"] is False
    outside = recorded(events, "grasp_width_outside_expected")
    assert len(outside) == 1
    assert outside[0]["width_m"] == pytest.approx(0.00782)
    # Projected cube interval: the tiny cube tilt adds ~0.03 mm to both bounds.
    assert outside[0]["min_width_m"] == pytest.approx(0.010, abs=1e-3)
    assert outside[0]["max_width_m"] == pytest.approx(0.022, abs=1e-3)
    assert outside[0]["width_in_expected_interval"] is False


def test_pick_accepts_thin_contact_that_used_to_be_rejected(settings, monkeypatch):
    """0.033 m is outside object_grasp [0.034, 0.055] but is a physical grasp."""
    events = record_events(monkeypatch, module)
    arm = FakePickArm(settings, object_width_m=0.033)
    result = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event()).run(
        "thin-contact"
    )
    assert result.status == "completed", result
    assert arm.calls.count("gripper") >= 2
    samples = recorded(events, "grasp_feedback")
    assert samples[-1]["width_m"] == pytest.approx(0.033)
    assert samples[-1]["width_ok"] is True
    assert len(recorded(events, "grasp_width_outside_expected")) == 1


def test_pick_rejects_contact_that_keeps_closing_inside_width_window(settings, monkeypatch):
    limits = settings.limits.model_copy(update={"stage_timeout_s": 0.25})
    settings = settings.model_copy(update={"limits": limits})
    events = record_events(monkeypatch, module)

    class MovingContact(FakePickArm):
        sample = False

        def feedback(self):
            feedback = super().feedback()
            if cycle.stage == "close_gripper":
                self.sample = not self.sample
                return replace(feedback, gripper_width_m=0.048 if self.sample else 0.052)
            return feedback

    arm = MovingContact(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event())
    result = cycle.run("moving-contact")
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted"
    samples = recorded(events, "grasp_feedback")
    assert samples[-1]["width_ok"] is True
    assert samples[-1]["stable_for_s"] == pytest.approx(0)


def test_pick_rejects_non_finite_joint_command(settings):
    arm = FakePickArm(settings)
    arm.acquire()
    with pytest.raises(ContractError, match="finite Arm joint positions"):
        arm.move_joints((0.0,) * 5)
    with pytest.raises(ContractError, match="finite Arm joint positions"):
        arm.move_joints((0.0, 0.0, 0.0, 0.0, 0.0, float("nan")))
    assert isinstance(settings, PickSettings)
