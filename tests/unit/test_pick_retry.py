"""Purpose: Verify bounded grasp confirmation, topic recovery and fresh retry plans.

Usage: .venv/bin/python -m pytest tests/unit/test_pick_retry.py
"""

from dataclasses import replace
from pathlib import Path
from threading import Event
from types import SimpleNamespace

import numpy as np
import pytest

from rule_base.config.load import load_settings
from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import (
    ArmFeedbackError,
    ArmSessionError,
    ContractError,
    PickScene,
    Pose,
)
from rule_base.infrastructure.fakes.runtime import FakePickArm, FakePickEnvironment, tf_entry
from rule_base.infrastructure.transform.math import compose, from_pose, matrix, offset, to_pose
from rule_base.infrastructure.transform.observations import PickObservationCache
from rule_base.stations.pick_place import execution
from rule_base.stations.pick_place.execution import PickRound
from rule_base.stations.pick_place.planning import plan_pick

CONFIG = Path(__file__).resolve().parents[2] / "configs/pick_place/offline.yaml"


@pytest.fixture
def rig(monkeypatch):
    settings = load_settings(CONFIG)
    assert isinstance(settings, PickSettings)
    settings = settings.model_copy(
        update={
            "profile": settings.profile.model_copy(update={"hold_duration_s": 0}),
            "limits": settings.limits.model_copy(
                update={"stable_time_s": 0.5, "warmup_timeout_s": 0.2}
            ),
        }
    )
    state = SimpleNamespace(
        now=1.0, events=[], fail_attempts={1}, wait_for_tf=False, gripper_commands=[]
    )

    class Stop(Event):
        def wait(self, timeout=None):
            state.now += timeout or 0
            return self.is_set()

    monkeypatch.setattr(execution, "time", SimpleNamespace(monotonic=lambda: state.now))
    monkeypatch.setattr(
        execution, "event", lambda name, **fields: state.events.append((name, fields, state.now))
    )
    scene = FakePickEnvironment(settings).snapshot()
    changed = np.asarray(scene.arm_to_object).copy()
    yaw = from_pose(Pose((0, 0, 0), (0, 0, np.sin(0.1), np.cos(0.1))))
    # Change world yaw with an unchanged command base and shift object center.
    base_rotation = np.asarray(scene.root_to_arm)[:3, :3]
    changed[:3, :3] = base_rotation.T @ np.asarray(yaw)[:3, :3] @ base_rotation @ changed[:3, :3]
    changed[1, 3] += 0.025
    state.scenes = [scene, PickScene(scene.root_to_arm, matrix(changed))]
    state.requests = 0
    state.snapshots = 0
    state.home = Pose(
        settings.profile.post_grasp_position_arm_m, settings.profile.post_grasp_orientation_arm_xyzw
    )

    class Environment:
        def snapshot(self):
            state.snapshots += 1
            if state.requests and state.wait_for_tf:
                raise ContractError("new object TF unavailable")
            return state.scenes[min(state.requests, 1)]

        def request_new_observation(self):
            assert state.arm.current.pose == state.home
            state.requests += 1

        def check_health(self):
            pass

    class Arm(FakePickArm):
        def __init__(self):
            super().__init__(settings)
            self.sent = []

        def move(self, target):
            self.sent.append((state.now, state.cycle.stage, target))
            super().move(target)

        def gripper(self, width_m, effort):
            state.gripper_commands.append((state.cycle.stage, self.current.pose, width_m))
            self.object_present = state.cycle.attempt not in state.fail_attempts
            super().gripper(width_m, effort)

    state.arm = Arm()
    state.stop = Stop()
    state.environment = Environment()
    state.settings = settings
    state.cycle = PickRound(settings, state.environment, lambda: state.arm, state.stop)
    return state


def test_failed_grasp_returns_by_pose_topic_and_replans_in_one_servo_session(rig):
    initial = plan_pick(rig.settings, rig.scenes[0])
    result = rig.cycle.run("retry-success")
    assert result.status == "completed", result
    assert rig.cycle.attempt == 2 and rig.requests == 1 and rig.snapshots == 2
    assert rig.arm.calls.count("connect") == rig.arm.calls.count("acquire") == 1
    assert rig.arm.calls.count("start_servo") == 1
    assert rig.arm.calls[-3:] == ["stop", "release", "close"]
    assert initial.grasp in rig.arm.targets and rig.cycle.plan.grasp in rig.arm.targets
    assert initial.grasp.position != pytest.approx(rig.cycle.plan.grasp.position)
    assert initial.grasp.orientation != pytest.approx(rig.cycle.plan.grasp.orientation)
    assert rig.arm.current.pose == rig.home
    stages = [fields["stage"] for name, fields, _ in rig.events if name == "stage"]
    assert stages.index("retry_retreat") < stages.index("retry_return_initial")
    assert stages.index("retry_return_initial") < stages.index("retry_open_gripper")
    assert stages.index("retry_open_gripper") < stages.index("reidentify")
    recovery_opens = [
        pose for stage, pose, width in rig.gripper_commands if stage == "retry_open_gripper"
    ]
    assert recovery_opens and all(pose == rig.home for pose in recovery_opens)
    # Every recovery waypoint uses the measured pose and world +Z despite a rotated base.
    retreat = [target for _, stage, target in rig.arm.sent if stage == "retry_retreat"]
    world_start = to_pose(compose(rig.scenes[0].root_to_arm, from_pose(initial.grasp)))
    world_retreat = [to_pose(compose(rig.scenes[0].root_to_arm, from_pose(p))) for p in retreat]
    assert all(p.position[:2] == pytest.approx(world_start.position[:2]) for p in world_retreat)
    assert world_retreat[-1].position[2] - world_start.position[2] == pytest.approx(0.1)
    assert all(p.orientation == pytest.approx(initial.grasp.orientation) for p in retreat)
    assert (
        max(b[0] - a[0] for a, b in zip(rig.arm.sent, rig.arm.sent[1:], strict=False)) <= 0.010001
    )


def test_three_failed_attempts_also_return_home_after_the_final_failure(rig):
    rig.fail_attempts = {1, 2, 3}
    result = rig.cycle.run("exhausted")
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted"
    assert "after 3 attempts" in result.message
    assert rig.cycle.attempt == 3 and rig.requests == 2
    stages = [f["stage"] for n, f, _ in rig.events if n == "stage"]
    assert stages.count("close_gripper") == stages.count("retry_return_initial") == 3
    assert rig.arm.current.pose == rig.home
    assert rig.arm.current.gripper_width_m == rig.settings.profile.gripper_open_m
    assert rig.arm.calls[-3:] == ["stop", "release", "close"]
    assert result.cleanup_confirmed


def test_reidentification_timeout_keeps_refreshing_home_and_never_reuses_old_plan(rig):
    rig.wait_for_tf = True
    result = rig.cycle.run("no-new-tf")
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted"
    assert "new Pick observation" in result.message
    assert rig.cycle.attempt == 3 and rig.requests == 2
    assert rig.cycle.plan is None
    started = next(
        at for name, fields, at in rig.events if name == "stage" and fields["stage"] == "reidentify"
    )
    refreshes = [(at, p) for at, _, p in rig.arm.sent if at >= started]
    assert len(refreshes) >= 20 and all(p == rig.home for _, p in refreshes)
    assert max(b[0] - a[0] for a, b in zip(refreshes, refreshes[1:], strict=False)) <= 0.010001


def test_invalid_new_geometry_waits_for_valid_observation_without_reusing_plan(rig):
    scene = rig.scenes[1]
    rig.scenes[1] = PickScene(
        scene.root_to_arm, from_pose(offset(to_pose(scene.arm_to_object), (2, 0, 0)))
    )
    result = rig.cycle.run("bad-new-geometry")
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted"
    assert "workspace" in result.message and "timed out" in result.message
    assert rig.cycle.attempt == 3 and rig.snapshots > 3 and rig.cycle.plan is None
    assert rig.arm.current.pose == rig.home


@pytest.mark.parametrize("stage", ["retry_retreat", "retry_return_initial", "retry_open_gripper"])
def test_fault_inside_recovery_is_bounded_and_does_not_start_another_grasp(rig, stage):
    original = rig.arm.feedback

    def feedback():
        if rig.cycle.stage == stage:
            raise ContractError("feedback stale or control lost")
        return original()

    rig.arm.feedback = feedback
    result = rig.cycle.run("feedback-fault")
    assert result.status == "failed" and result.stage == stage
    assert "feedback stale" in result.message
    assert rig.cycle.attempt == 1
    assert rig.requests == 0
    assert "recovery failed" in result.message
    assert rig.arm.calls[-3:] == ["stop", "release", "close"]


def test_transport_fault_during_reidentification_is_not_masked_as_tf_timeout(rig):
    def unhealthy():
        raise ContractError("TF transport decode failed")

    rig.environment.check_health = unhealthy
    result = rig.cycle.run("transport-fault")
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted"
    assert "TF transport decode failed" in result.message
    assert rig.snapshots == 1 and rig.cycle.attempt == 3
    assert rig.arm.current.pose == rig.home


@pytest.mark.parametrize("stage", ["retry_retreat", "reidentify"])
def test_cancellation_during_recovery_and_observation_wait_stops_further_motion(rig, stage):
    rig.wait_for_tf = True
    wait = rig.stop.wait

    def cancel(timeout=None):
        if rig.cycle.stage == stage:
            rig.stop.set()
        return wait(timeout)

    rig.stop.wait = cancel
    result = rig.cycle.run("cancel-retry")
    assert result.status == "interrupted" and result.stage == stage
    assert rig.cycle.attempt == (2 if stage == "reidentify" else 1)
    assert rig.arm.calls[-3:] == ["stop", "release", "close"]


def test_confirmation_uses_1_5_seconds_including_command_time_and_width_stability(rig):
    command = rig.arm.gripper
    command_delay = 0.04
    closes = set()

    def gripper(width_m, effort):
        if width_m == rig.settings.profile.gripper_closed_m and rig.cycle.attempt not in closes:
            closes.add(rig.cycle.attempt)
            rig.now += command_delay
        command(width_m, effort)

    rig.arm.gripper = gripper
    result = rig.cycle.run("timed-confirmation")
    assert result.status == "completed", result
    stages = [
        (fields["stage"], fields["attempt"], at)
        for name, fields, at in rig.events
        if name == "stage"
    ]
    started = next(at for stage, attempt, at in stages if stage == "close_gripper" and attempt == 1)
    failed = next(at for name, _, at in rig.events if name == "grasp_attempt_failed")
    assert failed - started == pytest.approx(1.5, abs=0.010001)
    started = next(at for stage, attempt, at in stages if stage == "close_gripper" and attempt == 2)
    moved = next(at for stage, _, at in stages if stage == "move_to_post_grasp")
    assert moved - started >= command_delay + 0.5


@pytest.mark.parametrize(
    "stage",
    [
        "open_gripper",
        "pregrasp",
        "wait_open_gripper",
        "grasp_pose",
        "close_gripper",
        "move_to_post_grasp",
        "hold_object",
        "release_gripper",
    ],
)
def test_failure_at_each_action_stage_returns_opens_and_retries_entire_cycle(rig, stage):
    rig.fail_attempts = set()
    if stage == "hold_object":
        rig.cycle.settings = rig.settings.model_copy(
            update={"profile": rig.settings.profile.model_copy(update={"hold_duration_s": 0.05})}
        )
    check = rig.cycle._check
    injected = False

    def fail_once(**kwargs):
        nonlocal injected
        if rig.cycle.stage == stage and rig.cycle.attempt == 1 and not injected:
            injected = True
            raise ContractError("injected action failure")
        return check(**kwargs)

    rig.cycle._check = fail_once
    result = rig.cycle.run("whole-cycle-retry")
    assert injected and result.status == "completed", result
    assert rig.cycle.attempt == 2 and rig.requests == 1
    failure = [fields for name, fields, _ in rig.events if name == "grasp_attempt_failed"]
    assert failure[0]["failed_stage"] == stage
    assert rig.arm.calls.count("start_servo") == 1
    assert all(
        pose == rig.home for stage, pose, _ in rig.gripper_commands if stage == "retry_open_gripper"
    )


def test_real_motion_timeout_recovers_before_another_attempt(rig):
    rig.fail_attempts = set()
    move = rig.arm.move

    def lagging_arm(target):
        actual = rig.arm.current.pose
        move(target)
        if rig.cycle.attempt == 1 and rig.cycle.stage == "pregrasp":
            rig.arm.current = replace(rig.arm.current, pose=actual)

    rig.arm.move = lagging_arm
    result = rig.cycle.run("pregrasp-timeout")
    assert result.status == "completed", result
    failures = [fields for name, fields, _ in rig.events if name == "grasp_attempt_failed"]
    assert failures[0]["failed_stage"] == "pregrasp"
    assert "stage timed out: pregrasp" in failures[0]["error"]
    assert rig.cycle.attempt == 2 and rig.requests == 1


@pytest.mark.parametrize("failure", [ArmFeedbackError, ArmSessionError])
def test_feedback_or_session_failure_restarts_at_measured_tcp_without_old_waypoint(rig, failure):
    rig.fail_attempts = set()
    feedback = rig.arm.feedback
    injected = False

    def interrupt_descent():
        nonlocal injected
        if rig.cycle.stage == "grasp_pose" and not injected:
            injected = True
            raise failure("session interrupted")
        return feedback()

    rig.arm.feedback = interrupt_descent
    result = rig.cycle.run("restore-servo")
    assert result.status == "completed", result
    assert rig.arm.calls.count("recover") == 1
    assert rig.arm.calls.count("acquire") == rig.arm.calls.count("start_servo") == 2
    restore = [target for _, stage, target in rig.arm.sent if stage == "retry_restore_session"]
    assert restore and all(
        pose == plan_pick(rig.settings, rig.scenes[0]).pregrasp for pose in restore
    )
    assert rig.cycle.attempt == 2 and rig.requests == 1


def test_potentially_held_object_is_not_opened_until_return_pose_is_confirmed(rig):
    rig.fail_attempts = set()
    feedback = rig.arm.feedback

    def lose_contact():
        sample = feedback()
        if rig.cycle.attempt == 1 and rig.cycle.stage == "move_to_post_grasp":
            return replace(sample, gripper_width_m=0.07)
        return sample

    rig.arm.feedback = lose_contact
    result = rig.cycle.run("held-return")
    assert result.status == "completed", result
    recovery = [
        (stage, pose, width)
        for stage, pose, width in rig.gripper_commands
        if stage.startswith("retry_")
    ]
    assert recovery
    for stage, pose, width in recovery:
        if width == rig.settings.profile.gripper_open_m:
            assert stage == "retry_open_gripper" and pose == rig.home
        else:
            assert width == rig.settings.profile.gripper_closed_m


def test_unrecoverable_session_never_moves_home_or_opens(rig):
    rig.fail_attempts = set()
    feedback = rig.arm.feedback

    def stale():
        if rig.cycle.stage == "grasp_pose":
            raise ArmFeedbackError("missing pose feedback")
        return feedback()

    def recovery_failed(*_):
        raise ContractError("lease id does not match the active lease")

    rig.arm.feedback = stale
    rig.arm.recover = recovery_failed
    result = rig.cycle.run("no-authority")
    assert result.status == "failed" and result.stage == "retry_restore_session"
    assert "recovery failed" in result.message and "lease id does not match" in result.message
    assert not any(
        stage in {"retry_return_initial", "retry_open_gripper"} for _, stage, _ in rig.arm.sent
    )
    assert rig.cycle.attempt == 1 and rig.requests == 0


def test_missed_servo_deadline_reinitializes_instead_of_resending_old_target(rig):
    rig.fail_attempts = set()
    feedback = rig.arm.feedback
    injected = False

    def slow_feedback():
        nonlocal injected
        if rig.cycle.stage == "grasp_pose" and not injected:
            rig.now += rig.settings.limits.servo_max_gap_s + 0.01
            injected = True
        return feedback()

    rig.arm.feedback = slow_feedback
    result = rig.cycle.run("gap-recovery")
    assert result.status == "completed", result
    assert rig.arm.calls.count("recover") == 1 and rig.cycle.attempt == 2


def test_final_cleanup_failure_does_not_repeat_successful_release(rig):
    rig.fail_attempts = set()

    def release_failed():
        raise ContractError("lease mismatch during cleanup")

    rig.arm.release = release_failed
    result = rig.cycle.run("cleanup-only")
    assert result.status == "failed" and not result.cleanup_confirmed
    # The normal completion path now ends with the per-round home (D-094).
    assert result.stage == "home_end" and rig.cycle.attempt == 1
    assert rig.requests == 0 and rig.arm.current.pose == rig.home
    assert rig.arm.current.gripper_width_m == rig.settings.profile.gripper_open_m


def test_retry_waits_through_temporarily_invalid_geometry_then_uses_valid_scene(rig):
    snapshot = rig.environment.snapshot
    rejected = 0

    def scene_becomes_valid():
        nonlocal rejected
        if rig.requests and rejected < 3:
            rejected += 1
            scene = rig.scenes[1]
            return PickScene(
                scene.root_to_arm, from_pose(offset(to_pose(scene.arm_to_object), (2, 0, 0)))
            )
        return snapshot()

    rig.environment.snapshot = scene_becomes_valid
    result = rig.cycle.run("new-valid-geometry")
    assert result.status == "completed", result
    assert rejected == 3 and rig.cycle.attempt == 2


def test_retreat_shortens_at_workspace_boundary_without_skipping_home(rig):
    rig.fail_attempts = set()
    rig.cycle.settings = rig.settings.model_copy(
        update={
            "profile": rig.settings.profile.model_copy(
                update={"workspace_min_m": (0.3, -0.3, 0.05)}
            ),
            "limits": rig.settings.limits.model_copy(update={"max_grasp_attempts": 1}),
        }
    )
    feedback = rig.arm.feedback
    injected = False

    def fail_before_descent():
        nonlocal injected
        if rig.cycle.stage == "grasp_pose" and not injected:
            injected = True
            raise ContractError("descent failed")
        return feedback()

    rig.arm.feedback = fail_before_descent
    result = rig.cycle.run("limited-retreat")
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted", result
    retreat = [pose for _, stage, pose in rig.arm.sent if stage == "retry_retreat"]
    assert retreat and min(p.position[0] for p in retreat) >= 0.3 - 1e-12
    assert retreat[-1].position[0] == pytest.approx(0.3)
    assert rig.arm.current.pose == rig.home
    assert rig.arm.current.gripper_width_m == rig.settings.profile.gripper_open_m


def test_width_drift_resets_half_second_confirmation_window(rig):
    rig.fail_attempts = set()
    feedback = rig.arm.feedback
    close_start = None

    def drifting_feedback():
        nonlocal close_start
        sample = feedback()
        if rig.cycle.stage == "close_gripper":
            if close_start is None:
                close_start = rig.now
            elapsed = rig.now - close_start
            # Every 0.4 s the width changes by 0.6 mm, despite remaining in range.
            return replace(sample, gripper_width_m=0.045 + 0.0006 * int(elapsed / 0.4))
        return sample

    rig.arm.feedback = drifting_feedback
    rig.settings = rig.settings.model_copy(
        update={"limits": rig.settings.limits.model_copy(update={"max_grasp_attempts": 1})}
    )
    rig.cycle.settings = rig.settings
    result = rig.cycle.run("drifting-width")
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted"
    assert "close_gripper" in result.message


def test_retry_cache_requires_new_object_sample_not_just_new_platform_tf(monkeypatch):
    settings = load_settings(CONFIG)
    cache = PickObservationCache(settings)
    f = settings.frames
    now = 10_000_000_000
    monkeypatch.setattr("rule_base.infrastructure.transform.buffer.time.time_ns", lambda: now)
    cache.update(
        "tf_static", {"transforms": [tf_entry(f.root, "camera", (0, 0, 0), timestamp_ns=0)]}
    )
    old = tf_entry("camera", f.object, (0.4, 0, 0.3), timestamp_ns=now)
    cache.update(
        "tf", {"transforms": [old, tf_entry(f.root, f.arm_base, (0, 0, 0), timestamp_ns=now)]}
    )
    cache.snapshot()
    now += 1_000_000
    cache.request_new_observation()
    cache.update(
        "tf", {"transforms": [old, tf_entry(f.root, f.arm_base, (0.1, 0, 0), timestamp_ns=now)]}
    )
    with pytest.raises(ContractError, match="requires new object TF"):
        cache.snapshot()
    now += 1_000_000
    cache.update(
        "tf", {"transforms": [tf_entry("camera", f.object, (0.45, 0, 0.3), timestamp_ns=now)]}
    )
    assert to_pose(cache.snapshot().arm_to_object).position == pytest.approx((0.35, 0, 0.3))
    # A delayed packet cannot overwrite the fresh measurement.
    cache.update("tf", {"transforms": [old]})
    assert to_pose(cache.snapshot().arm_to_object).position == pytest.approx((0.35, 0, 0.3))


@pytest.mark.parametrize(
    "updates",
    [
        {"grasp_confirmation_timeout_s": 0.5},
        {"max_grasp_attempts": 0},
        {"max_grasp_attempts": 4},
        {"recovery_feedback_timeout_s": 0.2},
        {"recovery_feedback_stable_time_s": 0},
    ],
)
def test_rejects_invalid_retry_configuration(updates):
    values = load_settings(CONFIG).model_dump()
    values["limits"].update(stable_time_s=0.5, **updates)
    with pytest.raises(ValueError):
        PickSettings.model_validate(values)
