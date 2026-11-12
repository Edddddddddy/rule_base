"""Purpose: Guard Pick planning, core Servo sequence, holds and legacy behavior.

Usage: source ros2_ws/setup_ros_env.sh && python -m pytest tests/unit/test_pick_place.py
"""

import asyncio
import time
from dataclasses import replace
from threading import Event
from types import SimpleNamespace

import numpy as np
import pytest

from pick_place_support import (
    CONFIG,
    offline_settings,
    record_events,
    recorded,
    stages,
    upright_environment,
)
from rule_base.config.load import load_settings
from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import ContractError, PickScene, Pose
from rule_base.infrastructure.fakes.runtime import FakePickArm, FakePickEnvironment, tf_entry
from rule_base.infrastructure.transform.math import compose, errors, from_pose, offset, to_pose
from rule_base.infrastructure.transform.observations import PickObservationCache
from rule_base.interfaces.node import serve
from rule_base.stations.pick_place import execution as module
from rule_base.stations.pick_place.execution import PickRound
from rule_base.stations.pick_place.planning import plan_pick


@pytest.fixture
def settings():
    return offline_settings()


def test_pick_translated_object_changes_target_without_fixed_pose_gate(settings):
    baseline = FakePickEnvironment(settings).snapshot()
    moved = PickScene(
        baseline.root_to_arm, from_pose(offset(to_pose(baseline.arm_to_object), (0.02, 0, 0)))
    )
    before, after = plan_pick(settings, baseline), plan_pick(settings, moved)
    assert after.grasp.position == pytest.approx(offset(before.grasp, (0.02, 0, 0)).position)


def test_live_pick_resolves_platform_and_camera_chain_with_unprefixed_arm_base():
    settings = load_settings(CONFIG.with_name("live.yaml"))
    assert isinstance(settings, PickSettings)
    f = settings.frames
    assert f.arm_base == "base_link"
    cache = PickObservationCache(settings)
    camera = "platform/cam0_optical_frame"
    base = "platform/base_link"
    end = "arm/end_pose_link"
    root_camera = tf_entry(f.root, camera, (0.6, 0, 0.4), timestamp_ns=0)
    root_camera["transform"]["rotation"] = dict(zip("xyzw", (2**-0.5, 0, 0, 2**-0.5), strict=True))
    cache.update(
        "tf_static",
        {
            "transforms": [
                tf_entry(f.root, base, (0.1, 0, 0), timestamp_ns=0),
                tf_entry("arm/Link5", end, (0, 0, 0), timestamp_ns=0),
                tf_entry(end, f.arm_base, (0, 0, 0), timestamp_ns=0),
                root_camera,
                tf_entry(camera, "objects_cube_01", (0, 0, 0.1), timestamp_ns=0),
                tf_entry(camera, "objects_cube_02", (10, 10, 10), timestamp_ns=0),
            ]
        },
    )
    parents = [base, *(f"arm/Link{i}" for i in range(1, 6))]
    cache.update(
        "tf",
        {
            "transforms": [
                tf_entry(parent, child, (0, 0, 0), timestamp_ns=time.time_ns())
                for parent, child in zip(parents[:-1], parents[1:], strict=True)
            ]
        },
    )
    plan = plan_pick(settings, cache.snapshot())
    assert plan.grasp.position == pytest.approx((0.5, -0.1, 0.44))
    rotation = np.asarray(from_pose(plan.grasp))[:3, :3]
    assert rotation[:, 0] == pytest.approx((0, 0, -1))
    assert rotation[:, 1] == pytest.approx((0, 1, 0))
    assert plan.pregrasp.position == pytest.approx((0.5, -0.1, 0.54))


@pytest.mark.parametrize(
    "offset_ns,reason",
    [
        (-1_500_000_000, None),
        (-1_500_000_001, "stale"),
        (0, None),
        (6_800_000, None),
        (20_000_000, None),
        (20_000_001, "future"),
        (200_000_000, "future"),
    ],
)
def test_live_pick_tf_tolerance_accepts_small_clock_offset_only(monkeypatch, offset_ns, reason):
    config = load_settings(CONFIG.with_name("live.yaml"))
    assert isinstance(config, PickSettings)
    cache = PickObservationCache(config)
    f = config.frames
    now = 10_000_000_000
    monkeypatch.setattr("rule_base.infrastructure.transform.buffer.time.time_ns", lambda: now)
    cache.update(
        "tf",
        {
            "transforms": [
                tf_entry(f.root, f.arm_base, (0, 0, 0), timestamp_ns=now + offset_ns),
                tf_entry(f.root, f.object, (0.3, 0, 0.4), timestamp_ns=now + offset_ns),
            ]
        },
    )
    if reason is None:
        assert cache.snapshot().arm_to_object[0][3] == pytest.approx(0.3)
    else:
        with pytest.raises(ContractError, match=f"reason={reason}"):
            cache.snapshot()


def test_pick_motion_grasp_hold_and_release(settings):
    environment = FakePickEnvironment(settings)
    arm = FakePickArm(settings)
    cycle = PickRound(settings, environment, lambda: arm, Event())
    result = cycle.run("pick-1")
    assert result.status == "completed", result
    assert result.stage == "home_end" and result.cleanup_confirmed
    assert arm.calls.count("start_servo") == 1
    targets = arm.targets
    # start_servo(hold_pose) may record the measured pose first; the observation
    # waypoint is the first planned target and precedes pregrasp/grasp.
    assert cycle.plan.observation in targets
    assert targets.index(cycle.plan.observation) < targets.index(cycle.plan.pregrasp)
    approach = targets.index(cycle.plan.pregrasp)
    grasp = targets.index(cycle.plan.grasp)
    assert 0 < approach < grasp
    target = Pose(
        settings.profile.post_grasp_position_arm_m,
        settings.profile.post_grasp_orientation_arm_xyzw,
    )
    assert target in targets[grasp + 1 :]
    assert targets[-1] == target
    assert arm.current.pose == target
    assert arm.current.gripper_width_m == settings.profile.gripper_open_m
    assert len(targets[grasp + 1 :]) > 10
    assert arm.calls[-3:] == ["stop", "release", "close"]
    assert not arm.owned


def test_pick_holds_at_configured_post_grasp_pose_before_releasing(settings, monkeypatch):
    now = 1.0
    stage_times = []
    settings = settings.model_copy(
        update={"profile": settings.profile.model_copy(update={"hold_duration_s": 5})}
    )

    class AdvancingStop(Event):
        def wait(self, timeout=None):
            nonlocal now
            now += timeout or 0
            return self.is_set()

    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: now))
    def record(name, **fields):
        if name == "stage":
            stage_times.append((fields["stage"], now))

    monkeypatch.setattr(module, "event", record)
    arm = FakePickArm(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, AdvancingStop())
    result = cycle.run("post-grasp-hold")
    hold_started = next(at for stage, at in stage_times if stage == "hold_object")
    released_at = next(at for stage, at in stage_times if stage == "release_gripper")
    assert result.status == "completed"
    # The patched monotonic clock advances in fixed 0.01 s steps, so 500 additions
    # can round to ~4.999999999999996 (< 5 by float accumulation); the configured
    # 5 s hold is still verified exactly up to that rounding noise.
    assert released_at - hold_started >= 5 - 1e-9
    assert arm.current.pose == Pose(
        settings.profile.post_grasp_position_arm_m,
        settings.profile.post_grasp_orientation_arm_xyzw,
    )
    assert arm.current.gripper_width_m == settings.profile.gripper_open_m


def test_pick_without_place_frame_keeps_the_legacy_hold_and_release(settings, monkeypatch):
    events = record_events(monkeypatch, module)
    arm = FakePickArm(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event())
    result = cycle.run("legacy-no-place")
    profile = settings.profile
    assert not settings.place_enabled
    assert result.status == "completed" and result.stage == "home_end"
    assert result.message == "object grasped, moved to post-grasp pose, held and released"
    assert "hold_object" in stages(events)
    assert not {"move_to_place_approach", "lower_to_place", "retreat_from_place"} & set(
        stages(events)
    )
    assert [fields["stage"] for fields in recorded(events, "linear_motion")] == [
        "grasp_pose",
        "move_to_post_grasp",
    ]
    assert arm.targets[-1] == Pose(
        profile.post_grasp_position_arm_m, profile.post_grasp_orientation_arm_xyzw
    )
    assert arm.current.gripper_width_m == profile.gripper_open_m


def test_pick_descent_commands_follow_world_down_and_close_without_joint_settling(
    settings, monkeypatch
):
    now = 1.0
    events = []
    descent = []

    class AdvancingStop(Event):
        def wait(self, timeout=None):
            nonlocal now
            now += timeout or 0
            return self.is_set()

    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: now))
    monkeypatch.setattr(module, "event", lambda name, **fields: events.append((name, fields)))
    environment = FakePickEnvironment(settings)
    root_arm = environment.snapshot().root_to_arm

    class MovingArm(FakePickArm):
        def move(self, target):
            super().move(target)
            if cycle.stage == "grasp_pose":
                descent.append((now, target))

        def feedback(self):
            feedback = super().feedback()
            if cycle.stage == "grasp_pose":
                return replace(feedback, joint_velocities=(1.0,) * 7)
            return feedback

    arm = MovingArm(settings)
    cycle = PickRound(settings, environment, lambda: arm, AdvancingStop())
    result = cycle.run("vertical-descent")
    assert result.status == "completed", result
    start = to_pose(compose(root_arm, from_pose(cycle.plan.pregrasp)))
    positions = [to_pose(compose(root_arm, from_pose(pose))).position for _, pose in descent]
    assert len(positions) > 10
    assert all(point[:2] == pytest.approx(start.position[:2]) for point in positions)
    assert all(pose.orientation == cycle.plan.grasp.orientation for _, pose in descent)
    z = [point[2] for point in positions]
    assert all(after <= before + 1e-12 for before, after in zip(z, z[1:], strict=False))
    step = min(
        settings.limits.linear_step_m, settings.limits.linear_speed_m_s / settings.limits.servo_hz
    )
    assert all(
        np.linalg.norm(np.subtract(after, before)) <= step + 1e-12
        for before, after in zip(positions, positions[1:], strict=False)
    )
    linear = [fields for name, fields in events if name == "linear_motion"]
    assert [fields["stage"] for fields in linear] == ["grasp_pose", "move_to_post_grasp"]
    assert linear[0]["start_pose_arm"]["position"] == cycle.plan.pregrasp.position
    assert linear[0]["target_pose_arm"]["position"] == cycle.plan.grasp.position
    assert linear[1]["target_pose_arm"]["position"] == settings.profile.post_grasp_position_arm_m


def test_pick_refuses_descent_if_tcp_has_left_pregrasp(settings):
    class DriftedArm(FakePickArm):
        def feedback(self):
            feedback = super().feedback()
            if cycle.stage == "grasp_pose":
                return replace(feedback, pose=offset(feedback.pose, (0.01, 0, 0)))
            return feedback

    arm = DriftedArm(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event())
    result = cycle.run("drift-before-descent")
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted"
    assert "left the planned pregrasp pose" in result.message
    assert arm.calls[-3:] == ["stop", "release", "close"]
    assert arm.current.gripper_width_m == settings.profile.gripper_open_m


def test_pick_interrupts_during_descent_without_closing_or_retreating(settings):
    stop = Event()

    class InterruptedArm(FakePickArm):
        def move(self, target):
            super().move(target)
            if cycle.stage == "grasp_pose" and errors(target, cycle.plan.pregrasp)[0] > 0.03:
                stop.set()

    arm = InterruptedArm(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, stop)
    result = cycle.run("interrupted-descent")
    assert result.status == "interrupted" and result.stage == "grasp_pose"
    assert arm.current.gripper_width_m == settings.profile.gripper_open_m
    assert errors(arm.current.pose, cycle.plan.grasp)[0] > 0.03
    assert arm.calls[-3:] == ["stop", "release", "close"]


@pytest.mark.parametrize("duration", [0.0, 0.25, 30.0])
def test_pick_runtime_hold_controls_execution_duration(monkeypatch, duration):
    monkeypatch.setenv("PICK_HOLD_DURATION_S", str(duration))
    settings = load_settings(CONFIG)
    assert isinstance(settings, PickSettings)
    now = 1.0
    hold_started = None
    released_at = None

    class AdvancingStop(Event):
        def wait(self, timeout=None):
            nonlocal now
            assert timeout is not None
            now += timeout
            return self.is_set()

    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: now))
    arm = FakePickArm(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, AdvancingStop())
    stage = cycle._stage

    def record_stage(name):
        nonlocal hold_started, released_at
        stage(name)
        if name == "hold_object":
            hold_started = now
        elif name == "release_gripper" and released_at is None:
            released_at = now

    monkeypatch.setattr(cycle, "_stage", record_stage)
    result = cycle.run("runtime-hold")
    assert result.status == "completed", result
    assert hold_started is not None and released_at is not None
    assert released_at - hold_started == pytest.approx(duration)
    assert arm.calls[-3:] == ["stop", "release", "close"]


@pytest.mark.parametrize("occlusion", ["missing", "shifted"])
def test_pick_uses_frozen_plan_when_approach_occludes_object(settings, occlusion):
    baseline = FakePickEnvironment(settings).snapshot()
    expected = plan_pick(settings, baseline)
    moving = False

    class OccludedEnvironment:
        def snapshot(self):
            if moving:
                if occlusion == "missing":
                    raise ContractError("object TF unavailable after occlusion")
                return PickScene(
                    baseline.root_to_arm,
                    from_pose(offset(to_pose(baseline.arm_to_object), (0.1, 0, 0))),
                )
            return baseline

    class MovingArm(FakePickArm):
        def move(self, target):
            nonlocal moving
            moving = True
            super().move(target)

    arm = MovingArm(settings)
    cycle = PickRound(settings, OccludedEnvironment(), lambda: arm, Event())
    result = cycle.run("occluded-pick")
    assert result.status == "completed", result.message
    assert result.cleanup_confirmed
    assert cycle.plan == expected
    assert expected.grasp in arm.targets
    assert arm.targets[-1].position == settings.profile.post_grasp_position_arm_m


@pytest.mark.parametrize("angle,width_m", [(0, 0.043), (0, 0.05), (6, 0.044)])
def test_pick_accepts_loaded_contact(settings, angle, width_m):
    environment = upright_environment(angle)
    arm = FakePickArm(settings, object_width_m=width_m)
    result = PickRound(settings, environment, lambda: arm, Event()).run("loaded-contact")
    assert result.status == "completed", result
    assert arm.current.gripper_width_m == settings.profile.gripper_open_m


def test_pick_rejects_empty_closure(settings):
    """An empty gripper closes to the commanded width: no contact evidence -> timeout."""
    environment = FakePickEnvironment(settings)
    arm = FakePickArm(settings, object_present=False)
    settings = settings.model_copy(
        update={"limits": settings.limits.model_copy(update={"stage_timeout_s": 0.25})}
    )
    cycle = PickRound(settings, environment, lambda: arm, Event())
    result = cycle.run("rejected-width")
    assert result.status == "failed", result
    assert result.stage == "grasp_attempts_exhausted" and result.cleanup_confirmed
    assert f"'width_m': {settings.profile.gripper_closed_m}" in result.message
    assert arm.current.gripper_width_m == settings.profile.gripper_open_m
    assert arm.targets[-1].position == settings.profile.post_grasp_position_arm_m


@pytest.mark.parametrize(
    "width,accepted",
    [
        (0, False),
        # Empty jaws stop at the commanded closed width (0.03 for object_grasp.yaml).
        (0.03, False),
        # closed + margin itself is not contact (live gate is strict).
        (0.0315, False),
        (0.0316, True),
        # 0.033 used to be rejected by the fixed cube interval [0.034, 0.055].
        (0.033, True),
        (0.034, True),
        (0.04, True),
        (0.055, True),
        (0.07, True),
        # open - margin itself is not contact: the jaws never really closed.
        (0.0785, False),
        (0.08, False),
    ],
)
def test_pick_grasp_width_boundaries(settings, width, accepted):
    """Before confirmation the live contact window decides, not the cube interval."""
    profile = settings.profile
    assert profile.grasp_contact_margin_m == 0.0015
    assert profile.grasp_min_width_m == 0.034
    assert profile.grasp_max_width_m == 0.055
    arm = FakePickArm(settings)
    environment = upright_environment()
    cycle = PickRound(settings, environment, lambda: arm, Event())
    cycle.plan = plan_pick(settings, environment.snapshot())
    assert cycle._grasp_width_m is None
    assert cycle._grasped(replace(arm.current, gripper_width_m=width)) is accepted
    diagnostics = cycle._grasp_diagnostics(replace(arm.current, gripper_width_m=width))
    assert diagnostics["width_ok"] is accepted
    assert diagnostics["contact_threshold_m"] == pytest.approx(0.0315)
    assert diagnostics["contact_ceiling_m"] == pytest.approx(0.0785)
    assert diagnostics["contact_width_m"] is None
    assert diagnostics["width_in_expected_interval"] is (
        profile.grasp_min_width_m <= width <= profile.grasp_max_width_m
    )


@pytest.mark.parametrize(
    "width,holding",
    [
        (0.0425, True),
        (0.0445, True),
        (0.046, True),
        (0.0464, True),
        (0.0424, False),
        (0.047, False),
        (0.07, False),
        (0.08, False),
    ],
)
def test_pick_holding_uses_measured_contact_width_tolerance(settings, width, holding):
    """After confirmation the hold tracks the recorded measured contact width."""
    environment = upright_environment()
    arm = FakePickArm(settings)
    cycle = PickRound(settings, environment, lambda: arm, Event())
    cycle.plan = plan_pick(settings, environment.snapshot())
    cycle._grasp_width_m = 0.0445
    feedback = replace(arm.current, gripper_width_m=width)
    assert cycle._grasped(feedback) is holding
    diagnostics = cycle._grasp_diagnostics(feedback)
    assert diagnostics["width_ok"] is holding
    assert diagnostics["contact_width_m"] == pytest.approx(0.0445)
    assert diagnostics["grasp_width_tolerance_m"] == pytest.approx(0.002)


@pytest.mark.parametrize(
    "width,accepted", [(0.03, False), (0.05, True), (0.055, True), (0.0707, True), (0.079, False)]
)
def test_rotated_cube_projected_interval_is_diagnostic_only(settings, monkeypatch, width, accepted):
    """A wide-but-real contact outside the projected cube interval still grasps.

    The projected interval [0.034, 0.055] from the 45 deg cube stays in the
    diagnostics, but the live measured width (0.0707 = closed between the targets)
    now completes the round; only truly empty/open widths fail.
    """
    now = 1.0

    class AdvancingStop(Event):
        def wait(self, timeout=None):
            nonlocal now
            now += timeout or 0
            return self.is_set()

    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: now))
    limits = settings.limits.model_copy(update={"stage_timeout_s": 5.0})
    settings = settings.model_copy(update={"limits": limits})
    environment = upright_environment(45)
    arm = FakePickArm(settings, object_width_m=width)
    cycle = PickRound(settings, environment, lambda: arm, AdvancingStop())
    result = cycle.run("rotated-contact")
    assert result.status == ("completed" if accepted else "failed"), result
    assert result.stage == ("home_end" if accepted else "grasp_attempts_exhausted")
    diagnostics = cycle._grasp_diagnostics(replace(arm.current, gripper_width_m=width))
    assert diagnostics["min_width_m"] == pytest.approx(0.034)
    assert diagnostics["max_width_m"] == pytest.approx(0.055)
    assert diagnostics["width_ok"] is accepted
    # 0.0707 lies outside the calibrated cube interval; it is accepted by the live
    # measured-width gate, so the interval must be flagged as diagnostic only.
    assert diagnostics["width_in_expected_interval"] is (0.034 <= width <= 0.055)
    assert result.cleanup_confirmed
    assert arm.calls[-3:] == ["stop", "release", "close"]


def test_pick_refuses_ungraspable_projection_before_constructing_arm(settings):
    values = settings.model_dump()
    values["profile"]["gripper_open_m"] = 0.056
    settings = PickSettings.model_validate(values)
    q = np.array([0.06, 0.06, 0, 1.0])
    tilted = PickScene(
        from_pose(Pose((0, 0, 0), (0, 0, 0, 1))),
        from_pose(Pose((0.4, 0, 0.3), tuple(q / np.linalg.norm(q)))),
    )
    result = PickRound(
        settings,
        SimpleNamespace(snapshot=lambda: tilted),
        lambda: pytest.fail("Arm must not be constructed"),
        Event(),
    ).run("ungraspable-projection")
    assert result.status == "failed" and result.stage == "validate"
    assert "projected grasp width does not fit gripper" in result.message


@pytest.mark.parametrize(
    "update",
    [
        {"grasp_min_width_m": -0.001},
        {"grasp_min_width_m": 0.065},
        {"grasp_max_width_m": 0.08},
        {"gripper_closed_m": 0.08},
        {"post_grasp_position_arm_m": (0.9, 0, 0.3)},
        {"post_grasp_orientation_arm_xyzw": (0, 0, 0, 0)},
    ],
)
def test_pick_rejects_invalid_width_configuration(settings, update):
    values = settings.model_dump()
    values["profile"].update(update)
    with pytest.raises(ValueError):
        PickSettings.model_validate(values)


def test_pick_refuses_out_of_workspace_tf_without_constructing_arm(settings):
    environment = FakePickEnvironment(settings)
    scene = environment.snapshot()

    class ShiftedTarget:
        def snapshot(self):
            return PickScene(
                scene.root_to_arm,
                from_pose(offset(to_pose(scene.arm_to_object), (2, 0, 0))),
            )

    result = PickRound(
        settings, ShiftedTarget(), lambda: pytest.fail("Arm must not be constructed"), Event()
    ).run("bad-tf")
    assert result.status == "failed"
    assert "workspace" in result.message


@pytest.mark.parametrize("initially_stale", [False, True])
def test_pick_reads_tf_once_and_requires_fresh_initial_snapshot(settings, initially_stale):
    scene = FakePickEnvironment(settings).snapshot()
    reads = 0

    class ExpiringSource:
        def snapshot(self):
            nonlocal reads
            reads += 1
            if initially_stale or reads > 1:
                raise ContractError("TF snapshot is stale or future-dated")
            return scene

    arm = FakePickArm(settings)

    def arm_factory():
        assert not initially_stale, "stale initial TF must fail before Arm construction"
        return arm

    cycle = PickRound(settings, ExpiringSource(), arm_factory, Event())
    result = cycle.run("tf-expires-during-acquire")
    assert reads == 1
    assert result.cleanup_confirmed
    if initially_stale:
        assert result.status == "failed" and result.stage == "validate"
        assert "TF snapshot is stale" in result.message
        assert not arm.calls
    else:
        assert result.status == "completed", result.message
        assert cycle.plan == plan_pick(settings, scene)
        assert cycle.plan.grasp in arm.targets


def test_pick_control_denial_sends_no_motion(settings):
    environment = FakePickEnvironment(settings)

    class Denied(FakePickArm):
        def acquire(self):
            raise ContractError("denied")

    arm = Denied(settings)
    result = PickRound(settings, environment, lambda: arm, Event()).run("denied")
    assert result.status == "failed" and result.stage == "retry_restore_session"
    assert "denied" in result.message and "recovery failed" in result.message
    assert "servo" not in arm.calls and "gripper" not in arm.calls
    assert arm.calls[-3:] == ["stop", "release", "close"]


def test_pick_offline_entrypoint(settings):
    assert asyncio.run(serve(CONFIG, live=False)) == 0
