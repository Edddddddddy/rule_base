"""Purpose: Guard the vision target frame, bounded visual servo and its failure semantics.

Covers the frozen contract (tools/vision/CONTRACT.md §6): target-frame planning, the
observation/pregrasp refresh loop, max_corrections bounding, refresh failure/timeout,
missing grasp_target TF and the additive vision events.

Usage: source ros2_ws/setup_ros_env.sh && python3 -m pytest tests/unit/test_pick_vision_servo.py
"""

from __future__ import annotations

from threading import Event
from types import SimpleNamespace

import numpy as np
import pytest
from pick_place_support import offline_settings, record_events, recorded, stages, with_place

from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import ContractError, PickScene, Pose
from rule_base.infrastructure.fakes.runtime import FakePickArm, FakePickEnvironment, tf_entry
from rule_base.infrastructure.transform.math import errors, from_pose, to_pose
from rule_base.infrastructure.transform.observations import PickObservationCache
from rule_base.stations.pick_place import execution as module
from rule_base.stations.pick_place import vision_servo
from rule_base.stations.pick_place.execution import PickRound
from rule_base.stations.pick_place.planning import first_observation_pose, plan_pick

TARGET_FRAME = "grasp_target_01"


def target_settings(**vision_updates: object) -> PickSettings:
    """Offline fake settings with grasp_target/vision enabled and no settle delay."""
    settings = offline_settings()
    settings = settings.model_copy(
        update={"frames": settings.frames.model_copy(update={"grasp_target": TARGET_FRAME})}
    )
    updates: dict[str, object] = {"enabled": True, "correction_settle_s": 0.0}
    updates.update(vision_updates)
    return settings.model_copy(
        update={"vision": settings.vision.model_copy(update=updates)}
    )


def target_only_settings() -> PickSettings:
    """grasp_target configured but vision disabled (target-frame planning only)."""
    settings = offline_settings()
    return settings.model_copy(
        update={"frames": settings.frames.model_copy(update={"grasp_target": TARGET_FRAME})}
    )


def legacy_plan() -> object:
    settings = offline_settings()
    return plan_pick(settings, FakePickEnvironment(settings).snapshot())


@pytest.fixture(scope="module")
def legacy():
    return legacy_plan()


def moved(pose: Pose, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> Pose:
    return Pose(
        tuple(value + delta for value, delta in zip(pose.position, (dx, dy, dz), strict=True)),
        pose.orientation,
    )


def run_round(
    settings: PickSettings,
    monkeypatch: pytest.MonkeyPatch,
    *,
    legacy_grasp: Pose,
    target_updates: tuple[Pose, ...] = (),
    refresh_responses: tuple[str, ...] = (),
    publish_on_refresh: bool = True,
    publish_target: bool = True,
    refresh_delay_s: float = 0.0,
    arm_factory: object = FakePickArm,
    round_id: str = "vision-round",
) -> SimpleNamespace:
    events = record_events(monkeypatch, module)
    monkeypatch.setattr(
        vision_servo, "event", lambda name, **fields: events.append((name, fields))
    )
    environment = FakePickEnvironment(
        settings, target_pose=legacy_grasp, publish_target=publish_target
    )
    environment.target_updates = list(target_updates)
    environment.refresh_responses = list(refresh_responses)
    environment.publish_on_refresh = publish_on_refresh
    environment.refresh_delay_s = refresh_delay_s
    arm = arm_factory(settings)  # type: ignore[operator]
    cycle = PickRound(settings, environment, lambda: arm, Event())
    result = cycle.run(round_id)
    return SimpleNamespace(
        result=result,
        events=events,
        names=[name for name, _ in events],
        environment=environment,
        arm=arm,
        cycle=cycle,
        settings=settings,
    )


def named(rig: SimpleNamespace, name: str) -> list[dict[str, object]]:
    return recorded(rig.events, name)


class WidthRecordingArm(FakePickArm):
    def __init__(self, settings: PickSettings) -> None:
        super().__init__(settings)
        self.gripper_widths: list[float] = []

    def gripper(self, width_m: float, effort: float) -> None:
        self.gripper_widths.append(width_m)
        super().gripper(width_m, effort)


# -- planning from the vision target ------------------------------------


def test_grasp_target_plan_reproduces_the_legacy_top_grasp(legacy):
    settings = target_only_settings()
    scene = FakePickEnvironment(settings, target_pose=legacy.grasp).snapshot()
    assert scene.arm_to_target is not None
    plan = plan_pick(settings, scene)
    assert plan.grasp.position == pytest.approx(legacy.grasp.position)
    assert plan.grasp.orientation == pytest.approx(legacy.grasp.orientation)
    assert plan.pregrasp.position == pytest.approx(legacy.pregrasp.position)
    assert plan.pregrasp.orientation == pytest.approx(legacy.pregrasp.orientation)
    assert plan.observation == legacy.observation
    assert plan.expected_cube_width_m == pytest.approx(legacy.expected_cube_width_m)
    assert plan.grasp_min_width_m == pytest.approx(legacy.grasp_min_width_m)
    assert plan.grasp_max_width_m == pytest.approx(legacy.grasp_max_width_m)
    # pregrasp = grasp + root +Z * pregrasp_clearance_m, keeping the grasp orientation.
    profile = settings.profile
    root_arm_rotation = np.asarray(scene.root_to_arm)[:3, :3]
    upward = root_arm_rotation.T @ np.array([0.0, 0.0, profile.pregrasp_clearance_m])
    assert np.asarray(plan.pregrasp.position) - plan.grasp.position == pytest.approx(upward)


def _flipped_about_approach(pose: Pose) -> Pose:
    """Return the same grasp line with the TCP closing axis flipped (180 deg roll)."""
    transform = np.asarray(from_pose(pose)).copy()
    transform[:3, 1] = -transform[:3, 1]
    transform[:3, 2] = -transform[:3, 2]
    return to_pose(transform)


def test_target_closing_axis_is_flipped_to_the_reachable_roll_family(legacy):
    """The vision target may ask for +Y closing while the arm only reaches -Y."""
    settings = target_only_settings()
    flipped = _flipped_about_approach(legacy.grasp)
    scene = FakePickEnvironment(settings, target_pose=flipped).snapshot()
    plan = plan_pick(settings, scene)
    # Position / approach are unchanged by the roll flip; the plan must reproduce
    # the legacy (profile reference) roll family exactly.
    assert plan.grasp.position == pytest.approx(legacy.grasp.position)
    distance, angle = errors(plan.grasp, legacy.grasp)
    assert distance == pytest.approx(0.0, abs=1e-9)
    assert angle == pytest.approx(0.0, abs=1e-9)
    assert plan.pregrasp.position == pytest.approx(legacy.pregrasp.position)
    assert errors(plan.pregrasp, legacy.pregrasp)[1] == pytest.approx(0.0, abs=1e-9)
    assert plan.observation.position == pytest.approx(legacy.observation.position)
    assert errors(plan.observation, legacy.observation)[1] == pytest.approx(0.0, abs=1e-9)


class ObservationDeniedArm(FakePickArm):
    """First servo move after start_servo (the observation) is IK-denied."""

    def __init__(self, settings: PickSettings) -> None:
        super().__init__(settings)
        self._deny_next = False
        self.denied = False

    def start_servo(self, hold_pose: Pose) -> None:
        super().start_servo(hold_pose)
        self._deny_next = True

    def move(self, target: Pose) -> None:
        if self._deny_next:
            self._deny_next = False
            self.denied = True
            raise ContractError(
                "Arm plan_request denied: IK_FAILED: /compute_ik error_code=-31"
            )
        super().move(target)


def test_unreachable_observation_is_skipped_in_vision_mode(monkeypatch, legacy):
    arm = ObservationDeniedArm(target_settings())

    rig = run_round(
        target_settings(), monkeypatch, legacy_grasp=legacy.grasp, arm_factory=lambda _s: arm
    )
    assert "observation_skipped" in rig.names
    skipped = named(rig, "observation_skipped")[0]
    assert "IK_FAILED" in str(skipped["error"])
    assert arm.denied is True
    assert rig.result.status == "completed"
    # The pregrasp refresh still runs, so visual servoing is not bypassed silently.
    assert named(rig, "vision_refresh")
    assert "pregrasp" in stages(rig.events)


def test_unreachable_observation_still_fails_without_vision(monkeypatch, legacy):
    arm = ObservationDeniedArm(target_only_settings())
    rig = run_round(
        target_only_settings(), monkeypatch, legacy_grasp=legacy.grasp,
        arm_factory=lambda _s: arm,
    )
    assert "observation_skipped" not in rig.names
    assert rig.result.status == "failed"


def test_grasp_target_plan_still_validates_the_top_grasp_pose(legacy):
    settings = target_only_settings()
    scene = FakePickEnvironment(settings, target_pose=legacy.grasp).snapshot()
    # 90 deg about the arm +Z: the TCP approach axis stops pointing down in the root.
    sideways = PickScene(
        scene.root_to_arm,
        scene.arm_to_object,
        scene.arm_to_place,
        from_pose(Pose(legacy.grasp.position, (0.0, 0.0, 2**-0.5, 2**-0.5))),
    )
    with pytest.raises(ContractError, match="approach axis down"):
        plan_pick(settings, sideways)
    assert scene.arm_to_target is not None


def test_grasp_target_outside_the_workspace_is_rejected(legacy):
    settings = target_only_settings()
    scene = FakePickEnvironment(settings, target_pose=legacy.grasp).snapshot()
    outside = PickScene(
        scene.root_to_arm,
        scene.arm_to_object,
        scene.arm_to_place,
        from_pose(Pose((2.0, 0.0, 0.4), legacy.grasp.orientation)),
    )
    with pytest.raises(ContractError, match="workspace"):
        plan_pick(settings, outside)


def _with_grasp_offset(settings, offset_m):
    return settings.model_copy(
        update={
            "profile": settings.profile.model_copy(
                update={"grasp_target_offset_arm_m": offset_m}
            )
        }
    )


def test_grasp_target_offset_shifts_grasp_pregrasp_and_observation(legacy):
    """S6j：offset 只平移 vision grasp 位置（姿态不变），pregrasp/observation 跟随。"""
    settings = target_only_settings()
    scene = FakePickEnvironment(settings, target_pose=legacy.grasp).snapshot()
    base_plan = plan_pick(settings, scene)
    offset_m = (0.0, 0.015, 0.0)
    plan = plan_pick(_with_grasp_offset(settings, offset_m), scene)

    def shifted(position):
        return tuple(a + b for a, b in zip(position, offset_m, strict=True))

    assert plan.grasp.position == pytest.approx(shifted(base_plan.grasp.position))
    # Only the position moves: the top-grasp orientation stays unlocked/locked as before.
    assert plan.grasp.orientation == pytest.approx(base_plan.grasp.orientation)
    assert plan.pregrasp.position == pytest.approx(shifted(base_plan.pregrasp.position))
    assert plan.observation.position == pytest.approx(shifted(base_plan.observation.position))


def test_grasp_target_offset_is_checked_by_the_workspace_gate(legacy):
    settings = target_only_settings()
    scene = FakePickEnvironment(settings, target_pose=legacy.grasp).snapshot()
    base_grasp = plan_pick(settings, scene).grasp
    profile = settings.profile
    tight = profile.model_copy(
        update={
            "grasp_target_offset_arm_m": (0.0, 0.015, 0.0),
            "workspace_max_m": (
                profile.workspace_max_m[0],
                base_grasp.position[1] + 0.01,
                profile.workspace_max_m[2],
            ),
        }
    )
    gated = settings.model_copy(update={"profile": tight})
    with pytest.raises(ContractError, match="workspace"):
        plan_pick(gated, scene)


def test_grasp_target_offset_keeps_the_top_grasp_gate(legacy):
    settings = _with_grasp_offset(target_only_settings(), (0.0, 0.015, 0.0))
    scene = FakePickEnvironment(settings, target_pose=legacy.grasp).snapshot()
    # 90 deg about the arm +Z: the TCP approach axis stops pointing down in the root.
    sideways = PickScene(
        scene.root_to_arm,
        scene.arm_to_object,
        scene.arm_to_place,
        from_pose(Pose(legacy.grasp.position, (0.0, 0.0, 2**-0.5, 2**-0.5))),
    )
    with pytest.raises(ContractError, match="approach axis down"):
        plan_pick(settings, sideways)


def test_missing_grasp_target_tf_fails_explicitly(monkeypatch, legacy):
    rig = run_round(
        target_settings(),
        monkeypatch,
        legacy_grasp=legacy.grasp,
        publish_target=False,
    )
    assert rig.result.status == "failed"
    assert rig.result.stage == "validate"
    assert "Pick grasp_target TF is missing" in rig.result.message
    assert rig.result.cleanup_confirmed
    assert rig.arm.calls == []


# -- the vision-disabled path stays unchanged ----------------------------


def test_unconfigured_vision_keeps_the_legacy_round(monkeypatch):
    settings = offline_settings()
    rig = run_round(settings, monkeypatch, legacy_grasp=legacy_plan().grasp)
    assert rig.result.status == "completed", rig.result
    assert rig.environment.refresh_calls == 0
    assert not [name for name in rig.names if name.startswith("vision_")]
    assert "vision_align" not in stages(rig.events)
    legacy = legacy_plan()
    assert rig.cycle.plan.grasp == legacy.grasp
    assert rig.cycle.plan.observation == legacy.observation


def test_vision_disabled_uses_the_target_plan_without_refreshing(monkeypatch, legacy):
    settings = target_only_settings()
    assert not settings.vision_enabled
    rig = run_round(settings, monkeypatch, legacy_grasp=legacy.grasp)
    assert rig.result.status == "completed", rig.result
    assert rig.environment.refresh_calls == 0
    assert not [name for name in rig.names if name.startswith("vision_")]
    assert rig.cycle.plan.grasp.position == pytest.approx(legacy.grasp.position)
    assert rig.cycle.plan.grasp.orientation == pytest.approx(legacy.grasp.orientation)
    assert "grasp_pose" in stages(rig.events)


# -- bounded visual servo ------------------------------------------------


def test_vision_refresh_without_target_motion_needs_no_correction(monkeypatch, legacy):
    rig = run_round(target_settings(), monkeypatch, legacy_grasp=legacy.grasp)
    assert rig.result.status == "completed", rig.result
    assert rig.environment.refresh_calls == 2
    refreshes = named(rig, "vision_refresh")
    assert len(refreshes) == 2
    assert all(fields["ok"] is True for fields in refreshes)
    assert [fields["stage"] for fields in refreshes] == [
        "vision_align",
        "vision_pregrasp_align",
    ]
    assert named(rig, "vision_correction") == []
    aligned = named(rig, "vision_aligned")
    assert [fields["attempts"] for fields in aligned] == [0, 0]
    assert "vision_correct_pregrasp" not in stages(rig.events)


def test_observation_refresh_replans_before_the_pregrasp_move(monkeypatch, legacy):
    settings = target_settings()
    shifted = moved(legacy.grasp, dx=0.02)
    rig = run_round(
        settings, monkeypatch, legacy_grasp=legacy.grasp, target_updates=(shifted,)
    )
    assert rig.result.status == "completed", rig.result
    corrections = named(rig, "vision_correction")
    assert len(corrections) == 1
    assert corrections[0]["stage"] == "vision_align"
    assert corrections[0]["attempt"] == 1
    assert corrections[0]["delta_position_m"] == pytest.approx(0.02)
    assert corrections[0]["delta_angle_rad"] == pytest.approx(0.0, abs=1e-6)
    assert rig.cycle.plan is not None
    assert rig.cycle.plan.grasp.position == pytest.approx(shifted.position)
    stage_names = stages(rig.events)
    assert stage_names.index("vision_align") < stage_names.index("pregrasp")
    # The arm still descends onto the corrected grasp pose.
    assert any(
        errors(target, rig.cycle.plan.grasp)[0] <= 1e-9 for target in rig.arm.targets
    )
    assert stage_names.index("grasp_pose") > stage_names.index("pregrasp")


def test_pregrasp_refresh_corrects_once_then_descends(monkeypatch, legacy):
    settings = target_settings()
    first = moved(legacy.grasp, dx=0.02)
    second = moved(legacy.grasp, dx=0.04)
    rig = run_round(
        settings,
        monkeypatch,
        legacy_grasp=legacy.grasp,
        target_updates=(first, second),
    )
    assert rig.result.status == "completed", rig.result
    corrections = named(rig, "vision_correction")
    assert [fields["stage"] for fields in corrections] == [
        "vision_align",
        "vision_pregrasp_align",
    ]
    assert corrections[1]["attempt"] == 1
    assert corrections[1]["delta_position_m"] == pytest.approx(0.02)
    aligned = named(rig, "vision_aligned")
    assert [fields["stage"] for fields in aligned] == ["vision_pregrasp_align"]
    assert aligned[0]["attempts"] == 1
    assert aligned[0]["residual_position_m"] == pytest.approx(0.0, abs=1e-6)
    stage_names = stages(rig.events)
    pregrasp_index = stage_names.index("pregrasp")
    correction_index = stage_names.index("vision_correct_pregrasp")
    wait_index = stage_names.index("wait_open_gripper")
    assert pregrasp_index < correction_index < wait_index
    assert rig.cycle.plan is not None
    assert rig.cycle.plan.grasp.position == pytest.approx(second.position)
    assert any(
        errors(target, rig.cycle.plan.pregrasp)[0] <= 1e-9 for target in rig.arm.targets
    )


def test_vision_alignment_is_bounded_and_never_descends(monkeypatch, legacy):
    settings = target_settings()  # contract default max_corrections = 2
    runway = tuple(moved(legacy.grasp, dx=0.02 * step) for step in range(1, 5))
    rig = run_round(
        settings,
        monkeypatch,
        legacy_grasp=legacy.grasp,
        target_updates=runway,
        arm_factory=WidthRecordingArm,
    )
    assert rig.result.status == "failed"
    assert rig.result.stage == "vision_pregrasp_align"
    assert "vision_alignment_incomplete" in rig.result.message
    assert "corrections=2" in rig.result.message
    assert rig.result.cleanup_confirmed
    corrections = named(rig, "vision_correction")
    assert [fields["attempt"] for fields in corrections if fields["stage"] == "vision_pregrasp_align"] == [
        1,
        2,
    ]
    # No blind descent, no jaw closure, no invented recovery motion.
    stage_names = stages(rig.events)
    assert "grasp_pose" not in stage_names
    assert "close_gripper" not in stage_names
    assert "recovery_started" not in rig.names
    assert settings.profile.gripper_closed_m not in rig.arm.gripper_widths
    assert rig.arm.gripper_widths and all(
        width == pytest.approx(settings.profile.gripper_open_m)
        for width in rig.arm.gripper_widths
    )


def test_refresh_service_failure_fails_the_round(monkeypatch, legacy):
    rig = run_round(
        target_settings(),
        monkeypatch,
        legacy_grasp=legacy.grasp,
        refresh_responses=("NO_MARKER: target not visible",),
    )
    assert rig.result.status == "failed"
    assert rig.result.stage == "vision_align"
    assert "vision refresh failed: NO_MARKER" in rig.result.message
    assert rig.result.cleanup_confirmed
    refreshes = named(rig, "vision_refresh")
    assert len(refreshes) == 1
    assert refreshes[0]["ok"] is False
    assert "NO_MARKER" in str(refreshes[0]["error"])
    assert "open_gripper" not in stages(rig.events)
    assert "grasp_pose" not in stages(rig.events)


def test_refresh_service_timeout_fails_the_round(monkeypatch, legacy):
    rig = run_round(
        target_settings(refresh_timeout_s=0.02),
        monkeypatch,
        legacy_grasp=legacy.grasp,
        refresh_delay_s=0.05,
    )
    assert rig.result.status == "failed"
    assert rig.result.stage == "vision_align"
    assert "vision refresh service timed out" in rig.result.message
    refreshes = named(rig, "vision_refresh")
    assert len(refreshes) == 1
    assert refreshes[0]["ok"] is False
    assert "timed out" in str(refreshes[0]["error"])
    assert "grasp_pose" not in stages(rig.events)


def test_refresh_without_a_new_tf_sample_times_out(monkeypatch, legacy):
    rig = run_round(
        target_settings(refresh_timeout_s=0.05),
        monkeypatch,
        legacy_grasp=legacy.grasp,
        publish_on_refresh=False,
    )
    assert rig.result.status == "failed"
    assert rig.result.stage == "vision_align"
    assert "timed out waiting for a new TF sample" in rig.result.message
    refreshes = named(rig, "vision_refresh")
    assert len(refreshes) == 1
    assert refreshes[0]["ok"] is False
    assert "requires new" in str(refreshes[0]["error"])


def test_vision_place_semantics_are_preserved(monkeypatch, legacy):
    settings = with_place(target_settings())
    rig = run_round(settings, monkeypatch, legacy_grasp=legacy.grasp)
    assert rig.result.status == "completed", rig.result
    assert rig.cycle.plan is not None and rig.cycle.plan.place is not None
    stage_names = stages(rig.events)
    assert stage_names.index("wait_open_gripper") < stage_names.index("grasp_pose")
    assert "lower_to_place" in stage_names
    assert "retreat_from_place" in stage_names


# -- event contract ------------------------------------------------------


def test_vision_events_carry_the_contract_fields(monkeypatch, legacy):
    settings = target_settings()
    first = moved(legacy.grasp, dx=0.02)
    second = moved(legacy.grasp, dx=0.04)
    rig = run_round(
        settings,
        monkeypatch,
        legacy_grasp=legacy.grasp,
        target_updates=(first, second),
    )
    refresh = named(rig, "vision_refresh")[0]
    assert {"stage", "ok", "age_s"} <= set(refresh)
    assert isinstance(refresh["age_s"], float)
    correction = named(rig, "vision_correction")[0]
    assert {"stage", "attempt", "delta_position_m", "delta_angle_rad"} <= set(correction)
    aligned = named(rig, "vision_aligned")[0]
    assert {
        "stage",
        "attempts",
        "residual_position_m",
        "residual_angle_rad",
    } <= set(aligned)


# -- observation-cache freshness ----------------------------------------


def test_retry_requires_a_new_grasp_target_sample(monkeypatch):
    settings = target_only_settings()
    frames = settings.frames
    assert frames.grasp_target is not None
    cache = PickObservationCache(settings)
    now = 10_000_000_000
    monkeypatch.setattr("rule_base.infrastructure.transform.buffer.time.time_ns", lambda: now)
    monkeypatch.setattr("rule_base.infrastructure.transform.observations.time.time_ns", lambda: now)
    cache.update(
        "tf_static",
        {"transforms": [tf_entry(frames.root, frames.arm_base, (0, 0, 0), timestamp_ns=0)]},
    )
    cache.update(
        "tf",
        {
            "transforms": [
                tf_entry(frames.arm_base, frames.object, (0.52, 0.0, 0.38), timestamp_ns=now),
                tf_entry(frames.object, frames.grasp_target, (-0.04, 0.0, 0.0), timestamp_ns=now),
            ]
        },
    )
    assert to_pose(cache.snapshot().arm_to_target).position == pytest.approx((0.48, 0.0, 0.38))
    now += 1_000_000
    cache.request_new_observation()
    # Only the object is republished; the stale target must not be reused.
    cache.update(
        "tf",
        {"transforms": [tf_entry(frames.arm_base, frames.object, (0.52, 0.0, 0.38), timestamp_ns=now)]},
    )
    with pytest.raises(ContractError, match="requires new grasp_target TF"):
        cache.snapshot()
    now += 1_000_000
    cache.update(
        "tf",
        {"transforms": [tf_entry(frames.object, frames.grasp_target, (-0.03, 0.0, 0.0), timestamp_ns=now)]},
    )
    assert to_pose(cache.snapshot().arm_to_target).position == pytest.approx((0.49, 0.0, 0.38))


def test_vision_refresh_requires_new_object_and_target_samples(monkeypatch):
    settings = target_only_settings()
    frames = settings.frames
    cache = PickObservationCache(settings)
    now = 20_000_000_000
    monkeypatch.setattr("rule_base.infrastructure.transform.buffer.time.time_ns", lambda: now)
    monkeypatch.setattr("rule_base.infrastructure.transform.observations.time.time_ns", lambda: now)
    cache.update(
        "tf_static",
        {"transforms": [tf_entry(frames.root, frames.arm_base, (0, 0, 0), timestamp_ns=0)]},
    )
    cache.update(
        "tf",
        {
            "transforms": [
                tf_entry(frames.arm_base, frames.object, (0.52, 0.0, 0.38), timestamp_ns=now),
                tf_entry(frames.object, frames.grasp_target, (-0.04, 0.0, 0.0), timestamp_ns=now),
            ]
        },
    )
    cache.snapshot()
    now += 1_000_000
    cache.request_new_vision_sample()
    assert cache.sample_age_s() is None
    with pytest.raises(ContractError, match="Pick vision refresh requires new object TF"):
        cache.snapshot()
    now += 1_000_000
    cache.update(
        "tf",
        {
            "transforms": [
                tf_entry(frames.arm_base, frames.object, (0.52, 0.0, 0.38), timestamp_ns=now),
                tf_entry(frames.object, frames.grasp_target, (-0.04, 0.0, 0.0), timestamp_ns=now),
            ]
        },
    )
    assert cache.snapshot().arm_to_target is not None
    assert cache.sample_age_s() == pytest.approx(0.0, abs=1e-6)


# --- task-33：rest → 关节空间观察位 → TF 快照/抓取（不走 IK PlanRequest） ------------
OBSERVE_JOINTS = (-0.046, 0.0491, -0.3175, 1.738, -1.4281)


class StuckObserveArm(FakePickArm):
    """Ignore only the observe-joint target (timeout path); other joint/home moves work."""

    def move_joints(self, positions: tuple[float, ...]) -> None:
        if len(positions) >= 5 and tuple(positions[:5]) == pytest.approx(OBSERVE_JOINTS):
            self.calls.append("move_joints_stuck_observe")
            return
        super().move_joints(positions)


def test_observe_joint_positions_moves_joint_space_and_skips_ik_observation(
        monkeypatch, legacy):
    settings = target_settings(observe_first=True,
                               observe_joint_positions=OBSERVE_JOINTS)
    rig = run_round(settings, monkeypatch, legacy_grasp=legacy.grasp)
    assert rig.result.status == "completed", rig.result.message
    # 关节流命令包含用户观察关节（前 5 位），且 home → 观察先后发生
    matching = [command for command in rig.arm.joint_commands
                if tuple(command[:5]) == pytest.approx(OBSERVE_JOINTS)]
    assert matching, rig.arm.joint_commands
    home = settings.ros2.home_joint_positions[:5]
    home_index = next(i for i, command in enumerate(rig.arm.joint_commands)
                      if tuple(command[:5]) == pytest.approx(home))
    observe_index = next(i for i, command in enumerate(rig.arm.joint_commands)
                         if tuple(command[:5]) == pytest.approx(OBSERVE_JOINTS))
    assert home_index < observe_index
    # 不走向上 IK 观察位 TCP 移动（observe_first_plan 的 _move/PlanRequest 被跳过）
    assert first_observation_pose(settings) not in rig.arm.targets
    assert "vision_observe_joints_complete" in rig.names
    assert "vision_observe_stable" in rig.names
    assert "pick_plan" in rig.names


def test_without_observe_joint_positions_keeps_ik_observation_path(monkeypatch, legacy):
    settings = target_settings(observe_first=True)
    rig = run_round(settings, monkeypatch, legacy_grasp=legacy.grasp)
    assert rig.result.status == "completed", rig.result.message
    assert first_observation_pose(settings) in rig.arm.targets
    assert "vision_observe_joints_complete" not in rig.names


def test_observe_joint_positions_timeout_fails_closed(monkeypatch, legacy):
    settings = target_settings(observe_first=True,
                               observe_joint_positions=OBSERVE_JOINTS)
    settings = settings.model_copy(update={
        "limits": settings.limits.model_copy(update={
            "home_timeout_s": 0.5, "motion_stable_time_s": 0.0})})
    rig = run_round(settings, monkeypatch, legacy_grasp=legacy.grasp,
                    arm_factory=StuckObserveArm)
    assert rig.result.status == "failed"
    assert "vision_observe_joints" in rig.result.message
    assert rig.result.cleanup_confirmed is True
