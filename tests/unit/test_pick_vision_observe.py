"""Purpose: guard observe-first sampling/lock and pregrasp reachability fallbacks.

Split out of test_pick_vision_servo.py (D-132) to keep every test module < 800 lines.
Usage: source ros2_ws/setup_ros_env.sh && python3 -m pytest tests/unit/test_pick_vision_observe.py
"""

from __future__ import annotations

from threading import Event

import pytest
from pick_place_support import (
    logged_line,
    logged_values,
    offline_settings,
    record_events,
    with_place,
)
from pydantic import ValidationError

from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import ContractError, Pose
from rule_base.infrastructure.fakes.runtime import (
    PICK_MOCK_OBJECT_POSE,
    PICK_MOCK_PLACE_POSE,
    FakePickArm,
    FakePickEnvironment,
)
from rule_base.stations.pick_place import execution as module
from rule_base.stations.pick_place import vision_servo
from rule_base.stations.pick_place.execution import PickRound
from test_pick_vision_servo import (
    TARGET_FRAME,
    legacy_plan,
    moved,
    named,
    run_round,
    stages,
    target_settings,
)


@pytest.fixture(scope="module")
def legacy():
    return legacy_plan()


# -- observe-first round flow (D-127) -----------------------------------


class ObserveFirstEnvironment(FakePickEnvironment):
    """Object TF is only published after the first vision refresh (detected at observation)."""

    def refresh_object_pose(self) -> str:
        self.publish_target = True
        return super().refresh_object_pose()


def test_observe_first_requires_vision_enabled() -> None:
    values = offline_settings().model_dump()
    values["frames"]["grasp_target"] = TARGET_FRAME
    values["vision"]["enabled"] = False
    values["vision"]["observe_first"] = True
    with pytest.raises(ValidationError, match="observe_first"):
        PickSettings.model_validate(values)


def test_observe_first_rejects_invalid_observation_pose() -> None:
    values = target_settings().model_dump()
    values["vision"]["observe_orientation_arm_xyzw"] = [0.0, 0.0, 0.0, 2.0]
    with pytest.raises(ValidationError, match="observe_orientation"):
        PickSettings.model_validate(values)


def test_observe_first_moves_then_plans_from_the_fresh_detection(monkeypatch) -> None:
    settings = target_settings(
        observe_first=True, observe_position_arm_m=(0.31, 0.02, 0.22)
    )
    execution_events = record_events(monkeypatch, module)
    vision_events = record_events(monkeypatch, vision_servo)
    environment = ObserveFirstEnvironment(settings, publish_target=False)
    arm = FakePickArm(settings)
    cycle = PickRound(settings, environment, lambda: arm, Event())

    result = cycle.run("observe-first")
    assert result.status == "completed", result.message
    # targets[0] is the servo hold pose; the first move is the fixed observation pose.
    observation_target = arm.targets[1]
    assert observation_target.position == pytest.approx(
        settings.vision.observe_position_arm_m
    )
    assert observation_target.orientation == pytest.approx(
        settings.vision.observe_orientation_arm_xyzw
    )
    stage_names = stages(execution_events)
    assert "vision_observe" in stage_names
    assert "pick_plan" in [name for name, _fields in vision_events]
    assert "vision_refresh" in [name for name, _fields in vision_events]
    # The plan was produced from the refreshed detection (target only existed after refresh).
    assert cycle.plan is not None
    assert environment.refresh_calls >= 1


OBSERVE_JOINTS = (-0.046, 0.0491, -0.3175, 1.738, -1.4281)


def test_observe_samples_settle_first_and_are_spaced(monkeypatch, legacy):
    """S6g/S6i：到达观察位先 settle → 丢 2 帧预热 → 间隔采样。"""
    settings = target_settings(
        observe_first=True,
        observe_settle_s=0.8,
        observe_sample_interval_s=0.2,
        observe_warmup_samples=2,
        observe_samples=3,
    )
    sequence: list[tuple[str, float]] = []

    def fake_settle(cycle, seconds):
        sequence.append(("settle", float(seconds)))

    class RecordingEnvironment(ObserveFirstEnvironment):
        def refresh_object_pose(self) -> str:
            sequence.append(("refresh", 0.0))
            return super().refresh_object_pose()

    monkeypatch.setattr(vision_servo, "_settle", fake_settle)
    record_events(monkeypatch, module)
    record_events(monkeypatch, vision_servo)
    environment = RecordingEnvironment(settings, publish_target=False)
    cycle = PickRound(settings, environment, lambda: FakePickArm(settings), Event())

    result = cycle.run("observe-settled")
    assert result.status == "completed", result.message
    # settle → warmup frame 1 → interval → warmup frame 2 → sample frames with interval.
    assert sequence[:9] == [
        ("settle", 0.8),
        ("refresh", 0.0),
        ("settle", 0.2),
        ("refresh", 0.0),
        ("refresh", 0.0),
        ("settle", 0.2),
        ("refresh", 0.0),
        ("settle", 0.2),
        ("refresh", 0.0),
    ]
    # Any further refresh (pregrasp alignment) must not be preceded by a settle.
    assert all(kind == "refresh" for kind, _seconds in sequence[9:])


def test_observe_warmup_discards_stale_frames_before_the_stability_gate(monkeypatch, legacy):
    """S6i：观察位首帧旧 coast 目标（现场差 64 mm）在预热阶段被丢弃，不进入稳定门。"""
    stale = moved(legacy.grasp, dx=0.064)
    settings = target_settings(
        observe_first=True,
        lock_target_after_observation=True,
        observe_samples=3,
        observe_warmup_samples=2,
        observe_settle_s=0.0,
        observe_sample_interval_s=0.0,
    )
    updates = (stale, stale, legacy.grasp, legacy.grasp, legacy.grasp)
    rig = run_round(
        settings, monkeypatch, legacy_grasp=legacy.grasp, target_updates=updates
    )
    assert rig.result.status == "completed", rig.result.message
    warmup = named(rig, "vision_observe_warmup")
    assert warmup and int(warmup[0]["discarded"]) == 2
    warmup_refreshes = [
        fields
        for name, fields in rig.events
        if name == "vision_refresh" and fields.get("stage") == "vision_observe_warmup"
    ]
    assert len(warmup_refreshes) == 2
    stable = named(rig, "vision_observe_stable")[0]
    assert int(stable["samples"]) == 3
    assert int(stable["requested"]) == 3
    assert float(stable["position_spread_m"]) == pytest.approx(0.0, abs=1e-9)
    # 2 warmup + 3 sampled refreshes; the locked round refreshes nothing else.
    assert rig.environment.refresh_calls == 5


def test_observe_warmup_refresh_failure_is_not_fatal(monkeypatch, legacy):
    """S6i：预热 refresh 失败只记事件，不影响后续采样帧与稳定门。"""
    settings = target_settings(
        observe_first=True,
        lock_target_after_observation=True,
        observe_samples=3,
        observe_warmup_samples=2,
        observe_settle_s=0.0,
        observe_sample_interval_s=0.0,
    )
    rig = run_round(
        settings,
        monkeypatch,
        legacy_grasp=legacy.grasp,
        refresh_responses=("stale coast target",),
    )
    assert rig.result.status == "completed", rig.result.message
    failed = named(rig, "vision_observe_warmup_failed")
    assert failed and int(failed[0]["index"]) == 0
    stable = named(rig, "vision_observe_stable")[0]
    assert int(stable["samples"]) == 3


def test_observation_joint_move_uses_the_strict_observe_tolerance(monkeypatch, legacy):
    """S6g：观察关节移动用 observe_joint_tolerance_rad；home 仍用 home_tolerance_rad。"""
    tolerances: list[float | None] = []
    original = module.joint_motion.stream_joints

    def spy(cycle, target, **kwargs):
        tolerances.append(kwargs.get("tolerance_rad"))
        return original(cycle, target, **kwargs)

    monkeypatch.setattr(module.joint_motion, "stream_joints", spy)
    settings = target_settings(
        observe_first=True,
        observe_joint_positions=OBSERVE_JOINTS,
        observe_settle_s=0.0,
        observe_sample_interval_s=0.0,
    )
    rig = run_round(settings, monkeypatch, legacy_grasp=legacy.grasp)
    assert rig.result.status == "completed", rig.result.message
    # home_start/home_end pass no tolerance (None -> limits.home_tolerance_rad, 0.08);
    # the observation move uses the stricter vision tolerance.
    assert tolerances[0] is None and tolerances[-1] is None
    assert tolerances[1] == pytest.approx(settings.vision.observe_joint_tolerance_rad)


def test_observe_first_pick_plan_prints_object_place_and_grasp_poses(
    monkeypatch, capsys, legacy
):
    """The observe-first plan path also logs the recognised poses on stdout."""
    settings = with_place(
        target_settings(observe_first=True, observe_position_arm_m=(0.31, 0.02, 0.22))
    )
    rig = run_round(settings, monkeypatch, legacy_grasp=legacy.grasp)
    assert rig.result.status == "completed", rig.result.message
    out = capsys.readouterr().out
    arm_base = settings.frames.arm_base
    assert out.count("[rule_base] target object") == 1

    object_line = logged_line(out, f"[rule_base] target object {settings.frames.object} ")
    logged_values(object_line, f"center_{arm_base}", PICK_MOCK_OBJECT_POSE.position)
    logged_values(object_line, "orientation_xyzw", PICK_MOCK_OBJECT_POSE.orientation)
    place_line = logged_line(out, f"[rule_base] place {settings.frames.place} ")
    logged_values(place_line, f"center_{arm_base}", PICK_MOCK_PLACE_POSE.position)
    logged_values(place_line, "orientation_xyzw", PICK_MOCK_PLACE_POSE.orientation)
    assert rig.cycle.plan is not None
    grasp_line = logged_line(out, "[rule_base] grasp TCP=")
    logged_values(grasp_line, "TCP", rig.cycle.plan.grasp.position)
    logged_values(grasp_line, "pregrasp", rig.cycle.plan.pregrasp.position)
    assert grasp_line.endswith(f" ({arm_base})")


# -- pregrasp reachability fallback (D-130) -----------------------------
class PoseDeniedArm(FakePickArm):
    """Deny servo targets that match configured arm-frame positions (IK failure)."""

    def __init__(self, settings: PickSettings) -> None:
        super().__init__(settings)
        self.deny_positions: list[tuple[float, float, float]] = []

    def move(self, target: Pose) -> None:
        for position in self.deny_positions:
            if max(abs(a - b) for a, b in zip(target.position, position, strict=True)) < 1e-6:
                self.calls.append("servo_denied")
                raise ContractError(
                    "Arm plan_request denied: IK_FAILED: /compute_ik error_code=-31"
                )
        super().move(target)


def test_unreachable_pregrasp_falls_back_to_the_grasp_pose(monkeypatch, legacy):
    settings = target_settings()
    arm = PoseDeniedArm(settings)
    arm.deny_positions = [legacy.pregrasp.position]
    rig = run_round(
        settings, monkeypatch, legacy_grasp=legacy.grasp, arm_factory=lambda _s: arm
    )
    assert rig.result.status == "completed", rig.result.message
    assert "pregrasp_reduced" in rig.names
    event = named(rig, "pregrasp_reduced")[0]
    assert "IK_FAILED" in str(event["error"])
    # No pregrasp vision alignment after the fallback.
    assert "vision_pregrasp_align" not in [name for name, _fields in rig.events]


def test_unreachable_pregrasp_and_grasp_fail_without_grasp_recovery(monkeypatch, legacy):
    settings = target_settings()
    arm = PoseDeniedArm(settings)
    arm.deny_positions = [legacy.pregrasp.position, legacy.grasp.position]
    rig = run_round(
        settings, monkeypatch, legacy_grasp=legacy.grasp, arm_factory=lambda _s: arm
    )
    assert rig.result.status == "failed"
    assert "PICK_PLACE_UNREACHABLE" in rig.result.message
    assert "recovery_started" not in rig.names


def test_reduced_pregrasp_fallback_skips_the_pregrasp_verify(monkeypatch, legacy):
    """S6e：已在 grasp pose 的 reduced 回退没有独立 pregrasp 视角，跳过视觉校验。"""
    settings = target_settings(grasp_verify_enabled=True)
    arm = PoseDeniedArm(settings)
    arm.deny_positions = [legacy.pregrasp.position]
    rig = run_round(
        settings, monkeypatch, legacy_grasp=legacy.grasp, arm_factory=lambda _s: arm
    )
    assert rig.result.status == "completed", rig.result.message
    assert "pregrasp_reduced" in rig.names
    skipped = named(rig, "grasp_verify_skipped")
    assert skipped
    assert skipped[0]["stage"] == "pregrasp_verify"
    assert skipped[0]["reason"] == "reduced pregrasp fallback"
    assert not named(rig, "grasp_verify_ok")
    assert "pregrasp_verify" not in stages(rig.events)
    assert not [
        fields
        for name, fields in rig.events
        if name == "vision_refresh" and fields.get("stage") == "pregrasp_verify"
    ]


# -- observation sampling + target lock (D-132) -------------------------
def test_observe_first_samples_and_locks_the_target(monkeypatch, legacy):
    settings = target_settings(
        observe_first=True,
        lock_target_after_observation=True,
        observe_samples=3,
        observe_warmup_samples=0,
        observe_stability_m=0.008,
        observe_stability_rad=0.02,
    )
    updates = (
        legacy.grasp,
        moved(legacy.grasp, dx=0.002),
        moved(legacy.grasp, dx=-0.003),
    )
    rig = run_round(
        settings, monkeypatch, legacy_grasp=legacy.grasp, target_updates=updates
    )
    assert rig.result.status == "completed", rig.result.message
    stable = named(rig, "vision_observe_stable")[0]
    assert int(stable["samples"]) == 3
    assert float(stable["position_spread_m"]) <= 0.008
    assert named(rig, "vision_target_locked")
    stage_names = stages(rig.events)
    assert "vision_pregrasp_align" not in stage_names
    assert rig.environment.refresh_calls >= 3
    # The locked plan uses the sample median, not any single jittered frame.
    plan = rig.cycle.plan
    assert plan is not None
    assert plan.grasp.position == pytest.approx(legacy.grasp.position, abs=0.002)


def test_observe_first_rejects_unstable_samples_without_moving(monkeypatch, legacy):
    settings = target_settings(
        observe_first=True,
        lock_target_after_observation=True,
        observe_samples=3,
        observe_warmup_samples=0,
        observe_stability_m=0.008,
        observe_stability_rad=0.02,
    )
    updates = (
        legacy.grasp,
        moved(legacy.grasp, dx=0.05),
        legacy.grasp,
    )
    rig = run_round(
        settings, monkeypatch, legacy_grasp=legacy.grasp, target_updates=updates
    )
    assert rig.result.status == "failed"
    assert rig.result.stage == "vision_observe"
    assert "vision_observe_unstable" in rig.result.message
    stage_names = stages(rig.events)
    assert "pregrasp" not in stage_names and "open_gripper" not in stage_names
    # Commands stay at the servo hold pose or the fixed observation pose; nothing
    # was sent towards the (unstable) object.
    assert all(target.position[2] >= 0.19 for target in rig.arm.targets)


class ReprojectionEnvironment(FakePickEnvironment):
    """Return a scripted refresh payload so the sample quality gate can be tested."""

    def __init__(self, settings: PickSettings, messages: list[str]) -> None:
        super().__init__(settings, target_pose=None)
        self.messages = list(messages)

    def refresh_object_pose(self) -> str:
        message = super().refresh_object_pose()
        return self.messages.pop(0) if self.messages else message


def test_observe_rejects_high_reprojection_samples(monkeypatch, legacy):
    settings = target_settings(
        observe_first=True,
        lock_target_after_observation=True,
        observe_samples=3,
        observe_warmup_samples=0,
        observe_max_reprojection_px=1.5,
    )
    messages = ['{"reprojection_px": 2.5}', '{"reprojection_px": 0.3}', '{"reprojection_px": 0.4}']
    environment = ReprojectionEnvironment(settings, messages)
    events = record_events(monkeypatch, module)
    vision_events = record_events(monkeypatch, vision_servo)
    arm = FakePickArm(settings)
    cycle = PickRound(settings, environment, lambda: arm, Event())
    result = cycle.run("quality-gate")
    assert result.status == "completed", result.message
    rejected = [name for name, _fields in vision_events if name == "vision_observe_rejected"]
    assert rejected
    stable = [fields for name, fields in vision_events if name == "vision_observe_stable"][0]
    assert int(stable["samples"]) == 2
    assert any(name == "pick_plan" for name, _fields in vision_events)


# -- pregrasp verification (D-134/S6e) ---------------------------------
def test_grasp_verify_accepts_aligned_target(monkeypatch, legacy):
    settings = target_settings(
        observe_first=True,
        lock_target_after_observation=True,
        observe_samples=3,
        observe_warmup_samples=0,
        grasp_verify_enabled=True,
    )
    updates = tuple([legacy.grasp] * 4)  # 3 observation samples + 1 pregrasp verify
    rig = run_round(
        settings, monkeypatch, legacy_grasp=legacy.grasp, target_updates=updates
    )
    assert rig.result.status == "completed", rig.result.message
    verified = named(rig, "grasp_verify_ok")
    assert verified
    assert verified[0]["stage"] == "pregrasp_verify"
    stage_names = stages(rig.events)
    assert stage_names.index("pregrasp_verify") < stage_names.index("grasp_pose")
    # The pregrasp check is the last vision refresh: nothing re-detects after the
    # descent to the grasp pose (S6e: the close-range view is occluded/unreliable).
    names = [name for name, _fields in rig.events]
    grasp_pose_event = next(
        index
        for index, (name, fields) in enumerate(rig.events)
        if name == "stage" and fields.get("stage") == "grasp_pose"
    )
    assert "vision_refresh" not in names[grasp_pose_event:]
    # 3 observation samples + exactly one pregrasp verification, no second check.
    assert rig.environment.refresh_calls == 4


def test_grasp_verify_aborts_before_descending_when_object_moved(monkeypatch, legacy):
    settings = target_settings(
        observe_first=True,
        lock_target_after_observation=True,
        observe_samples=3,
        observe_warmup_samples=0,
        grasp_verify_enabled=True,
        grasp_verify_tolerance_m=0.02,
    )
    updates = (
        legacy.grasp,
        legacy.grasp,
        legacy.grasp,
        moved(legacy.grasp, dx=0.05),  # object pushed 5 cm while approaching pregrasp
    )
    rig = run_round(
        settings, monkeypatch, legacy_grasp=legacy.grasp, target_updates=updates
    )
    assert rig.result.status == "failed"
    assert rig.result.stage == "pregrasp_verify"
    assert "GRASP_TARGET_MISMATCH" in rig.result.message
    stage_names = stages(rig.events)
    # The mismatch aborts before the descent: no close-range move and no empty close.
    assert "grasp_pose" not in stage_names
    assert "close_gripper" not in stage_names
    assert "recovery_started" not in rig.names
