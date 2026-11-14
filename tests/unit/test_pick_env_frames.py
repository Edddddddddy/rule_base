"""Purpose: Guard the env position-subscription mode (env_app TF object + tf_static place).

Additive third coordinate source (default off): the object id/pose comes from the
env node's dynamic ``env_marker_<id>`` TF plus its ``/tf_static`` ``place_target_01``
frame.  The pose is read once after the observation pose and reused for the round;
the grasp TCP is the marker position + profile contact offset (top-grasp reference
orientation), and no wrist vision / refresh service / station_support is involved.

Usage: source ros2_ws/setup_ros_env.sh && python3 -m pytest tests/unit/test_pick_env_frames.py
"""

from __future__ import annotations

import asyncio
import math
import time
from pathlib import Path
from threading import Event

import numpy as np
import pytest
from pydantic import ValidationError

from rule_base.application import runner as runner_module
from rule_base.config.load import load_settings
from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import ContractError, PickScene, Pose
from rule_base.infrastructure.fakes.runtime import (
    PICK_MOCK_OBJECT_POSE,
    PICK_MOCK_PLACE_POSE,
    PICK_MOCK_ROOT_TO_ARM,
    FakePickArm,
    FakePickEnvironment,
    FakeTransport,
    tf_entry,
)
from rule_base.infrastructure.transform.math import IDENTITY, from_pose, inverse, to_pose
from rule_base.infrastructure.transform.observations import PickObservationCache
from rule_base.stations.pick_place import env_snapshot as env_module
from rule_base.stations.pick_place import execution as execution_module
from rule_base.stations.pick_place.execution import PickRound
from rule_base.stations.pick_place.planning import (
    format_pick_plan_log,
    pick_plan_fields,
    plan_pick,
)

ROOT = Path(__file__).resolve().parents[2]
ENV_CONFIG = ROOT / "configs/pick_place/live_ros2_env.yaml"
TEACH_CONFIG = ROOT / "configs/pick_place/live_ros2_teach.yaml"
ENV_TARGET_ID = "RULE_BASE_TARGET_OBJECT_ID"


def env_settings(**env_updates: object) -> PickSettings:
    """Env-mode settings with a deterministic (fast) offline observation."""
    settings = load_settings(ENV_CONFIG)
    updates: dict[str, object] = {"observe_settle_s": 0.0}
    updates.update(env_updates)
    return settings.model_copy(
        update={
            "env_frames": settings.env_frames.model_copy(update=updates),
            "limits": settings.limits.model_copy(
                update={
                    "max_grasp_attempts": 1,
                    "tf_age_s": 10.0,
                    "stable_time_s": 0.02,
                    "motion_stable_time_s": 0.01,
                    "servo_hz": 50.0,
                    "linear_step_m": 0.05,
                    "linear_speed_m_s": 2.0,
                }
            ),
        }
    )


def values(**updates: object) -> dict[str, object]:
    """Validated env config as a mutable mapping (for negative validation tests)."""
    raw = load_settings(ENV_CONFIG).model_dump()
    for path, value in updates.items():
        section, _, key = path.partition("__")
        target = raw if not key else raw[section]
        target[key if key else section] = value
    return raw


def scene_with(
    marker: Pose,
    place: Pose,
    *,
    root_to_arm: object = IDENTITY,
) -> PickScene:
    return PickScene(
        root_to_arm,  # type: ignore[arg-type]
        from_pose(marker),
        from_pose(place),
    )


DEFAULT_MARKER = Pose((0.25, 0.02, -0.01), (0.0, 0.0, 0.0, 1.0))
DEFAULT_PLACE = Pose((0.25, 0.15, -0.01), (0.7071067811865476, 0.0, -0.7071067811865476, 0.0))


# -- config parsing / validation ----------------------------------------


def test_env_config_loads_and_exposes_the_env_frames() -> None:
    settings = load_settings(ENV_CONFIG)
    assert settings.env_frames.enabled is True
    assert settings.frames.object is None
    assert settings.frames.place is None
    assert settings.frames.grasp_target is None
    assert settings.object_frame == "env_marker_1"
    assert settings.place_frame == "place_target_01"
    assert settings.grasp_target_frame is None
    assert settings.place_enabled is True
    assert settings.vision.enabled is False
    assert settings.arm.manage_lease is False


def test_env_target_id_environment_override(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(ENV_TARGET_ID, " 7 ")
    settings = load_settings(ENV_CONFIG)
    assert settings.env_frames.target_object_id == "7"
    assert settings.object_frame == "env_marker_7"


def test_env_target_id_override_rejects_empty(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(ENV_TARGET_ID, "   ")
    with pytest.raises(ValueError, match=ENV_TARGET_ID):
        load_settings(ENV_CONFIG)


def test_env_target_id_override_is_ignored_for_other_modes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv(ENV_TARGET_ID, "9")
    settings = load_settings(TEACH_CONFIG)
    assert settings.env_frames.enabled is False
    assert settings.frames.grasp_target == "grasp_target_01"


@pytest.mark.parametrize("target_id", ["", "a b", "/1", "1/2", "-1"])
def test_env_target_id_rejects_invalid_ids(target_id: str) -> None:
    with pytest.raises(ValidationError, match="target_object_id"):
        PickSettings.model_validate(values(env_frames__target_object_id=target_id))


def test_env_target_id_accepts_numeric_and_underscore_tokens() -> None:
    for target_id in ("1", "42", "cube_2A"):
        settings = PickSettings.model_validate(values(env_frames__target_object_id=target_id))
        assert settings.object_frame == f"env_marker_{target_id}"


@pytest.mark.parametrize(
    ("path", "value", "message"),
    [
        ("frames__object", "objects_cube_01", "frames.object must be omitted"),
        ("frames__grasp_target", "grasp_target_01", "frames.grasp_target must be omitted"),
        ("frames__place", "place_target_01", "frames.place must be omitted"),
        ("vision__enabled", True, "mutually exclusive"),
        ("env_frames__place_frame", "/place_target_01", "place_frame"),
        ("env_frames__object_tf_prefix", "env marker ", "object_tf_prefix"),
    ],
)
def test_env_mode_conflicting_or_invalid_settings_are_rejected(
    path: str, value: object, message: str
) -> None:
    with pytest.raises(ValidationError, match=message):
        PickSettings.model_validate(values(**{path: value}))


def test_env_mode_requires_an_object_frame_when_disabled() -> None:
    raw = values()
    raw["env_frames"]["enabled"] = False  # type: ignore[index]
    raw["env_frames"]["observe_joint_positions"] = None  # type: ignore[index]
    raw["frames"]["object"] = None  # type: ignore[index]
    with pytest.raises(ValidationError, match="frames.object is required"):
        PickSettings.model_validate(raw)


def test_env_observe_joints_are_validated() -> None:
    raw = values()
    env = raw["env_frames"]
    assert isinstance(env, dict)
    env["observe_joint_positions"] = [-0.046, 0.0491, -0.3175, 1.6536]  # wrong length
    with pytest.raises(ValidationError, match="observe_joint_positions"):
        PickSettings.model_validate(raw)
    raw = values()
    env = raw["env_frames"]
    assert isinstance(env, dict)
    env["observe_joint_positions"] = [0.0, 9.9, 0.0, 0.0, 0.0]  # outside hardware limit
    with pytest.raises(ValidationError, match="hardware limit"):
        PickSettings.model_validate(raw)


def test_env_observe_pose_and_quaternion_are_validated() -> None:
    with pytest.raises(ValidationError, match="observe_orientation_arm_xyzw"):
        PickSettings.model_validate(
            values(env_frames__observe_orientation_arm_xyzw=[0.0, 0.0, 0.0, 2.0])
        )
    with pytest.raises(ValidationError, match="observe_position_arm_m"):
        PickSettings.model_validate(values(env_frames__observe_position_arm_m=[0.0, math.nan, 0.0]))


def test_env_mode_is_allowed_for_live_execution() -> None:
    """D-142: env mode consumes live TF, so it must pass the live-source gate."""
    runner_module.validate_live_grasp_source(load_settings(ENV_CONFIG))
    with pytest.raises(ValueError, match="grasp_target"):
        runner_module.validate_live_grasp_source(
            load_settings(ROOT / "configs/pick_place/live_ros2.yaml")
        )


# -- grasp geometry from the env marker ---------------------------------


def test_env_plan_uses_marker_position_and_the_top_grasp_reference() -> None:
    settings = env_settings(grasp_offset_arm_m=(0.0, 0.0, 0.0))
    scene = scene_with(DEFAULT_MARKER, DEFAULT_PLACE)
    plan = plan_pick(settings, scene)
    assert plan.grasp.position == pytest.approx(DEFAULT_MARKER.position)
    reference_rotation = np.asarray(
        from_pose(Pose((0, 0, 0), settings.profile.reference_tcp_orientation_root_xyzw))
    )[:3, :3]
    assert np.allclose(np.asarray(from_pose(plan.grasp))[:3, :3], reference_rotation)
    assert plan.pregrasp.orientation == pytest.approx(plan.grasp.orientation)
    # real_pick contact offset is zero -> the grasp TCP is the marker position.
    assert settings.profile.grasp_contact_offset_tcp_m == (0.0, 0.0, 0.0)
    assert plan.place is not None
    assert plan.place.position == pytest.approx(DEFAULT_PLACE.position)
    assert np.allclose(
        np.asarray(from_pose(plan.place))[:3, :3],
        np.asarray(from_pose(DEFAULT_PLACE))[:3, :3],
    )
    # pregrasp = grasp + root +Z * clearance, keeping the fixed top-grasp orientation.
    clearance = settings.profile.pregrasp_clearance_m
    assert plan.pregrasp.position == pytest.approx(
        tuple(
            value + delta
            for value, delta in zip(DEFAULT_MARKER.position, (0, 0, clearance), strict=True)
        )
    )
    assert plan.pregrasp.orientation == pytest.approx(plan.grasp.orientation)
    # Width: calibrated cube interval is kept as-is (no face projection).
    profile = settings.profile
    assert plan.expected_cube_width_m == pytest.approx(profile.cube_side_m)
    assert plan.grasp_min_width_m == pytest.approx(profile.grasp_min_width_m)
    assert plan.grasp_max_width_m == pytest.approx(profile.grasp_max_width_m)


def test_env_plan_ignores_the_marker_rotation() -> None:
    settings = load_settings(ENV_CONFIG)
    rotated = Pose(DEFAULT_MARKER.position, (0.0, 0.0, 0.2588190451, 0.9659258263))
    plan_identity = plan_pick(settings, scene_with(DEFAULT_MARKER, DEFAULT_PLACE))
    plan_rotated = plan_pick(settings, scene_with(rotated, DEFAULT_PLACE))
    assert plan_rotated.grasp.position == pytest.approx(plan_identity.grasp.position)
    assert plan_rotated.grasp.orientation == pytest.approx(plan_identity.grasp.orientation)
    assert plan_rotated.observation == plan_identity.observation


def test_env_plan_composes_the_tcp_contact_offset_in_the_tcp_frame() -> None:
    settings = env_settings(grasp_offset_arm_m=(0.0, 0.0, 0.0))
    offset = (0.02, 0.01, 0.0)
    settings = settings.model_copy(
        update={
            "profile": settings.profile.model_copy(update={"grasp_contact_offset_tcp_m": offset})
        }
    )
    plan = plan_pick(settings, scene_with(DEFAULT_MARKER, DEFAULT_PLACE))
    reference = np.asarray(
        from_pose(Pose((0, 0, 0), settings.profile.reference_tcp_orientation_root_xyzw))
    )[:3, :3]
    expected = np.asarray(DEFAULT_MARKER.position) + reference @ np.asarray(offset)
    assert plan.grasp.position == pytest.approx(tuple(expected))
    reference_rotation = np.asarray(
        from_pose(Pose((0, 0, 0), settings.profile.reference_tcp_orientation_root_xyzw))
    )[:3, :3]
    assert np.allclose(np.asarray(from_pose(plan.grasp))[:3, :3], reference_rotation)


def test_env_plan_converts_root_poses_to_the_arm_command_frame() -> None:
    settings = env_settings(grasp_offset_arm_m=(0.0, 0.0, 0.0))
    settings = settings.model_copy(
        update={
            "profile": settings.profile.model_copy(
                update={
                    "workspace_min_m": (-1.0, -1.0, -1.0),
                    "workspace_max_m": (1.0, 1.0, 1.0),
                }
            )
        }
    )
    marker = Pose(PICK_MOCK_OBJECT_POSE.position, (0.0, 0.0, 0.0, 1.0))
    place = Pose(PICK_MOCK_PLACE_POSE.position, (0.0, 0.0, 0.0, 1.0))
    scene = scene_with(marker, place, root_to_arm=from_pose(PICK_MOCK_ROOT_TO_ARM))
    plan = plan_pick(settings, scene)
    root_to_arm = np.asarray(scene.root_to_arm)
    arm_to_root = np.asarray(inverse(scene.root_to_arm))
    marker_root = root_to_arm[:3, :3] @ np.asarray(marker.position) + root_to_arm[:3, 3]
    expected_arm = arm_to_root[:3, :3] @ marker_root + arm_to_root[:3, 3]
    assert plan.grasp.position == pytest.approx(tuple(expected_arm))
    expected_rotation = (
        arm_to_root[:3, :3]
        @ np.asarray(
            from_pose(Pose((0, 0, 0), settings.profile.reference_tcp_orientation_root_xyzw))
        )[:3, :3]
    )
    actual_rotation = np.asarray(from_pose(plan.grasp))[:3, :3]
    assert np.allclose(actual_rotation, expected_rotation, atol=1e-9)


def test_env_plan_applies_the_field_calibrated_grasp_offset_in_the_arm_frame() -> None:
    settings = load_settings(ENV_CONFIG)
    grasp_offset = tuple(settings.env_frames.grasp_offset_arm_m)
    assert grasp_offset == pytest.approx((0.0014, 0.0090, -0.0114))
    plan = plan_pick(settings, scene_with(DEFAULT_MARKER, DEFAULT_PLACE))
    assert plan.grasp.position == pytest.approx(
        tuple(value + delta for value, delta in zip(DEFAULT_MARKER.position, grasp_offset, strict=True))
    )
    # Offset shifts the pregrasp alongside the grasp but keeps the top-grasp orientation.
    assert plan.pregrasp.position == pytest.approx(
        tuple(
            value + delta + (0.0, 0.0, settings.profile.pregrasp_clearance_m)[index]
            for index, (value, delta) in enumerate(zip(DEFAULT_MARKER.position, grasp_offset, strict=True))
        )
    )
    reference_rotation = np.asarray(
        from_pose(Pose((0, 0, 0), settings.profile.reference_tcp_orientation_root_xyzw))
    )[:3, :3]
    assert np.allclose(np.asarray(from_pose(plan.grasp))[:3, :3], reference_rotation)


def test_env_grasp_offset_validation() -> None:
    with pytest.raises(ValidationError, match="grasp_offset_arm_m"):
        PickSettings.model_validate(values(env_frames__grasp_offset_arm_m=[0.0, 0.0, 0.5]))
    with pytest.raises(ValidationError, match="grasp_offset_arm_m"):
        PickSettings.model_validate(
            values(env_frames__grasp_offset_arm_m=[0.0, math.nan, 0.0])
        )


def test_env_plan_requires_the_place_frame() -> None:
    settings = load_settings(ENV_CONFIG)
    scene = PickScene(IDENTITY, from_pose(DEFAULT_MARKER), None)
    with pytest.raises(ContractError, match="place TF is missing"):
        plan_pick(settings, scene)
    with pytest.raises(ContractError, match="env object TF is missing"):
        plan_pick(settings, PickScene(IDENTITY, None, from_pose(DEFAULT_PLACE)))  # type: ignore[arg-type]


def test_env_plan_fields_and_log_name_the_env_frames() -> None:
    settings = load_settings(ENV_CONFIG)
    scene = scene_with(DEFAULT_MARKER, DEFAULT_PLACE)
    plan = plan_pick(settings, scene)
    fields = pick_plan_fields(settings, scene, plan)
    assert fields["object_frame"] == "env_marker_1"
    assert fields["place_frame"] == "place_target_01"
    assert fields["target_closing_axis_flipped"] is False
    log = format_pick_plan_log(settings, scene, plan)
    assert "env_marker_1" in log
    assert "place_target_01" in log


# -- observation cache snapshot (dynamic marker + tf_static place) -------


def _publish_static(cache: PickObservationCache, settings: PickSettings) -> None:
    frames = settings.frames
    root_arm = tf_entry(
        frames.root, frames.arm_base, PICK_MOCK_ROOT_TO_ARM.position, timestamp_ns=0
    )
    root_arm["transform"]["rotation"] = dict(
        zip("xyzw", PICK_MOCK_ROOT_TO_ARM.orientation, strict=True)
    )
    cache.update("tf_static", {"transforms": [root_arm]})


def _marker_entry(settings: PickSettings, *, timestamp_ns: int | None = None) -> dict[str, object]:
    timestamp_ns = time.time_ns() if timestamp_ns is None else timestamp_ns
    frames = settings.frames
    return tf_entry(
        frames.arm_base,
        "env_marker_1",
        PICK_MOCK_OBJECT_POSE.position,
        timestamp_ns=timestamp_ns,
    )


def _place_static_entry(settings: PickSettings) -> dict[str, object]:
    frames = settings.frames
    return tf_entry(
        frames.arm_base,
        "place_target_01",
        PICK_MOCK_PLACE_POSE.position,
        timestamp_ns=0,
    )


def test_cache_snapshot_reads_env_marker_and_static_place() -> None:
    settings = load_settings(ENV_CONFIG)
    cache = PickObservationCache(settings)
    _publish_static(cache, settings)
    cache.update("tf_static", {"transforms": [_place_static_entry(settings)]})
    with pytest.raises(ContractError, match="env_marker_1"):
        cache.snapshot()
    cache.update("tf", {"transforms": [_marker_entry(settings)]})
    scene = cache.snapshot()
    assert to_pose(scene.arm_to_object).position == pytest.approx(PICK_MOCK_OBJECT_POSE.position)
    assert scene.arm_to_place is not None
    assert scene.arm_to_target is None


def test_cache_snapshot_requires_the_static_place_frame() -> None:
    settings = load_settings(ENV_CONFIG)
    cache = PickObservationCache(settings)
    _publish_static(cache, settings)
    cache.update("tf", {"transforms": [_marker_entry(settings)]})
    with pytest.raises(ContractError, match="place_target_01"):
        cache.snapshot()


# -- one-snapshot round flow --------------------------------------------


class FlakySnapshotEnvironment(FakePickEnvironment):
    """Fails the first N snapshots (missing env marker) then delegates."""

    def __init__(self, settings: PickSettings, failures: int) -> None:
        super().__init__(settings)
        self.failures = failures
        self.snapshot_calls = 0

    def snapshot(self) -> PickScene:
        self.snapshot_calls += 1
        if self.failures:
            self.failures -= 1
            raise ContractError("Pick object TF is missing: base_link <- env_marker_1")
        return super().snapshot()


def record_env_events(monkeypatch: pytest.MonkeyPatch) -> list[tuple[str, dict[str, object]]]:
    events: list[tuple[str, dict[str, object]]] = []
    sink = lambda name, **fields: events.append((name, fields))  # noqa: E731
    monkeypatch.setattr(execution_module, "event", sink)
    monkeypatch.setattr(env_module, "event", sink)
    return events


def test_env_round_observes_then_takes_exactly_one_snapshot(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    settings = env_settings(grasp_offset_arm_m=(0.0, 0.0, 0.0))
    events = record_env_events(monkeypatch)
    environment = FakePickEnvironment(settings)
    arm = FakePickArm(settings)
    cycle = PickRound(settings, environment, lambda: arm, Event())
    result = cycle.run("env-round")

    assert result.status == "completed", result.message
    assert result.stage == "home_end" and result.cleanup_confirmed
    assert cycle.plan is not None
    assert cycle.plan.grasp.position == pytest.approx(PICK_MOCK_OBJECT_POSE.position)
    names = [name for name, _fields in events]
    stages = [str(fields["stage"]) for name, fields in events if name == "stage"]
    # The observation move happens before the snapshot; the plan is emitted once.
    assert "env_observe_joints" in stages
    assert stages.index("env_observe_joints") < stages.index("env_snapshot")
    assert names.count("env_snapshot") == 1
    assert names.count("pick_plan") == 1
    assert not [name for name in names if name.startswith("vision_")]
    snapshot_event = next(fields for name, fields in events if name == "env_snapshot")
    assert snapshot_event["target_object_id"] == "1"
    assert snapshot_event["object_frame"] == "env_marker_1"
    assert snapshot_event["place_frame"] == "place_target_01"
    assert snapshot_event["place_pose_arm"] is not None
    plan_event = next(fields for name, fields in events if name == "pick_plan")
    assert plan_event["grasp_pose_arm"]["position"] == pytest.approx(PICK_MOCK_OBJECT_POSE.position)


def test_env_round_retries_the_snapshot_until_tf_is_available(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    settings = env_settings()
    events = record_env_events(monkeypatch)
    environment = FlakySnapshotEnvironment(settings, failures=2)
    cycle = PickRound(settings, environment, lambda: FakePickArm(settings), Event())
    result = cycle.run("env-retry")
    assert result.status == "completed", result.message
    wait_events = [name for name, _fields in events if name == "env_snapshot_wait"]
    assert len(wait_events) >= 1
    assert environment.snapshot_calls >= 3


def test_env_round_fails_explicitly_when_the_snapshot_times_out(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    settings = env_settings(snapshot_timeout_s=0.1)
    events = record_env_events(monkeypatch)
    environment = FakePickEnvironment(settings)
    environment.snapshot = lambda: (_ for _ in ()).throw(  # type: ignore[method-assign]
        ContractError("Pick object TF is missing: base_link <- env_marker_1")
    )
    cycle = PickRound(settings, environment, lambda: FakePickArm(settings), Event())
    result = cycle.run("env-timeout")
    assert result.status == "failed"
    assert result.stage == "env_snapshot"
    assert "env snapshot timed out" in result.message
    assert result.cleanup_confirmed is True
    round_result = next(fields for name, fields in events if name == "round_result")
    assert round_result["status"] == "failed"


# -- warm-up gate (robot chain only) ------------------------------------


def test_env_mode_warmup_does_not_require_the_object_tf() -> None:
    settings = load_settings(ENV_CONFIG)
    environment = FakePickEnvironment(settings)
    environment.snapshot = lambda: (_ for _ in ()).throw(  # type: ignore[method-assign]
        ContractError("Pick object TF is missing: base_link <- env_marker_1")
    )
    assert environment.robot_ready() is True
    rounds: list[str] = []
    runner = runner_module.InstanceRunner(
        FakeTransport(),
        environment,
        lambda round_id: (rounds.append(round_id), _completed())[1],
        Event(),
        warmup_timeout_s=0.01,
        robot_only_warmup=True,
    )
    assert asyncio.run(runner.run()) == 0
    assert len(rounds) == 1


def _completed() -> object:
    from rule_base.domain.models import RoundResult

    return RoundResult("1", "completed", "test", "test result", 0.02, 0.02, True)
