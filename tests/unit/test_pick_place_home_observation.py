"""Purpose: Guard the D-094 observation view, per-round home and home diagnostics.

Usage: source ros2_ws/setup_ros_env.sh
       python3 -m pytest tests/unit/test_pick_place_home_observation.py
"""

from dataclasses import replace
from threading import Event
from types import SimpleNamespace

import pytest

from pick_place_support import (
    offline_settings,
    record_events,
    recorded,
    stages,
    with_home,
)
from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import ContractError, PickScene, Pose
from rule_base.infrastructure.fakes.runtime import FakePickArm, FakePickEnvironment
from rule_base.infrastructure.transform.math import compose, from_pose, inverse, to_pose
from rule_base.stations.pick_place import execution as module
from rule_base.stations.pick_place.execution import PickRound
from rule_base.stations.pick_place.planning import plan_pick

REAL_REST = (0.0, -1.52, 1.45, 0.75, 0.0, 0.0)


@pytest.fixture
def settings():
    return offline_settings()


def test_pick_moves_to_observation_before_opening_gripper(settings, monkeypatch):
    events = record_events(monkeypatch, module)
    moves: list[tuple[str, Pose]] = []

    class RecordingArm(FakePickArm):
        def start_servo(self, hold_pose):
            # Mirror the real adapter: confirming Servo does not plan a pose.
            if not self.owned:
                raise ContractError("fake Pick Arm has no control")
            self.calls.append("start_servo")
            self.servo_ready = True

        def move(self, target):
            moves.append((cycle.stage, target))
            super().move(target)

    arm = RecordingArm(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event())
    result = cycle.run("observation-first")
    assert result.status == "completed", result

    all_stages = stages(events)
    assert (
        all_stages.index("connect")
        < all_stages.index("acquire_control")
        < all_stages.index("home_start")
        < all_stages.index("initialize_servo")
        < all_stages.index("move_to_observation")
        < all_stages.index("open_gripper")
        < all_stages.index("pregrasp")
        < all_stages.index("grasp_pose")
    )
    # The first Cartesian target of the attempt is the observation pose; the descent
    # still starts from the pregrasp pose, which is distinct from the observation.
    assert moves[0] == ("move_to_observation", cycle.plan.observation)
    assert cycle.plan.observation != cycle.plan.pregrasp
    observation_index = next(
        index for index, (stage, _) in enumerate(moves) if stage == "move_to_observation"
    )
    assert observation_index == 0


def test_observation_pose_combines_root_offset_and_clearance(settings):
    settings = settings.model_copy(
        update={
            "profile": settings.profile.model_copy(
                update={
                    "observation_clearance_m": 0.12,
                    "observation_offset_root_m": (0.01, -0.02, 0.0),
                }
            )
        }
    )
    scene = FakePickEnvironment(settings).snapshot()
    plan = plan_pick(settings, scene)
    root_grasp = to_pose(compose(scene.root_to_arm, from_pose(plan.grasp)))
    offset = settings.profile.observation_offset_root_m
    expected_root = Pose(
        (
            root_grasp.position[0] + offset[0],
            root_grasp.position[1] + offset[1],
            root_grasp.position[2] + offset[2] + settings.profile.observation_clearance_m,
        ),
        settings.profile.reference_tcp_orientation_root_xyzw,
    )
    expected_arm = to_pose(compose(inverse(scene.root_to_arm), from_pose(expected_root)))
    assert plan.observation == expected_arm


def test_observation_outside_the_workspace_is_rejected(settings):
    values = settings.model_dump()
    # The observation pose sits 6 cm above the grasp TCP; cap the box below it while
    # keeping the grasp and the configured post-grasp pose inside.
    values["profile"]["workspace_max_m"] = [0.85, 0.3, 0.29]
    values["profile"]["post_grasp_position_arm_m"] = [0.31, 0.0, 0.2]
    settings = PickSettings.model_validate(values)
    scene = PickScene(
        from_pose(Pose((0, 0, 0), (0, 0, 0, 1))),
        from_pose(Pose((0.4, 0, 0.2), (0, 0, 0, 1))),
    )
    with pytest.raises(ContractError, match="workspace"):
        plan_pick(settings, scene)


def test_pick_home_wraps_the_round_with_the_configured_rest(settings, monkeypatch):
    settings = with_home(settings, REAL_REST)
    events = record_events(monkeypatch, module)
    arm = FakePickArm(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event())
    result = cycle.run("home-round")
    assert result.status == "completed", result
    assert result.stage == "home_end"
    all_stages = stages(events)
    assert all_stages[-1] == "home_end"
    assert all_stages.count("home_start") == all_stages.count("home_end") == 1
    completions = recorded(events, "home_complete")
    assert [fields["stage"] for fields in completions] == ["home_start", "home_end"]
    for fields in completions:
        assert fields["elapsed_s"] >= 0
    # Home streams five arm joints plus the measured gripper angle.
    assert arm.joint_commands
    for command in arm.joint_commands:
        assert len(command) == len(settings.ros2.joint_names)
        assert command[:5] == pytest.approx(REAL_REST[:5])


def test_pick_home_commands_keep_the_measured_gripper_angle(settings):
    settings = with_home(settings, REAL_REST)
    gripper_angle = 0.42
    gripper_calls: list[tuple[str, float]] = []

    class MeasuredGripperArm(FakePickArm):
        def gripper(self, width_m, effort):
            gripper_calls.append((cycle.stage, width_m))
            super().gripper(width_m, effort)

    arm = MeasuredGripperArm(settings)
    arm.current = replace(
        arm.current, joint_positions=(0.0, 0.0, 0.0, 0.0, 0.0, gripper_angle)
    )
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event())
    result = cycle.run("home-gripper-hold")
    assert result.status == "completed", result
    # Home must never command the configured gripper width; the measured angle is
    # forwarded unchanged in the 6th slot of every streamed joint command.
    assert gripper_calls
    assert not [stage for stage, _ in gripper_calls if stage in {"home_start", "home_end"}]
    assert all(command[-1] == pytest.approx(gripper_angle) for command in arm.joint_commands)


def test_pick_home_already_at_rest_returns_quickly(settings, monkeypatch):
    now = 1.0

    class AdvancingStop(Event):
        def wait(self, timeout=None):
            nonlocal now
            now += timeout or 0
            return self.is_set()

    events = record_events(monkeypatch, module)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: now))
    arm = FakePickArm(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, AdvancingStop())
    result = cycle.run("home-quick")
    assert result.status == "completed", result
    completions = recorded(events, "home_complete")
    assert len(completions) == 2
    servo_period = 1 / settings.limits.servo_hz
    assert all(
        fields["elapsed_s"] <= settings.limits.motion_stable_time_s + 2 * servo_period
        for fields in completions
    )


def test_pick_home_timeout_fails_the_round_without_grasp_retry(settings, monkeypatch):
    now = 1.0
    settings = with_home(settings, REAL_REST)
    settings = settings.model_copy(
        update={
            "limits": settings.limits.model_copy(
                update={"home_timeout_s": 0.5, "max_grasp_attempts": 3}
            )
        }
    )

    class AdvancingStop(Event):
        def wait(self, timeout=None):
            nonlocal now
            now += timeout or 0
            return self.is_set()

    events = record_events(monkeypatch, module)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: now))

    class StuckJointArm(FakePickArm):
        def move_joints(self, positions):
            assert len(positions) == len(self.settings.ros2.joint_names)
            self.calls.append("move_joints")
            self.joint_commands.append(tuple(positions))

    arm = StuckJointArm(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, AdvancingStop())
    result = cycle.run("home-timeout")
    assert result.status == "failed" and result.stage == "home_start"
    assert "stage timed out: home_start" in result.message
    assert "home_feedback=" in result.message
    assert arm.calls[-3:] == ["stop", "release", "close"]
    assert result.cleanup_confirmed
    # A blocked home is not retried as a grasp attempt and sends no Cartesian motion.
    assert arm.calls.count("connect") == 1
    assert "servo" not in arm.calls
    assert "grasp_attempt_failed" not in [name for name, _ in events]
    assert "home_start" in [fields["stage"] for fields in recorded(events, "home_feedback")]


def test_pick_home_does_not_apply_the_tcp_workspace_gate(settings):
    # The arm starts far outside the Pick TCP workspace (e.g. folded rest) and the
    # per-round home/move-to-observation stages must not gate on the grasp box.
    settings = with_home(settings, REAL_REST)
    arm = FakePickArm(settings)
    arm.current = replace(arm.current, pose=Pose((2.0, 0.0, 2.0), (0, 0, 0, 1)))
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event())
    result = cycle.run("home-outside-workspace")
    assert result.status == "completed", result
    assert cycle.plan is not None
    assert result.stage == "home_end"


def test_initialize_servo_does_not_replan_the_measured_pose(settings, monkeypatch):
    settings = with_home(settings, REAL_REST)
    moves: list[str] = []

    class ServoOnlyArm(FakePickArm):
        def start_servo(self, hold_pose):
            if not self.owned:
                raise ContractError("fake Pick Arm has no control")
            self.calls.append("start_servo")
            self.servo_ready = True

        def move(self, target):
            moves.append(cycle.stage)
            super().move(target)

    arm = ServoOnlyArm(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event())
    result = cycle.run("servo-init")
    assert result.status == "completed", result
    assert arm.calls.count("start_servo") == 1
    assert "initialize_servo" not in moves
    assert moves[0] == "move_to_observation"
