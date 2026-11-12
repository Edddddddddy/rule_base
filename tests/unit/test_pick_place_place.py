"""Purpose: Guard the TF-driven place stage, its execution chain and failures.

Usage: source ros2_ws/setup_ros_env.sh && python -m pytest tests/unit/test_pick_place_place.py
"""

import time
from dataclasses import replace
from threading import Event

import pytest

from pick_place_support import (
    PLACE_FRAME,
    logged_line,
    logged_values,
    offline_settings,
    record_events,
    recorded,
    stages,
    with_place,
)
from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import ContractError, PickScene, Pose
from rule_base.infrastructure.fakes.runtime import (
    PICK_MOCK_OBJECT_POSE,
    PICK_MOCK_PLACE_POSE,
    FakePickArm,
    FakePickEnvironment,
    tf_entry,
)
from rule_base.infrastructure.transform.observations import PickObservationCache
from rule_base.infrastructure.transform.math import compose, from_pose, offset, to_pose
from rule_base.stations.pick_place import execution as module
from rule_base.stations.pick_place.execution import PickRound
from rule_base.stations.pick_place.planning import plan_pick


@pytest.fixture
def settings():
    return offline_settings()


def test_pick_place_moves_to_approach_lowers_releases_and_retreats(settings, monkeypatch):
    settings = with_place(settings)
    environment = FakePickEnvironment(settings)
    events = record_events(monkeypatch, module)
    moves: list[tuple[str, Pose]] = []
    gripper_calls: list[tuple[str, Pose, float]] = []

    class RecordingArm(FakePickArm):
        def move(self, target):
            moves.append((cycle.stage, target))
            super().move(target)

        def gripper(self, width_m, effort):
            gripper_calls.append((cycle.stage, self.current.pose, width_m))
            super().gripper(width_m, effort)

    arm = RecordingArm(settings)
    cycle = PickRound(settings, environment, lambda: arm, Event())
    root_to_arm = environment.snapshot().root_to_arm
    result = cycle.run("place-chain")

    profile = settings.profile
    place = PICK_MOCK_PLACE_POSE
    approach = offset(place, (0.0, 0.0, profile.place_approach_clearance_m))
    post_grasp = Pose(profile.post_grasp_position_arm_m, profile.post_grasp_orientation_arm_xyzw)
    assert result.status == "completed", result
    assert result.stage == "home_end"
    assert result.message == "object grasped, moved to place pose, released"
    assert result.cleanup_confirmed

    all_stages = stages(events)
    assert "hold_object" not in all_stages
    assert all_stages[-6:] == [
        "move_to_post_grasp",
        "move_to_place_approach",
        "lower_to_place",
        "release_gripper",
        "retreat_from_place",
        "home_end",
    ]

    first: dict[str, int] = {}
    for index, (stage, _) in enumerate(moves):
        first.setdefault(stage, index)
    assert (
        first["grasp_pose"]
        < first["move_to_post_grasp"]
        < first["move_to_place_approach"]
        < first["lower_to_place"]
        < first["retreat_from_place"]
    )
    assert post_grasp in arm.targets
    approach_targets = [target for stage, target in moves if stage == "move_to_place_approach"]
    assert approach_targets[-1] == approach
    assert approach_targets[-1].position[:2] == pytest.approx(place.position[:2])
    assert approach_targets[-1].position[2] - place.position[2] == pytest.approx(
        profile.place_approach_clearance_m
    )
    lower_targets = [target for stage, target in moves if stage == "lower_to_place"]
    assert lower_targets[-1] == place

    # The jaws open at the place pose (not at the post-grasp pose) and only after
    # the approach/lower motions reached it.
    open_calls = [
        (stage, pose) for stage, pose, width in gripper_calls if width == profile.gripper_open_m
    ]
    assert open_calls[-1][0] == "release_gripper"
    assert open_calls[-1][1] == place

    # Retreat lifts along root +Z by place_retreat_clearance_m.
    retreat_targets = [target for stage, target in moves if stage == "retreat_from_place"]
    assert retreat_targets
    world_place = to_pose(compose(root_to_arm, from_pose(place)))
    world_retreat = to_pose(compose(root_to_arm, from_pose(retreat_targets[-1])))
    assert world_retreat.position[:2] == pytest.approx(world_place.position[:2])
    assert world_retreat.position[2] - world_place.position[2] == pytest.approx(
        profile.place_retreat_clearance_m
    )
    assert retreat_targets[-1].orientation == pytest.approx(place.orientation)

    # All place waypoints are linear motions; the descent starts where the approach ended.
    linear = recorded(events, "linear_motion")
    assert [fields["stage"] for fields in linear] == [
        "grasp_pose",
        "move_to_post_grasp",
        "move_to_place_approach",
        "lower_to_place",
        "retreat_from_place",
    ]
    assert linear[2]["target_pose_arm"]["position"] == pytest.approx(approach.position)
    assert linear[2]["target_pose_arm"]["orientation"] == pytest.approx(approach.orientation)
    assert linear[3]["start_pose_arm"]["position"] == pytest.approx(approach.position)
    assert linear[3]["target_pose_arm"]["position"] == pytest.approx(place.position)
    assert linear[4]["start_pose_arm"]["position"] == pytest.approx(place.position)
    assert arm.calls[-3:] == ["stop", "release", "close"]


def test_pick_place_pose_comes_from_tf_and_follows_the_published_pose(settings):
    settings = with_place(settings)
    shifted = Pose(
        (
            PICK_MOCK_PLACE_POSE.position[0] + 0.03,
            PICK_MOCK_PLACE_POSE.position[1] + 0.02,
            PICK_MOCK_PLACE_POSE.position[2],
        ),
        PICK_MOCK_PLACE_POSE.orientation,
    )

    class ShiftedPlaceEnvironment(FakePickEnvironment):
        def publish_object(self):
            super().publish_object()
            f = self.settings.frames
            assert f.place is not None
            entry = tf_entry(f.arm_base, f.place, shifted.position, timestamp_ns=time.time_ns())
            entry["transform"]["rotation"] = dict(zip("xyzw", shifted.orientation, strict=True))
            self.cache.update("tf", {"transforms": [entry]})

    environment = ShiftedPlaceEnvironment(settings)
    scene = environment.snapshot()
    assert scene.arm_to_place is not None
    plan = plan_pick(settings, scene)
    assert plan.place is not None
    assert plan.place.position == pytest.approx(shifted.position)
    assert plan.place.orientation == pytest.approx(shifted.orientation)

    arm = FakePickArm(settings)
    cycle = PickRound(settings, environment, lambda: arm, Event())
    result = cycle.run("shifted-place")
    assert result.status == "completed", result
    assert cycle.plan is not None and cycle.plan.place is not None
    assert cycle.plan.place in arm.targets


def test_pick_place_tf_missing_fails_before_constructing_the_arm(settings):
    settings = with_place(settings)
    scene = FakePickEnvironment(settings).snapshot()

    class MissingPlaceEnv:
        def snapshot(self):
            return PickScene(scene.root_to_arm, scene.arm_to_object)

    result = PickRound(
        settings, MissingPlaceEnv(), lambda: pytest.fail("Arm must not be constructed"), Event()
    ).run("missing-place-tf")
    assert result.status == "failed" and result.stage == "validate"
    assert "place TF is missing" in result.message
    assert result.cleanup_confirmed


def test_legacy_settings_ignore_a_present_place_tf(settings):
    scene = FakePickEnvironment(settings).snapshot()
    legacy_scene = PickScene(
        scene.root_to_arm, scene.arm_to_object, from_pose(PICK_MOCK_PLACE_POSE)
    )
    assert legacy_scene.arm_to_place is not None
    plan = plan_pick(settings, legacy_scene)
    assert not settings.place_enabled
    assert plan.place is None


def test_pick_place_failure_lifts_away_before_returning_and_opening(settings, monkeypatch):
    settings = with_place(settings)
    events = record_events(monkeypatch, module)
    gripper_calls: list[tuple[str, Pose, float]] = []

    class LostAtPlace(FakePickArm):
        lost = False

        def move(self, target):
            super().move(target)
            if cycle.stage == "lower_to_place" and not self.lost:
                self.lost = True
                self.current = replace(self.current, gripper_width_m=0.07)

        def gripper(self, width_m, effort):
            gripper_calls.append((cycle.stage, self.current.pose, width_m))
            super().gripper(width_m, effort)

    arm = LostAtPlace(settings)
    cycle = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event())
    result = cycle.run("lost-at-place")

    assert arm.lost
    assert result.status == "failed" and result.stage == "grasp_attempts_exhausted"
    failures = recorded(events, "grasp_attempt_failed")
    assert failures[0]["failed_stage"] == "lower_to_place"
    all_stages = stages(events)
    # A failure near the table retreats vertically before the lateral return.
    assert "retry_retreat" in all_stages
    assert all_stages.index("retry_retreat") < all_stages.index("retry_return_initial")
    # The jaws only open after the recovery returned to the guarded pose.
    open_calls = [
        (stage, pose)
        for stage, pose, width in gripper_calls
        if width == settings.profile.gripper_open_m
    ]
    assert open_calls[-1][0] == "retry_open_gripper"
    assert open_calls[-1][1].position == pytest.approx(settings.profile.post_grasp_position_arm_m)
    assert arm.calls[-3:] == ["stop", "release", "close"]
    assert result.cleanup_confirmed


def test_place_frame_is_required_for_a_complete_place_stage():
    assert not offline_settings().place_enabled
    enabled = with_place(offline_settings())
    assert enabled.place_enabled
    assert enabled.frames.place == PLACE_FRAME
    assert isinstance(enabled, PickSettings)


def test_thin_contact_outside_fixed_interval_completes_the_place_chain(settings, monkeypatch):
    """User field decision (2026-11-12): measured contact width confirms the grasp.

    A 0.033 m contact is below the old fixed interval [0.034, 0.055], but it is a
    real object between the jaws. The round must continue through the full place
    chain (lift -> approach -> lower -> release -> retreat) instead of failing the
    grasp and dropping the object during recovery; the fixed interval stays in the
    diagnostics and one ``grasp_width_outside_expected`` event records the deviation.
    """
    settings = with_place(settings)
    events = record_events(monkeypatch, module)
    arm = FakePickArm(settings, object_width_m=0.033)
    result = PickRound(settings, FakePickEnvironment(settings), lambda: arm, Event()).run(
        "thin-contact-place"
    )
    profile = settings.profile
    assert profile.grasp_min_width_m == 0.034
    assert result.status == "completed", result
    assert result.stage == "home_end"
    assert result.message == "object grasped, moved to place pose, released"
    all_stages = stages(events)
    assert all_stages[-5:] == [
        "move_to_place_approach",
        "lower_to_place",
        "release_gripper",
        "retreat_from_place",
        "home_end",
    ]
    samples = recorded(events, "grasp_feedback")
    assert samples[-1]["width_m"] == pytest.approx(0.033)
    assert samples[-1]["width_ok"] is True
    outside = recorded(events, "grasp_width_outside_expected")
    assert len(outside) == 1
    assert outside[0]["width_m"] == pytest.approx(0.033)
    assert outside[0]["stage"] == "close_gripper"
    assert arm.current.gripper_width_m == profile.gripper_open_m


def test_observation_cache_reads_place_tf_with_the_regular_age_gate(monkeypatch):
    settings = with_place(offline_settings())
    cache = PickObservationCache(settings)
    f = settings.frames
    assert f.place is not None
    now = 10_000_000_000
    monkeypatch.setattr(
        "rule_base.infrastructure.transform.buffer.time.time_ns", lambda: now
    )
    monkeypatch.setattr(
        "rule_base.infrastructure.transform.observations.time.time_ns", lambda: now
    )
    cache.update(
        "tf_static",
        {"transforms": [tf_entry(f.root, f.arm_base, (0, 0, 0), timestamp_ns=0)]},
    )
    cache.update(
        "tf",
        {
            "transforms": [
                tf_entry(f.arm_base, f.object, (0.4, 0, 0.3), timestamp_ns=now),
                tf_entry(f.arm_base, f.place, (0.4, 0.1, 0.2), timestamp_ns=now),
            ]
        },
    )
    scene = cache.snapshot()
    assert scene.arm_to_place is not None
    # Age only the place sample: the object stays fresh, the place lookup is gated.
    now += 400_000_000
    cache.update(
        "tf", {"transforms": [tf_entry(f.arm_base, f.object, (0.4, 0, 0.3), timestamp_ns=now)]}
    )
    with pytest.raises(ContractError, match="stale"):
        cache.snapshot()


def test_retry_needs_a_new_object_sample_but_reuses_a_fresh_place_sample(monkeypatch):
    settings = with_place(offline_settings())
    cache = PickObservationCache(settings)
    f = settings.frames
    assert f.place is not None
    now = 10_000_000_000
    monkeypatch.setattr(
        "rule_base.infrastructure.transform.buffer.time.time_ns", lambda: now
    )
    monkeypatch.setattr(
        "rule_base.infrastructure.transform.observations.time.time_ns", lambda: now
    )
    cache.update(
        "tf_static",
        {"transforms": [tf_entry(f.root, f.arm_base, (0, 0, 0), timestamp_ns=0)]},
    )
    cache.update(
        "tf",
        {
            "transforms": [
                tf_entry(f.arm_base, f.object, (0.4, 0, 0.3), timestamp_ns=now),
                tf_entry(f.arm_base, f.place, (0.4, 0.1, 0.2), timestamp_ns=now),
            ]
        },
    )
    cache.snapshot()
    now += 10_000_000
    cache.request_new_observation()
    # Only the object is republished; the place sample is not refreshed.
    cache.update(
        "tf", {"transforms": [tf_entry(f.arm_base, f.object, (0.41, 0, 0.3), timestamp_ns=now)]}
    )
    scene = cache.snapshot()
    assert scene.arm_to_place is not None
    assert scene.arm_to_object[0][3] == pytest.approx(0.41)


def test_legacy_pick_plan_prints_object_place_and_grasp_poses(settings, capsys):
    """Legacy (no observe_first) planning logs the recognised poses on stdout."""
    settings = with_place(settings)
    environment = FakePickEnvironment(settings)
    cycle = PickRound(settings, environment, lambda: FakePickArm(settings), Event())
    result = cycle.run("legacy-plan-log")
    assert result.status == "completed", result.message
    out = capsys.readouterr().out
    arm_base = settings.frames.arm_base

    object_line = logged_line(out, f"[rule_base] target object {settings.frames.object} ")
    logged_values(object_line, f"center_{arm_base}", PICK_MOCK_OBJECT_POSE.position)
    logged_values(object_line, "orientation_xyzw", PICK_MOCK_OBJECT_POSE.orientation)
    place_line = logged_line(out, f"[rule_base] place {PLACE_FRAME} ")
    logged_values(place_line, f"center_{arm_base}", PICK_MOCK_PLACE_POSE.position)
    logged_values(place_line, "orientation_xyzw", PICK_MOCK_PLACE_POSE.orientation)
    assert cycle.plan is not None
    grasp_line = logged_line(out, "[rule_base] grasp TCP=")
    logged_values(grasp_line, "TCP", cycle.plan.grasp.position)
    logged_values(grasp_line, "pregrasp", cycle.plan.pregrasp.position)
    assert grasp_line.endswith(f" ({arm_base})")
