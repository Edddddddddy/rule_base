"""Purpose: Verify horizontal cube alignment, vertical approach and tool compensation.

Usage: uv run --no-sync python -m pytest tests/unit/test_pick_geometry.py
"""

from dataclasses import asdict
from itertools import permutations, product
from pathlib import Path

import numpy as np
import pytest
from pick_place_support import logged_values

from rule_base.config.load import load_settings
from rule_base.domain.models import ContractError, PickScene, Pose
from rule_base.infrastructure.transform.math import compose, from_pose, inverse, matrix, to_pose
from rule_base.stations.pick_place.planning import (
    format_pick_plan_log,
    pick_plan_fields,
    plan_pick,
)

CONFIG = Path(__file__).resolve().parents[2] / "configs/pick_place/offline.yaml"
PLACE_FRAME = "place_target_01"


def with_place_frame(settings):
    values = settings.model_dump()
    values["frames"]["place"] = PLACE_FRAME
    return type(settings).model_validate(values)


def place_scene(*, place_position=(0.4, 0.0, 0.2), place_quaternion=None):
    """Scene whose place TF is a valid top-grasp pose in the root frame."""
    reference = (0.0, 0.7071067811865476, 0.0, 0.7071067811865476)
    return PickScene(
        from_pose(Pose((0, 0, 0), (0, 0, 0, 1))),
        from_pose(Pose((0.5, 0.0, 0.3), (0, 0, 0, 1))),
        from_pose(Pose(place_position, place_quaternion or reference)),
    )


def pose_matrix(position=(0, 0, 0), quaternion=(0, 0, 0, 1)):
    return np.asarray(from_pose(Pose(position, quaternion)))


@pytest.fixture
def settings():
    values = load_settings(CONFIG).model_dump()
    # Compare the same world geometry under arbitrary command bases.
    values["profile"]["workspace_min_m"] = [-5, -5, -5]
    values["profile"]["workspace_max_m"] = [5, 5, 5]
    return type(load_settings(CONFIG)).model_validate(values)


def scene_for(root_arm, root_object):
    return PickScene(matrix(root_arm), compose(inverse(matrix(root_arm)), matrix(root_object)))


def cube_symmetries():
    for order in permutations(range(3)):
        for signs in product((-1, 1), repeat=3):
            rotation = np.eye(3)[:, order] @ np.diag(signs)
            if np.linalg.det(rotation) > 0:
                yield rotation


@pytest.mark.parametrize(
    "angle", [0, 30, 44.999999, 45, 45.000001, 89.999999, 90, 90.000001, 135, 180, 270, 359.999999]
)
def test_cube_yaw_aligns_closing_axis_and_preserves_vertical_approach(settings, angle):
    half = np.deg2rad(angle) / 2
    obj = pose_matrix((0.5, 0.02, 0.4), (0, 0, np.sin(half), np.cos(half)))
    plan = plan_pick(settings, scene_for(np.eye(4), obj))
    grasp = np.asarray(from_pose(plan.grasp))
    assert grasp[:3, 3] == pytest.approx([0.5, 0.02, 0.44])
    assert grasp[:3, 0] == pytest.approx([0, 0, -1])
    cube = obj[:3, :3]
    closing = grasp[:3, 1]
    assert closing[2] == pytest.approx(0)
    assert max(abs(closing @ cube[:, axis]) for axis in (0, 1)) == pytest.approx(1)
    assert np.asarray(plan.pregrasp.position) - plan.grasp.position == pytest.approx([0, 0, 0.1])
    assert plan.pregrasp.orientation == pytest.approx(plan.grasp.orientation)
    width = 0.05
    assert plan.expected_cube_width_m == pytest.approx(width)
    assert plan.grasp_min_width_m == pytest.approx(width - 0.016)
    assert plan.grasp_max_width_m == pytest.approx(width + 0.005)


@pytest.mark.parametrize(
    "tool_quaternion", [(0, 0, 0, 1), (np.sin(np.pi / 12), 0, 0, np.cos(np.pi / 12))]
)
def test_tilt_symmetry_and_base_changes_keep_world_targets_fixed(settings, tool_quaternion):
    values = settings.model_dump()
    profile = values["profile"]
    profile["cube_axes_in_object_xyzw"] = (0.5, 0.5, 0.5, 0.5)
    profile["tcp_to_gripper_rotation_xyzw"] = tool_quaternion
    profile["grasp_contact_offset_tcp_m"] = (0.04, -0.012, 0.008)
    profile["gripper_open_m"] = 0.12
    settings = type(settings).model_validate(values)
    axes_calibration = pose_matrix(quaternion=profile["cube_axes_in_object_xyzw"])[:3, :3]
    installation = pose_matrix(quaternion=tool_quaternion)[:3, :3]
    corners = np.array(list(product((-0.025, 0.025), repeat=3))).T
    bases = [
        np.eye(4),
        pose_matrix((-0.10325, 0.02, 0), (0, 2**-0.5, 0, 2**-0.5)),
        pose_matrix((0.1, -0.2, 0.05), (2**-0.5, 0, 0, 2**-0.5)),
    ]
    rng = np.random.default_rng(79)
    for yaw in (0, 30, 45, 90, 135):
        tilt = pose_matrix(quaternion=(np.sin(np.pi / 36), 0, 0, np.cos(np.pi / 36)))
        world_cube = (
            pose_matrix(quaternion=(0, 0, np.sin(np.deg2rad(yaw) / 2), np.cos(np.deg2rad(yaw) / 2)))
            @ tilt
        )
        obj = world_cube.copy()
        obj[:3, 3] = rng.uniform(-0.5, 0.5, size=3)
        cube = world_cube[:3, :3]
        obj[:3, :3] = cube @ axes_calibration.T
        baseline = plan_pick(settings, scene_for(np.eye(4), obj))
        expected_rotation = pose_matrix(quaternion=baseline.grasp.orientation)[:3, :3]
        closing = (expected_rotation @ installation)[:, 1]
        assert closing[2] == pytest.approx(0, abs=1e-9)
        assert expected_rotation[:, 0] == pytest.approx([0, 0, -1])
        expected_width = float(np.ptp(closing @ cube @ corners))
        for symmetry in cube_symmetries():
            relabeled = obj.copy()
            relabeled[:3, :3] = cube @ symmetry @ axes_calibration.T
            for base in bases:
                plan = plan_pick(settings, scene_for(base, relabeled))
                grasp = np.asarray(compose(matrix(base), from_pose(plan.grasp)))
                pregrasp = np.asarray(compose(matrix(base), from_pose(plan.pregrasp)))
                assert grasp[:3, :3] == pytest.approx(expected_rotation, abs=1e-9)
                assert grasp[:3, 3] + expected_rotation @ profile[
                    "grasp_contact_offset_tcp_m"
                ] == pytest.approx(obj[:3, 3])
                assert pregrasp[:3, 3] - grasp[:3, 3] == pytest.approx([0, 0, 0.1])
                assert pregrasp[:3, :3] == pytest.approx(expected_rotation, abs=1e-9)
                assert plan.expected_cube_width_m == pytest.approx(expected_width)


def test_cube_axis_calibration_changes_horizontal_alignment(settings):
    values = settings.model_dump()
    values["profile"]["cube_axes_in_object_xyzw"] = (0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8))
    calibrated = type(settings).model_validate(values)
    scene = scene_for(np.eye(4), pose_matrix((0.5, 0.02, 0.4)))
    before, after = plan_pick(settings, scene), plan_pick(calibrated, scene)
    assert before.grasp.orientation != pytest.approx(after.grasp.orientation)
    assert before.expected_cube_width_m == pytest.approx(0.05)
    assert after.expected_cube_width_m == pytest.approx(0.05)


def test_projected_tilt_width_and_contact_tolerance_must_fit_open_gripper(settings):
    values = settings.model_dump()
    values["profile"]["gripper_open_m"] = 0.056
    settings = type(settings).model_validate(values)
    # Both horizontal cube axes have a small vertical component, increasing projection.
    q = np.array([0.06, 0.06, 0, 1.0])
    obj = pose_matrix((0.5, 0.02, 0.4), tuple(q / np.linalg.norm(q)))
    with pytest.raises(ContractError, match="projected grasp width does not fit gripper"):
        plan_pick(settings, scene_for(np.eye(4), obj))


def test_plan_log_reports_the_frozen_world_and_arm_geometry(settings):
    base = pose_matrix((-0.10325, 0, 0), (0, 2**-0.5, 0, 2**-0.5))
    obj = pose_matrix((0.2, -0.18, 0.42), (0.5, 0.5, 0.5, 0.5))
    scene = scene_for(base, obj)
    plan = plan_pick(settings, scene)
    fields = pick_plan_fields(settings, scene, plan)
    assert fields["object_center_root_m"] == pytest.approx(obj[:3, 3])
    assert fields["object_center_arm_m"] == pytest.approx(to_pose(scene.arm_to_object).position)
    assert fields["expected_cube_width_m"] == pytest.approx(0.05)
    assert fields["grasp_min_width_m"] == pytest.approx(0.034)
    assert fields["grasp_max_width_m"] == pytest.approx(0.055)
    assert fields["actual_gripper_closing_axis_root"] == pytest.approx([0, 1, 0])
    assert fields["grasp_pose_arm"]["position"] == pytest.approx(plan.grasp.position)
    assert fields["grasp_pose_root"]["position"] == pytest.approx([0.2, -0.18, 0.46])
    assert fields["approach_axis_root"] == [0, 0, -1]
    # D-093/D-094 observation fields and the TF place frame are logged even when absent.
    assert fields["observation_pose_arm"] == asdict(plan.observation)
    root_observation = to_pose(compose(scene.root_to_arm, from_pose(plan.observation)))
    assert fields["observation_pose_root"] == asdict(root_observation)
    assert fields["place_frame"] is None
    assert fields["place_pose_arm"] is None
    assert fields["place_pose_root"] is None


def test_console_plan_log_prints_object_place_and_grasp_poses_in_the_arm_frame(settings):
    values = settings.model_dump()
    values["frames"]["arm_base"] = "base_link"
    values["frames"]["place"] = PLACE_FRAME
    values["ros2"]["base_frame"] = "base_link"
    settings = type(settings).model_validate(values)
    scene = place_scene(place_position=(0.45, 0.1, 0.3))
    plan = plan_pick(settings, scene)
    fields = pick_plan_fields(settings, scene, plan)
    object_line, place_line, grasp_line = format_pick_plan_log(settings, scene, plan).splitlines()

    assert object_line.startswith(
        f"[rule_base] target object {settings.frames.object} center_base_link="
    )
    logged_values(object_line, "center_base_link", fields["object_center_arm_m"])
    logged_values(object_line, "orientation_xyzw", to_pose(scene.arm_to_object).orientation)
    assert place_line.startswith(f"[rule_base] place {PLACE_FRAME} center_base_link=")
    logged_values(place_line, "center_base_link", fields["place_pose_arm"]["position"])
    logged_values(place_line, "orientation_xyzw", fields["place_pose_arm"]["orientation"])
    assert grasp_line.startswith("[rule_base] grasp TCP=")
    logged_values(grasp_line, "TCP", fields["grasp_pose_arm"]["position"])
    logged_values(grasp_line, "pregrasp", fields["pregrasp_pose_arm"]["position"])
    assert grasp_line.endswith(" (base_link)")


def test_console_plan_log_reports_a_disabled_place_stage(settings):
    scene = scene_for(np.eye(4), pose_matrix((0.5, 0.02, 0.4)))
    plan = plan_pick(settings, scene)
    object_line, place_line, grasp_line = format_pick_plan_log(settings, scene, plan).splitlines()
    assert place_line == "[rule_base] place=(disabled)"
    assert object_line.startswith(
        f"[rule_base] target object {settings.frames.object} "
        f"center_{settings.frames.arm_base}="
    )
    assert grasp_line.endswith(f" ({settings.frames.arm_base})")


@pytest.mark.parametrize("tilt_deg", [16, 30, 45])
def test_rejects_cube_tilt_instead_of_tilting_the_fingers(settings, tilt_deg):
    half = np.deg2rad(tilt_deg) / 2
    obj = pose_matrix((0.5, 0, 0.4), (np.sin(half), 0, 0, np.cos(half)))
    with pytest.raises(ContractError, match="cube tilt"):
        plan_pick(settings, scene_for(np.eye(4), obj))


def test_retry_keeps_nearest_equivalent_orientation_across_symmetry_boundary(settings):
    def scene(angle):
        half = np.deg2rad(angle) / 2
        return scene_for(np.eye(4), pose_matrix((0.5, 0, 0.4), (0, 0, np.sin(half), np.cos(half))))

    before = plan_pick(settings, scene(44.9))
    after = plan_pick(settings, scene(45.1), previous_grasp_root=before.grasp.orientation)
    from rule_base.infrastructure.transform.math import errors

    assert errors(before.grasp, after.grasp)[1] == pytest.approx(np.deg2rad(0.2))


def test_rejects_reference_or_installation_that_cannot_keep_vertical_approach(settings):
    values = settings.model_dump()
    values["profile"]["tcp_to_gripper_rotation_xyzw"] = (0.5, -0.5, 0.5, 0.5)
    invalid = type(settings).model_validate(values)
    with pytest.raises(ContractError, match="horizontal gripper"):
        plan_pick(invalid, scene_for(np.eye(4), pose_matrix((0.5, 0, 0.4))))


def test_observation_pose_uses_the_reference_top_grasp_above_the_grasp_point(settings):
    scene = scene_for(np.eye(4), pose_matrix((0.5, 0.02, 0.4)))
    plan = plan_pick(settings, scene)
    root_grasp = np.asarray(compose(scene.root_to_arm, from_pose(plan.grasp)))
    expected = np.array(root_grasp)
    expected[:3, 3] += [0.0, 0.0, settings.profile.observation_clearance_m]
    expected[:3, :3] = np.asarray(
        from_pose(Pose((0, 0, 0), settings.profile.reference_tcp_orientation_root_xyzw))
    )[:3, :3]
    assert np.allclose(np.asarray(from_pose(plan.observation)), expected)


def test_observation_offset_and_clearance_are_root_frame_quantities(settings):
    values = settings.model_dump()
    values["profile"]["observation_clearance_m"] = 0.12
    values["profile"]["observation_offset_root_m"] = (0.01, -0.02, 0.0)
    settings = type(settings).model_validate(values)
    scene = scene_for(np.eye(4), pose_matrix((0.5, 0.02, 0.4)))
    plan = plan_pick(settings, scene)
    root_grasp = np.asarray(compose(scene.root_to_arm, from_pose(plan.grasp)))
    expected = np.array(root_grasp)
    expected[:3, 3] += np.asarray(settings.profile.observation_offset_root_m) + [
        0.0,
        0.0,
        settings.profile.observation_clearance_m,
    ]
    assert np.allclose(np.asarray(from_pose(plan.observation))[:3, 3], expected[:3, 3])


def test_place_pose_is_taken_from_the_scene_tf(settings):
    settings = with_place_frame(settings)
    scene = place_scene(place_position=(0.45, 0.1, 0.3))
    plan = plan_pick(settings, scene)
    assert plan.place is not None
    assert plan.place.position == pytest.approx((0.45, 0.1, 0.3))
    fields = pick_plan_fields(settings, scene, plan)
    assert fields["place_frame"] == PLACE_FRAME
    assert fields["place_pose_arm"] == asdict(plan.place)
    assert fields["place_pose_root"] == asdict(
        to_pose(compose(scene.root_to_arm, from_pose(plan.place)))
    )


def test_place_tf_is_required_once_frames_place_is_configured(settings):
    settings = with_place_frame(settings)
    scene = PickScene(
        from_pose(Pose((0, 0, 0), (0, 0, 0, 1))),
        from_pose(Pose((0.5, 0.0, 0.3), (0, 0, 0, 1))),
    )
    with pytest.raises(ContractError, match="place TF is missing"):
        plan_pick(settings, scene)


def test_place_tf_is_ignored_without_frames_place(settings):
    assert settings.frames.place is None
    plan = plan_pick(settings, place_scene())
    assert plan.place is None


def test_rejects_place_pose_outside_the_workspace():
    settings = load_settings(CONFIG)
    settings = with_place_frame(settings)
    with pytest.raises(ContractError, match="workspace"):
        plan_pick(settings, place_scene(place_position=(0.9, 0.0, 0.2)))


def test_rejects_place_approach_outside_the_workspace():
    settings = load_settings(CONFIG)
    settings = with_place_frame(settings)
    # The place pose is inside the box but the +clearance approach is above z_max.
    with pytest.raises(ContractError, match="workspace"):
        plan_pick(settings, place_scene(place_position=(0.4, 0.0, 0.79)))


def test_rejects_place_pose_that_is_not_a_top_grasp():
    settings = load_settings(CONFIG)
    settings = with_place_frame(settings)
    with pytest.raises(ContractError, match="approach axis down"):
        plan_pick(settings, place_scene(place_quaternion=(0.0, 0.0, 0.0, 1.0)))


def test_rejects_place_pose_tilted_beyond_the_top_grasp_limit():
    settings = load_settings(CONFIG)
    settings = with_place_frame(settings)
    reference = np.asarray(
        from_pose(Pose((0, 0, 0), (0.0, 0.7071067811865476, 0.0, 0.7071067811865476)))
    )
    tilt = np.asarray(
        from_pose(Pose((0, 0, 0), (0.0, np.sin(np.deg2rad(8)), 0.0, np.cos(np.deg2rad(8)))))
    )
    place = np.eye(4)
    place[:3, :3] = tilt[:3, :3] @ reference[:3, :3]
    place[:3, 3] = (0.4, 0.0, 0.2)
    quaternion = to_pose(matrix(place)).orientation
    with pytest.raises(ContractError, match="approach axis down"):
        plan_pick(settings, place_scene(place_quaternion=quaternion))
