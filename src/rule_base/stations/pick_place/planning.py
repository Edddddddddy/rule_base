"""Align the jaws with cube sides while keeping a vertical world approach."""

from __future__ import annotations

import math
from dataclasses import asdict
from typing import cast

import numpy as np
from numpy.typing import NDArray

from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import ContractError, PickPlan, PickScene, Pose, Quaternion
from rule_base.infrastructure.transform.math import (
    compose,
    from_pose,
    inverse,
    matrix,
    offset,
    to_pose,
)


def _rotation(quaternion: Quaternion) -> NDArray[np.float64]:
    return np.asarray(from_pose(Pose((0, 0, 0), quaternion)))[:3, :3]


def workspace(pose: Pose, settings: PickSettings) -> None:
    profile = settings.profile
    if not all(
        low <= value <= high
        for value, low, high in zip(
            pose.position, profile.workspace_min_m, profile.workspace_max_m, strict=True
        )
    ):
        raise ContractError(
            "Pick TCP exceeds calibrated workspace: "
            f"position_m={pose.position}, "
            f"min_m={profile.workspace_min_m}, max_m={profile.workspace_max_m}"
        )


def _aligned_rotation(
    settings: PickSettings,
    cube: NDArray[np.float64],
    previous_grasp_root: Quaternion | None,
) -> NDArray[np.float64]:
    profile = settings.profile
    reference = _rotation(profile.reference_tcp_orientation_root_xyzw)
    installation = _rotation(profile.tcp_to_gripper_rotation_xyzw)
    closing = (reference @ installation)[:, 1]
    if not np.allclose(reference[:, 0], [0, 0, -1], atol=1e-3) or abs(closing[2]) > 1e-3:
        raise ContractError("Pick alignment requires downward TCP +X and horizontal gripper +Y")
    vertical_axis = int(np.argmax(np.abs(cube[2, :])))
    tilt = math.acos(float(np.clip(abs(cube[2, vertical_axis]), 0, 1)))
    if tilt > settings.limits.max_cube_tilt_rad:
        raise ContractError(f"Pick cube tilt exceeds horizontal grasp limit: {tilt:.6f} rad")
    preferred = (
        closing
        if previous_grasp_root is None
        else (_rotation(previous_grasp_root) @ installation)[:, 1]
    )
    candidates: list[tuple[float, float, NDArray[np.float64]]] = []
    for index in range(3):
        if index == vertical_axis:
            continue
        axis = cube[:2, index] / np.linalg.norm(cube[:2, index])
        for sign in (-1, 1):
            direction = sign * axis
            yaw = math.atan2(
                closing[0] * direction[1] - closing[1] * direction[0],
                float(closing[:2] @ direction),
            )
            delta = math.atan2(
                preferred[0] * direction[1] - preferred[1] * direction[0],
                float(preferred[:2] @ direction),
            )
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ reference
            # Cube axis relabeling must not change the selected pose at a tie.
            candidates.append((round(abs(delta), 9), yaw, rotation))
    return min(candidates, key=lambda candidate: candidate[:2])[2]


def _observation_pose(settings: PickSettings, scene: PickScene, grasp: Pose) -> Pose:
    """Capture view above the grasp TCP with the fixed reference top-grasp orientation."""
    profile = settings.profile
    root_grasp = to_pose(compose(scene.root_to_arm, from_pose(grasp)))
    offset_root = profile.observation_offset_root_m
    observation_root = Pose(
        (
            root_grasp.position[0] + offset_root[0],
            root_grasp.position[1] + offset_root[1],
            root_grasp.position[2] + offset_root[2] + profile.observation_clearance_m,
        ),
        profile.reference_tcp_orientation_root_xyzw,
    )
    observation = to_pose(compose(inverse(scene.root_to_arm), from_pose(observation_root)))
    workspace(observation, settings)
    return observation


def _place_pose(settings: PickSettings, scene: PickScene) -> Pose | None:
    """Place pose read from the effective place TF (frames.place or env place_frame)."""
    frame = settings.place_frame
    if frame is None:
        return None
    if scene.arm_to_place is None:
        raise ContractError(f"Pick place TF is missing: {settings.frames.arm_base} <- {frame}")
    profile = settings.profile
    place = to_pose(scene.arm_to_place)
    workspace(place, settings)
    workspace(offset(place, (0.0, 0.0, profile.place_approach_clearance_m)), settings)
    # The TCP approach axis (+X) must point down and the closing axis (+Y) must
    # stay horizontal in the root frame, exactly like the grasp alignment gate.
    root_place = np.asarray(compose(scene.root_to_arm, from_pose(place)))
    limit = settings.limits.max_cube_tilt_rad
    approach_tilt = math.acos(float(np.clip(-root_place[2, 0], -1.0, 1.0)))
    closing_tilt = math.asin(float(np.clip(abs(root_place[2, 1]), 0.0, 1.0)))
    if approach_tilt > limit or closing_tilt > limit:
        raise ContractError(
            "Pick place pose must keep the TCP approach axis down and the closing axis "
            f"horizontal: approach_tilt_rad={approach_tilt:.6f}, "
            f"closing_tilt_rad={closing_tilt:.6f}, limit_rad={limit:.6f}"
        )
    return place


def _target_grasp(settings: PickSettings, scene: PickScene) -> Pose:
    """Vision target TCP pose, gated for a vertical top grasp.

    The vision node publishes the target TCP directly (contract §5.2), so the
    grasp pose is the TF pose instead of the cube-face alignment heuristic. The
    existing top-grasp gate still applies: the TCP approach axis (+X) must point
    down and the closing axis (+Y) must stay horizontal in the root frame.
    S6j: ``profile.grasp_target_offset_arm_m`` is added to the target position
    first (arm_base frame, position only; orientation unchanged), so the fixed
    lateral correction still goes through the same top-grasp/workspace gates and
    the pregrasp/observation poses derived from the grasp follow it.
    """
    frame = settings.frames.grasp_target
    if scene.arm_to_target is None:
        raise ContractError(
            f"Pick grasp_target TF is missing: {settings.frames.arm_base} <- {frame}"
        )
    grasp = offset(to_pose(scene.arm_to_target), settings.profile.grasp_target_offset_arm_m)
    root_grasp = np.asarray(compose(scene.root_to_arm, from_pose(grasp)))
    limit = settings.limits.max_cube_tilt_rad
    approach_tilt = math.acos(float(np.clip(-root_grasp[2, 0], -1.0, 1.0)))
    closing_tilt = math.asin(float(np.clip(abs(root_grasp[2, 1]), 0.0, 1.0)))
    if approach_tilt > limit or closing_tilt > limit:
        raise ContractError(
            "Pick grasp_target pose must keep the TCP approach axis down and the closing "
            f"axis horizontal: approach_tilt_rad={approach_tilt:.6f}, "
            f"closing_tilt_rad={closing_tilt:.6f}, limit_rad={limit:.6f}"
        )
    grasp = _align_target_closing_axis(settings, scene, grasp)
    workspace(grasp, settings)
    return grasp


def _align_target_closing_axis(settings: PickSettings, scene: PickScene, grasp: Pose) -> Pose:
    """Pick the 180°-equivalent top-grasp roll that the calibrated profile supports.

    A single ArUco marker fixes the target yaw, so the vision target can ask for
    either ``+Y`` or ``-Y`` closing direction.  Both are the same physical grasp
    line for a top grasp, but the SO-101 wrist only reaches the roll family of the
    profile reference orientation (the D-047 180° roll); asking for the other one
    makes MoveIt return ``NO_IK_SOLUTION``.  Flip 180° about the TCP approach axis
    (position and approach direction stay unchanged) when the target closing axis
    points against the reference closing axis.
    """
    profile = settings.profile
    root_grasp = np.asarray(compose(scene.root_to_arm, from_pose(grasp)))
    reference = _rotation(profile.reference_tcp_orientation_root_xyzw)
    if float(root_grasp[:3, 1] @ reference[:, 1]) >= 0.0:
        return grasp
    rotation = root_grasp[:3, :3].copy()
    rotation[:, 1] = -rotation[:, 1]
    rotation[:, 2] = -rotation[:, 2]
    arm_rotation = np.asarray(scene.root_to_arm)[:3, :3].T @ rotation
    arm_tcp = np.eye(4)
    arm_tcp[:3, :3] = arm_rotation
    arm_tcp[:3, 3] = grasp.position
    flipped = to_pose(matrix(arm_tcp))
    root_flipped = np.asarray(compose(scene.root_to_arm, from_pose(flipped)))
    limit = settings.limits.max_cube_tilt_rad
    approach_tilt = math.acos(float(np.clip(-root_flipped[2, 0], -1.0, 1.0)))
    closing_tilt = math.asin(float(np.clip(abs(root_flipped[2, 1]), 0.0, 1.0)))
    if approach_tilt > limit or closing_tilt > limit:
        raise ContractError(
            "Pick grasp_target flip must keep the top grasp: "
            f"approach_tilt_rad={approach_tilt:.6f}, closing_tilt_rad={closing_tilt:.6f}, "
            f"limit_rad={limit:.6f}"
        )
    return flipped


def _plan_from_target(settings: PickSettings, scene: PickScene) -> PickPlan:
    """Plan from the vision target TCP; keep the existing width-check semantics."""
    profile = settings.profile
    grasp = _target_grasp(settings, scene)
    root_arm_rotation = np.asarray(scene.root_to_arm)[:3, :3]
    pregrasp = offset(
        grasp, tuple(root_arm_rotation.T @ np.array([0.0, 0.0, profile.pregrasp_clearance_m]))
    )
    workspace(pregrasp, settings)
    # Keep the calibrated face-contact width interval; only the projection now uses
    # the vision target's closing axis instead of the cube-face heuristic.
    root_object = np.asarray(compose(scene.root_to_arm, scene.arm_to_object))
    cube = root_object[:3, :3] @ _rotation(profile.cube_axes_in_object_xyzw)
    root_grasp = np.asarray(compose(scene.root_to_arm, from_pose(grasp)))
    tool = root_grasp[:3, :3] @ _rotation(profile.tcp_to_gripper_rotation_xyzw)
    expected_width = profile.cube_side_m * float(np.abs(tool[:, 1] @ cube).sum())
    width_delta = expected_width - profile.cube_side_m
    min_width = profile.grasp_min_width_m + width_delta
    max_width = profile.grasp_max_width_m + width_delta
    if not (profile.gripper_closed_m < min_width < max_width < profile.gripper_open_m):
        raise ContractError(
            "Pick projected grasp width does not fit gripper: "
            f"expected_width_m={expected_width}, "
            f"interval_m=({min_width}, {max_width}), open_width_m={profile.gripper_open_m}"
        )
    return PickPlan(
        grasp=grasp,
        pregrasp=pregrasp,
        observation=_observation_pose(settings, scene, grasp),
        calibration_revision=profile.calibration_revision,
        expected_cube_width_m=expected_width,
        grasp_min_width_m=min_width,
        grasp_max_width_m=max_width,
        place=_place_pose(settings, scene),
    )


def _env_grasp_pose(settings: PickSettings, scene: PickScene) -> Pose:
    """Env-mode grasp TCP: env marker position + TCP-frame top-grasp contact offset.

    The env node publishes ``env_marker_<target_object_id>`` in ``frames.arm_base``.
    The grasp orientation is always the calibrated top-grasp reference converted to
    the arm frame; ``profile.grasp_contact_offset_tcp_m`` is expressed in the TCP
    frame and therefore rotated by that reference orientation.  The marker rotation
    is deliberately not consumed: the env camera places the marker on the object,
    while the grasp stays a calibrated fixed top grasp.
    """
    env = settings.env_frames
    profile = settings.profile
    if scene.arm_to_object is None:
        raise ContractError(
            f"Pick env object TF is missing: {settings.frames.arm_base} <- {env.object_frame}"
        )
    root_to_arm = np.asarray(scene.root_to_arm)
    arm_to_root = np.asarray(inverse(scene.root_to_arm))
    reference = profile.reference_tcp_orientation_root_xyzw
    marker_arm = np.asarray(to_pose(scene.arm_to_object).position)
    marker_root = root_to_arm[:3, :3] @ marker_arm + root_to_arm[:3, 3]
    contact_root = _rotation(reference) @ np.asarray(profile.grasp_contact_offset_tcp_m)
    grasp_root = from_pose(Pose(tuple(marker_root + contact_root), reference))
    grasp = to_pose(compose(arm_to_root, grasp_root))
    # Field calibration in the arm/command frame: the env marker only exposes the
    # marker plane, while the top grasp needs the fingertip contact point (contact
    # depth below the marker) and absorbs residual marker/extrinsics bias.
    grasp_offset = np.asarray(env.grasp_offset_arm_m, dtype=float)
    if np.any(grasp_offset):
        grasp = Pose(tuple(np.asarray(grasp.position) + grasp_offset), grasp.orientation)
    root_grasp = np.asarray(compose(scene.root_to_arm, from_pose(grasp)))
    limit = settings.limits.max_cube_tilt_rad
    approach_tilt = math.acos(float(np.clip(-root_grasp[2, 0], -1.0, 1.0)))
    closing_tilt = math.asin(float(np.clip(abs(root_grasp[2, 1]), 0.0, 1.0)))
    if approach_tilt > limit or closing_tilt > limit:
        raise ContractError(
            "Pick env grasp pose must keep the TCP approach axis down and the closing "
            f"axis horizontal: approach_tilt_rad={approach_tilt:.6f}, "
            f"closing_tilt_rad={closing_tilt:.6f}, limit_rad={limit:.6f}"
        )
    workspace(grasp, settings)
    return grasp


def _plan_from_env(settings: PickSettings, scene: PickScene) -> PickPlan:
    """Plan from the one env snapshot (object marker + tf_static place frame)."""
    profile = settings.profile
    grasp = _env_grasp_pose(settings, scene)
    root_arm_rotation = np.asarray(scene.root_to_arm)[:3, :3]
    pregrasp = offset(
        grasp, tuple(root_arm_rotation.T @ np.array([0.0, 0.0, profile.pregrasp_clearance_m]))
    )
    workspace(pregrasp, settings)
    # The env marker rotation is not the cube axis convention, so keep the
    # calibrated cube-side interval instead of projecting a face width; the live
    # measured-width contact gate still decides grasp success.
    expected_width = profile.cube_side_m
    min_width = profile.grasp_min_width_m
    max_width = profile.grasp_max_width_m
    if not (profile.gripper_closed_m < min_width < max_width < profile.gripper_open_m):
        raise ContractError(
            "Pick env grasp width interval does not fit gripper: "
            f"interval_m=({min_width}, {max_width}), open_width_m={profile.gripper_open_m}"
        )
    return PickPlan(
        grasp=grasp,
        pregrasp=pregrasp,
        observation=_observation_pose(settings, scene, grasp),
        calibration_revision=profile.calibration_revision,
        expected_cube_width_m=expected_width,
        grasp_min_width_m=min_width,
        grasp_max_width_m=max_width,
        place=_place_pose(settings, scene),
    )


def first_observation_pose(settings: PickSettings) -> Pose:
    """Fixed wide-view observation TCP pose for ``vision.observe_first`` (D-127).

    The pose is configured in the arm command frame (``frames.arm_base``); the
    caller is responsible for the workspace gate.  It is deliberately independent
    of any object TF so a round can start before the marker has been detected.
    """
    vision = settings.vision
    return Pose(vision.observe_position_arm_m, vision.observe_orientation_arm_xyzw)


def plan_pick(
    settings: PickSettings,
    scene: PickScene,
    *,
    previous_grasp_root: Quaternion | None = None,
) -> PickPlan:
    if settings.env_frames.enabled:
        return _plan_from_env(settings, scene)
    if settings.frames.grasp_target is not None:
        return _plan_from_target(settings, scene)
    profile = settings.profile
    root_object = np.asarray(compose(scene.root_to_arm, scene.arm_to_object))
    root_arm_rotation = np.asarray(scene.root_to_arm)[:3, :3]
    cube = root_object[:3, :3] @ _rotation(profile.cube_axes_in_object_xyzw)
    root_tcp_rotation = _aligned_rotation(settings, cube, previous_grasp_root)
    tool = root_tcp_rotation @ _rotation(profile.tcp_to_gripper_rotation_xyzw)
    expected_width = profile.cube_side_m * float(np.abs(tool[:, 1] @ cube).sum())
    # Preserve the calibrated face-contact tolerance around this scene's projection.
    width_delta = expected_width - profile.cube_side_m
    min_width = profile.grasp_min_width_m + width_delta
    max_width = profile.grasp_max_width_m + width_delta
    if not (profile.gripper_closed_m < min_width < max_width < profile.gripper_open_m):
        raise ContractError(
            "Pick projected grasp width does not fit gripper: "
            f"expected_width_m={expected_width}, "
            f"interval_m=({min_width}, {max_width}), open_width_m={profile.gripper_open_m}"
        )
    arm_tcp = np.eye(4)
    arm_tcp[:3, :3] = root_arm_rotation.T @ root_tcp_rotation
    # arm_to_object already contains the base translation. Apply only the tool offset.
    arm_tcp[:3, 3] = (
        np.asarray(scene.arm_to_object)[:3, 3]
        - arm_tcp[:3, :3] @ profile.grasp_contact_offset_tcp_m
    )
    grasp = to_pose(matrix(arm_tcp))
    observation = _observation_pose(settings, scene, grasp)
    arm_tcp[:3, 3] += root_arm_rotation.T @ [0, 0, profile.pregrasp_clearance_m]
    pregrasp = to_pose(matrix(arm_tcp))
    workspace(grasp, settings)
    workspace(pregrasp, settings)
    place = _place_pose(settings, scene)
    return PickPlan(
        grasp=grasp,
        pregrasp=pregrasp,
        observation=observation,
        calibration_revision=profile.calibration_revision,
        expected_cube_width_m=expected_width,
        grasp_min_width_m=min_width,
        grasp_max_width_m=max_width,
        place=place,
    )


def pick_plan_fields(settings: PickSettings, scene: PickScene, plan: PickPlan) -> dict[str, object]:
    """Describe the frozen input and plan without reading TF or replanning."""
    profile = settings.profile
    root_object = compose(scene.root_to_arm, scene.arm_to_object)
    cube = np.asarray(root_object)[:3, :3] @ _rotation(profile.cube_axes_in_object_xyzw)
    root_grasp = compose(scene.root_to_arm, from_pose(plan.grasp))
    tool = np.asarray(root_grasp)[:3, :3] @ _rotation(profile.tcp_to_gripper_rotation_xyzw)
    closing = tool[:, 1]
    root_observation = compose(scene.root_to_arm, from_pose(plan.observation))
    root_place = (
        compose(scene.root_to_arm, from_pose(plan.place)) if plan.place is not None else None
    )
    return {
        "root_frame": settings.frames.root,
        "arm_base_frame": settings.frames.arm_base,
        "object_frame": settings.object_frame,
        "place_frame": settings.place_frame,
        "calibration_revision": plan.calibration_revision,
        "field_validated": profile.field_validated,
        "root_to_arm": scene.root_to_arm,
        "object_center_root_m": to_pose(root_object).position,
        "object_center_arm_m": to_pose(scene.arm_to_object).position,
        "object_orientation_root_xyzw": to_pose(root_object).orientation,
        "cube_axes_root": cube.T.tolist(),
        "expected_cube_width_m": plan.expected_cube_width_m,
        "grasp_min_width_m": plan.grasp_min_width_m,
        "grasp_max_width_m": plan.grasp_max_width_m,
        "actual_gripper_closing_axis_root": closing.tolist(),
        "finger_longitudinal_axis_root": tool[:, 0].tolist(),
        "reference_tcp_orientation_root_xyzw": profile.reference_tcp_orientation_root_xyzw,
        "grasp_contact_offset_tcp_m": profile.grasp_contact_offset_tcp_m,
        "target_closing_axis_flipped": bool(
            scene.arm_to_target is not None
            and float(
                np.asarray(compose(scene.root_to_arm, scene.arm_to_target))[:3, 1]
                @ _rotation(profile.reference_tcp_orientation_root_xyzw)[:, 1]
            )
            < 0.0
        ),
        "grasp_pose_root": asdict(to_pose(root_grasp)),
        "pregrasp_pose_root": asdict(to_pose(compose(scene.root_to_arm, from_pose(plan.pregrasp)))),
        "observation_pose_root": asdict(to_pose(root_observation)),
        "place_pose_root": asdict(to_pose(root_place)) if root_place is not None else None,
        "grasp_pose_arm": asdict(plan.grasp),
        "pregrasp_pose_arm": asdict(plan.pregrasp),
        "observation_pose_arm": asdict(plan.observation),
        "place_pose_arm": asdict(plan.place) if plan.place is not None else None,
        "approach_axis_root": [0, 0, -1],
        "lift_axis_root": [0, 0, 1],
    }


def _format_log_numbers(values: tuple[float, ...]) -> str:
    """Console-friendly ``[x, y, z]`` at 0.1 mm / 1e-4 quaternion resolution."""
    # ``+ 0.0`` collapses IEEE -0.0 so the operator log never shows "[-0.0000, ...]".
    return "[" + ", ".join(f"{float(value) + 0.0:.4f}" for value in values) + "]"


def format_pick_plan_log(settings: PickSettings, scene: PickScene, plan: PickPlan) -> str:
    """Render the operator console block for a freshly generated Pick plan.

    Reuses :func:`pick_plan_fields` instead of repeating the plan geometry, so the
    printed numbers match the ``pick_plan`` event exactly.  Frame names come from
    the config (never hardcoded) and the poses are expressed in the arm command
    frame (``frames.arm_base``, usually ``base_link``).  Pure function: the legacy
    and observe-first plan paths only print the returned block.
    """
    fields = pick_plan_fields(settings, scene, plan)
    arm_base = settings.frames.arm_base
    arm_object = to_pose(scene.arm_to_object)
    object_center = cast(tuple[float, float, float], fields["object_center_arm_m"])
    lines = [
        f"[rule_base] target object {settings.object_frame} "
        f"center_{arm_base}={_format_log_numbers(object_center)} m "
        f"orientation_xyzw={_format_log_numbers(arm_object.orientation)}"
    ]
    place = fields["place_pose_arm"]
    if place is None or settings.place_frame is None:
        lines.append("[rule_base] place=(disabled)")
    else:
        place_pose = cast(dict[str, tuple[float, ...]], place)
        lines.append(
            f"[rule_base] place {settings.place_frame} "
            f"center_{arm_base}={_format_log_numbers(place_pose['position'])} m "
            f"orientation_xyzw={_format_log_numbers(place_pose['orientation'])}"
        )
    grasp = cast(dict[str, tuple[float, ...]], fields["grasp_pose_arm"])
    pregrasp = cast(dict[str, tuple[float, ...]], fields["pregrasp_pose_arm"])
    lines.append(
        f"[rule_base] grasp TCP={_format_log_numbers(grasp['position'])} "
        f"/ pregrasp={_format_log_numbers(pregrasp['position'])} ({arm_base})"
    )
    return "\n".join(lines)
