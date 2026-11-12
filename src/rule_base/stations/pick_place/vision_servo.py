"""Bounded visual-servo alignment for the Pick station.

Contract (tools/vision/CONTRACT.md §6.2): the station closes the loop discretely —
refresh one detection, wait for a newer object/target TF sample, replan/correct, and
descend only when the target is within tolerance. It never runs a continuous
high-rate loop and never descends while misaligned. Refresh failures, missing TF
samples and exhausted corrections are explicit `ContractError`s: the round fails
without descending or closing the jaws, and no extra recovery motion is invented.

The helpers operate on a `PickRound` instance so the state machine keeps a single
owner of stages, Servo deadlines and movement; this module only sequences vision.
"""

from __future__ import annotations

import dataclasses
import json
import time
from typing import TYPE_CHECKING

import numpy as np

from rule_base.domain.models import ContractError, PickScene, Pose
from rule_base.infrastructure.transform.math import errors, from_pose, to_pose
from rule_base.observability.events import event

from .planning import (
    first_observation_pose,
    format_pick_plan_log,
    pick_plan_fields,
    plan_pick,
)

if TYPE_CHECKING:  # pragma: no cover - typing only, avoids a runtime import cycle
    from .execution import PickRound

OBSERVATION_STAGE = "vision_align"
PREGRASP_STAGE = "vision_pregrasp_align"
CORRECTION_STAGE = "vision_correct_pregrasp"


def align_at_observation(cycle: PickRound) -> None:
    """Refresh once at the observation pose and replan when the target moved."""
    if not cycle.settings.vision.enabled:
        return
    assert cycle.plan is not None
    stage = OBSERVATION_STAGE
    cycle._stage(stage)
    scene = refresh_scene(cycle, stage)
    candidate = plan_pick(cycle.settings, scene)
    delta_position, delta_angle = errors(candidate.grasp, cycle.plan.grasp)
    if _within_tolerance(cycle, delta_position, delta_angle):
        event(
            "vision_aligned",
            round_id=cycle.round_id,
            stage=stage,
            attempts=0,
            residual_position_m=delta_position,
            residual_angle_rad=delta_angle,
        )
        return
    event(
        "vision_correction",
        round_id=cycle.round_id,
        stage=stage,
        attempt=1,
        delta_position_m=delta_position,
        delta_angle_rad=delta_angle,
    )
    # The observation view itself is not re-approached: the next move is the
    # (replanned) pregrasp, exactly as the frozen stage order requires.
    cycle.plan = candidate
    cycle._scene = scene


def verify_grasp_target(cycle: PickRound, *, stage: str = "pregrasp_verify") -> None:
    """One-shot safety check at the pregrasp pose before descending to the grasp pose.

    The target is locked after the observation pose (D-132), so this check never
    corrects the plan.  It only detects the obvious failure mode where the object
    was pushed/moved during approach: a fresh detection that disagrees with the
    locked grasp by more than the configured tolerance aborts the round *before*
    the descent and the close (real-machine close width 0.0049 = jaws closed on
    nothing).  The check runs at the pregrasp pose, where the wide-view marker is
    still visible: at the grasp pose the gripper occludes the marker and the
    close-range re-detection is unreliable (S6e field failure: z jumped
    -23 mm -> +47 mm and the grasp was wrongly aborted).  A missing/occluded
    detection is logged and skipped — it must not block the grasp; the close-width
    gate still covers the final grasp.
    """
    vision = cycle.settings.vision
    if not (vision.enabled and vision.grasp_verify_enabled):
        return
    assert cycle.plan is not None
    cycle._stage(stage)
    try:
        scene = refresh_scene(cycle, stage)
    except ContractError as exc:
        event(
            "grasp_verify_skipped",
            round_id=cycle.round_id,
            stage=stage,
            reason="vision refresh unavailable at the pregrasp pose",
            error=str(exc),
        )
        return
    try:
        candidate = plan_pick(cycle.settings, scene)
    except (ContractError, ValueError) as exc:
        event(
            "grasp_verify_skipped",
            round_id=cycle.round_id,
            stage=stage,
            reason="candidate plan invalid at the pregrasp pose",
            error=str(exc),
        )
        return
    delta_position, delta_angle = errors(candidate.grasp, cycle.plan.grasp)
    if (
        delta_position > vision.grasp_verify_tolerance_m
        or delta_angle > vision.grasp_verify_tolerance_rad
    ):
        raise ContractError(
            "GRASP_TARGET_MISMATCH: detection at the pregrasp pose disagrees with the "
            f"locked plan (delta {delta_position * 1000:.1f} mm / {delta_angle:.4f} rad; "
            f"limits {vision.grasp_verify_tolerance_m * 1000:.1f} mm / "
            f"{vision.grasp_verify_tolerance_rad:.4f} rad); the object may have been "
            "pushed during approach. The arm did not descend and the gripper was not closed."
        )
    event(
        "grasp_verify_ok",
        round_id=cycle.round_id,
        stage=stage,
        delta_position_m=round(delta_position, 6),
        delta_angle_rad=round(delta_angle, 6),
    )


def align_or_lock_at_pregrasp(cycle: PickRound, reduced: bool) -> None:
    """Pregrasp vision step: align, or skip when the target was locked (D-132)."""
    if reduced:
        return
    if cycle.settings.vision.enabled and cycle.settings.vision.lock_target_after_observation:
        event(
            "vision_target_locked",
            round_id=cycle.round_id,
            stage="pregrasp",
            reason="target locked after observation; no pregrasp correction",
        )
        return
    align_at_pregrasp(cycle)


def align_at_pregrasp(cycle: PickRound) -> None:
    """Refresh at the pregrasp pose; correct at most `max_corrections`, then descend."""
    if not cycle.settings.vision.enabled:
        return
    assert cycle.plan is not None
    stage = PREGRASP_STAGE
    vision = cycle.settings.vision
    corrections = 0
    if vision.correction_settle_s > 0:
        _settle(cycle, vision.correction_settle_s)
    while True:
        cycle._stage(stage)
        scene = refresh_scene(cycle, stage)
        candidate = plan_pick(cycle.settings, scene)
        delta_position, delta_angle = errors(candidate.grasp, cycle.plan.grasp)
        if _within_tolerance(cycle, delta_position, delta_angle):
            event(
                "vision_aligned",
                round_id=cycle.round_id,
                stage=stage,
                attempts=corrections,
                residual_position_m=delta_position,
                residual_angle_rad=delta_angle,
            )
            return
        if corrections >= vision.max_corrections:
            raise ContractError(
                "vision_alignment_incomplete: "
                f"stage={stage}; corrections={corrections}; "
                f"residual_position_m={delta_position:.6f}; "
                f"residual_angle_rad={delta_angle:.6f}; "
                f"position_tolerance_m={vision.position_tolerance_m}; "
                f"orientation_tolerance_rad={vision.orientation_tolerance_rad}"
            )
        corrections += 1
        event(
            "vision_correction",
            round_id=cycle.round_id,
            stage=stage,
            attempt=corrections,
            delta_position_m=delta_position,
            delta_angle_rad=delta_angle,
        )
        cycle.plan = candidate
        cycle._scene = scene
        # Keep the jaws open and the Servo cadence: a plain move to the corrected
        # pregrasp, then a servo-kept settle so the next detection sees a still arm.
        cycle._move(CORRECTION_STAGE, candidate.pregrasp, settle=False)
        if vision.correction_settle_s > 0:
            _settle(cycle, vision.correction_settle_s)


def _within_tolerance(cycle: PickRound, delta_position: float, delta_angle: float) -> bool:
    vision = cycle.settings.vision
    return (
        delta_position <= vision.position_tolerance_m
        and delta_angle <= vision.orientation_tolerance_rad
    )


def refresh_scene(cycle: PickRound, stage: str) -> PickScene:
    """Refresh the vision detection, then require one newer object/target sample."""
    return _refresh_scene_with_message(cycle, stage)[0]


def _refresh_scene_with_message(cycle: PickRound, stage: str) -> tuple[PickScene, str]:
    """Refresh + wait for a newer TF sample; also return the raw service message."""
    vision, limits = cycle.settings.vision, cycle.settings.limits
    started = time.monotonic()
    deadline = started + vision.refresh_timeout_s
    try:
        message = cycle.environment.refresh_object_pose()
    except ContractError as exc:
        _refresh_event(cycle, stage, ok=False, started=started, error=str(exc))
        raise
    while True:
        iteration = time.monotonic()
        cycle._check(check_workspace=False)
        cycle.environment.check_health()
        try:
            scene = cycle.environment.snapshot()
        except (ContractError, ValueError) as exc:
            if time.monotonic() >= deadline:
                _refresh_event(
                    cycle, stage, ok=False, started=started, message=message, error=str(exc)
                )
                raise ContractError(
                    "vision refresh timed out waiting for a new TF sample during "
                    f"{stage}: {exc}"
                ) from exc
            cycle._pause(max(0.0, 1 / limits.servo_hz - (time.monotonic() - iteration)))
            continue
        if time.monotonic() > deadline:
            _refresh_event(
                cycle,
                stage,
                ok=False,
                started=started,
                message=message,
                error="refresh_timeout_s exceeded",
            )
            raise ContractError(
                f"vision refresh exceeded refresh_timeout_s during {stage}: "
                f"{time.monotonic() - started:.3f}s > {vision.refresh_timeout_s:.3f}s"
            )
        _refresh_event(cycle, stage, ok=True, started=started, message=message)
        return scene, message


def _refresh_event(
    cycle: PickRound,
    stage: str,
    *,
    ok: bool,
    started: float,
    message: str = "",
    error: str = "",
) -> None:
    fields: dict[str, object] = {
        "round_id": cycle.round_id,
        "stage": stage,
        "ok": ok,
        "age_s": cycle.environment.sample_age_s(),
        "elapsed_s": time.monotonic() - started,
        "service": cycle.settings.vision.refresh_service,
    }
    if message:
        fields["message"] = message
    if error:
        fields["error"] = error
    event("vision_refresh", **fields)


def _settle(cycle: PickRound, seconds: float) -> None:
    """Hold the current Servo target for `seconds` while keeping its deadline alive.

    A TCP Servo target is re-streamed at ``servo_hz`` while waiting: the observation
    settle (0.8 s) is longer than ``servo_max_gap_s``, so a plain sleep would trip the
    Servo refresh deadline.  A joint-space observation hold has no Servo target and
    just keeps checking health/stop.
    """
    limits = cycle.settings.limits
    deadline = time.monotonic() + seconds
    hold = seconds > 0
    next_send = time.monotonic()
    while True:
        iteration = time.monotonic()
        cycle._check(check_workspace=False)
        target = cycle._servo_target
        if hold and target is not None and time.monotonic() >= next_send:
            cycle._send_servo(target)
            next_send = time.monotonic() + 1 / limits.servo_hz
        if time.monotonic() >= deadline:
            return
        cycle._pause(max(0.0, 1 / limits.servo_hz - (time.monotonic() - iteration)))


def is_ik_denial(message: str) -> bool:
    """True when arm_app rejected the pose because MoveIt found no IK solution."""
    return (
        "IK_FAILED" in message
        or "UNREACHABLE" in message
        or "plan_request denied" in message
    )


def move_to_pregrasp(cycle: PickRound) -> bool:
    """Move to the pregrasp pose, falling back to the grasp pose when unreachable.

    The real arm's vertical reach with the top-grasp roll shrinks quickly with the
    horizontal distance: for an object near the outer workspace the nominal
    pregrasp (grasp + clearance) can be IK-unreachable while the grasp pose itself
    is still reachable.  In that case skip the approach clearance and vision
    pre-alignment, and move straight to the grasp pose (still servo-limited, still
    the same top-grasp orientation) instead of aborting the round.
    """
    assert cycle.plan is not None
    try:
        cycle._move("pregrasp", cycle.plan.pregrasp, settle=False)
        return False
    except ContractError as exc:
        if not is_ik_denial(str(exc)):
            raise
        try:
            cycle._move("pregrasp_reduced", cycle.plan.grasp, settle=False)
        except ContractError as reduced_exc:
            raise ContractError(
                "PICK_PLACE_UNREACHABLE: pregrasp and grasp pose are both "
                "IK-unreachable; move the object towards the workspace centre "
                f"(x≈0.20-0.24 m, y≈±0.05 m). pregrasp: {exc}; grasp: {reduced_exc}"
            ) from reduced_exc
        event(
            "pregrasp_reduced",
            round_id=cycle.round_id,
            stage="pregrasp",
            reason=("nominal pregrasp is IK-unreachable; approaching the grasp pose directly"),
            error=str(exc),
        )
        return True


def move_to_observation(cycle: PickRound) -> bool:
    """Move to the observation pose; skip it when the vision-round pose is IK-unreachable.

    The observation pose is a camera convenience (D-095); the vision mode also
    refreshes at the pregrasp pose (``align_at_pregrasp``).  For an off-centre
    object the observation pose can be outside the arm's reachable top-grasp
    workspace even though the pregrasp/grasp poses are reachable, so a vision
    round falls through to the pregrasp instead of failing at the first move.
    Non-vision rounds keep the hard failure (no behaviour change).
    """
    assert cycle.plan is not None
    try:
        cycle._move("move_to_observation", cycle.plan.observation, check_workspace=False)
    except ContractError as exc:
        message = str(exc)
        denied = is_ik_denial(message)
        if not (cycle.settings.vision.enabled and denied):
            raise
        event(
            "observation_skipped",
            round_id=cycle.round_id,
            stage="move_to_observation",
            reason="observation pose is not reachable; refreshing at the pregrasp pose instead",
            error=message,
        )
        return False
    return True


def _parse_reprojection_px(message: str) -> float | None:
    """Best-effort ``reprojection_px`` from the refresh service JSON payload."""
    try:
        payload = json.loads(message)
    except (TypeError, ValueError):
        return None
    if not isinstance(payload, dict):
        return None
    value = payload.get('reprojection_px')
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if np.isfinite(number) else None


def robust_target(samples: list[np.ndarray]) -> tuple[np.ndarray, float, float]:
    """Median translation + hemisphere-aligned quaternion mean of target matrices."""
    if not samples:
        raise ContractError('robust_target requires at least one sample')
    poses = [to_pose(np.asarray(sample)) for sample in samples]
    positions = np.array([pose.position for pose in poses], dtype=float)
    median_position = np.median(positions, axis=0)
    reference = np.asarray(poses[0].orientation, dtype=float)
    quat_sum = np.zeros(4, dtype=float)
    for pose in poses:
        quat = np.asarray(pose.orientation, dtype=float)
        if float(quat @ reference) < 0.0:
            quat = -quat
        quat_sum += quat
    norm = float(np.linalg.norm(quat_sum))
    if not np.isfinite(norm) or norm < 1e-9:
        raise ContractError('observation samples have degenerate rotations')
    average_pose = Pose(tuple(float(v) for v in median_position), tuple(quat_sum / norm))
    position_spread = 0.0
    angle_spread = 0.0
    for pose in poses:
        delta_position, delta_angle = errors(pose, average_pose)
        position_spread = max(position_spread, delta_position)
        angle_spread = max(angle_spread, delta_angle)
    return from_pose(average_pose), position_spread, angle_spread


def observe_and_lock_target(cycle: PickRound) -> PickScene:
    """Sample the vision TF at the observation pose, reject unstable frames, lock it.

    The pregrasp view is often partially occluded by the gripper and a single
    square marker has two near-equivalent PnP branches; correcting from those
    views can jump the target by centimetres (D-131 real-machine failure).  Here
    every round takes `observe_samples` independent refreshes at the wide-view
    observation pose, drops samples with too-high re-projection error, requires
    the remaining samples to agree (median/quaternion-mean + spread gate), and
    plans once from that robust target for the rest of the round.

    S6g (field): the observation pose must be *stopped* before sampling and the
    samples must be spaced out — a still-settling wrist (wrist_flex residue
    0.084 rad ≈ 4.8°) tilts the camera and biases the object TF by ~2 cm, and
    fast consecutive refreshes locked samples taken while the arm was moving.
    Settle first, then wait ``observe_sample_interval_s`` between refreshes; the
    median/spread/re-projection gates are unchanged.

    S6i (field, run 20261112-123821): the vision node can still be on an old
    coasting/smoothed target right after the arm arrives — the first refresh was
    64 mm away from the next four stable ones and failed the spread gate.  So
    ``observe_warmup_samples`` refreshes are discarded after the settle (spaced by
    the same interval) and never enter the stability gate; a failed warmup refresh
    is ignored (the sampled frames still gate the round).
    """
    vision = cycle.settings.vision
    # S6g: let the arm stop moving before the first detection.
    _settle(cycle, vision.observe_settle_s)
    # S6i: flush the stale/coasting vision target before taking the real samples.
    warmup = vision.observe_warmup_samples
    if warmup:
        event(
            'vision_observe_warmup',
            round_id=cycle.round_id,
            stage='vision_observe',
            discarded=warmup,
        )
    for index in range(warmup):
        if index and vision.observe_sample_interval_s > 0:
            _settle(cycle, vision.observe_sample_interval_s)
        try:
            _refresh_scene_with_message(cycle, 'vision_observe_warmup')
        except ContractError as exc:
            event(
                'vision_observe_warmup_failed',
                round_id=cycle.round_id,
                stage='vision_observe_warmup',
                index=index,
                error=str(exc),
            )
    samples: list[np.ndarray] = []
    accepted = 0
    last_scene: PickScene | None = None
    for index in range(vision.observe_samples):
        if index and vision.observe_sample_interval_s > 0:
            # S6g: space the refreshes so every accepted sample comes from a still arm.
            _settle(cycle, vision.observe_sample_interval_s)
        scene, message = _refresh_scene_with_message(cycle, 'vision_observe')
        if scene.arm_to_target is None:
            raise ContractError(
                'vision_observe_unstable: the vision node did not publish a grasp_target TF'
            )
        last_scene = scene
        reprojection = _parse_reprojection_px(message)
        if reprojection is not None and reprojection > vision.observe_max_reprojection_px:
            event(
                'vision_observe_rejected',
                round_id=cycle.round_id,
                stage='vision_observe',
                reprojection_px=round(reprojection, 3),
                limit_px=vision.observe_max_reprojection_px,
            )
            continue
        samples.append(np.asarray(scene.arm_to_target, dtype=float))
        accepted += 1
    if len(samples) < 2:
        raise ContractError(
            'vision_observe_unstable: only '
            f'{accepted}/{vision.observe_samples} observation samples accepted '
            f'(re-projection limit {vision.observe_max_reprojection_px:g} px); '
            'check marker visibility/lighting and retry'
        )
    robust, position_spread, angle_spread = robust_target(samples)
    event(
        'vision_observe_stable',
        round_id=cycle.round_id,
        stage='vision_observe',
        samples=accepted,
        requested=vision.observe_samples,
        position_spread_m=round(position_spread, 6),
        angle_spread_rad=round(angle_spread, 6),
    )
    if (
        position_spread > vision.observe_stability_m
        or angle_spread > vision.observe_stability_rad
    ):
        raise ContractError(
            'vision_observe_unstable: observation samples disagree '
            f'(spread {position_spread * 1000:.2f} mm / {angle_spread:.4f} rad; '
            f'limits {vision.observe_stability_m * 1000:.1f} mm / '
            f'{vision.observe_stability_rad:.4f} rad); the arm was not moved towards '
            'the object'
        )
    assert last_scene is not None
    return dataclasses.replace(last_scene, arm_to_target=robust)


def _complete_observe_first(cycle: PickRound) -> None:
    """At the observation pose: sample/lock target → plan from the vision target TF."""
    assert cycle.arm is not None and cycle.plan is None
    cycle._stage("vision_observe")
    scene = observe_and_lock_target(cycle)
    cycle.plan = plan_pick(cycle.settings, scene)
    cycle._scene = scene
    event(
        "pick_plan",
        round_id=cycle.round_id,
        **pick_plan_fields(cycle.settings, scene, cycle.plan),
    )
    # User-facing console log: target object / place / grasp poses.
    print(format_pick_plan_log(cycle.settings, scene, cycle.plan), flush=True)


def observe_first_plan(cycle: PickRound) -> None:
    """Observe-first round start: fixed look-down pose → sample/lock target → plan.

    The folded rest pose may not see the marker and the arm_app top-grasp gate
    rejects arbitrary PlanRequest seeds, so no object TF is required before this
    move: the first observation pose is configured in the vision section
    (``observe_position_arm_m`` / ``observe_orientation_arm_xyzw``).  The target is
    sampled several times at that pose, filtered for re-projection errors and
    spread, and then locked for the rest of the round (D-132).  A failure here is
    a vision-stage failure (no grasp recovery), matching contract §6.2.
    """
    assert cycle.arm is not None and cycle.plan is None
    pose = first_observation_pose(cycle.settings)
    cycle._stage("vision_observe")
    cycle._move("vision_observe", pose, check_workspace=True)
    _complete_observe_first(cycle)


def observe_first_plan_from_joints(cycle: PickRound) -> None:
    """S6b/task-33：已在用户手动观察关节位（关节空间伺服到达）→ 直接 TF 快照/规划。

    不调用任何 TCP PlanRequest（rest 种子 /compute_ik 对该观察位报 IK_FAILED -31）；
    refresh/采样/锁定/规划语义与 ``observe_first_plan`` 相同。
    """
    assert cycle.arm is not None and cycle.plan is None
    _complete_observe_first(cycle)
