"""Env position-subscription round step: observation pose + one TF snapshot.

Third Pick coordinate source (additive, default off): the object id/pose comes from
the env node's dynamic ``env_marker_<target_object_id>`` frame and the place target
from its ``/tf_static`` ``place_target_01`` frame.  The pose is read **once** after
the arm reaches the configured observation pose and the snapshot is reused for the
rest of the round (no per-frame following, no visual servo, no refresh service).

The stage names/events are additive (``env_observe_joints`` / ``env_observe`` /
``env_snapshot`` / ``env_snapshot_wait`` / ``pick_plan``); failures are explicit
(``env snapshot timed out ...``) and never fake a completed round.
"""

from __future__ import annotations

import time
from dataclasses import asdict
from typing import Any

from rule_base.domain.models import ContractError, Pose
from rule_base.infrastructure.transform.math import to_pose
from rule_base.observability.events import event
from rule_base.stations.pick_place.planning import (
    format_pick_plan_log,
    pick_plan_fields,
    plan_pick,
)

#: Report a pending snapshot at most this often (s) while the TF is not ready.
_SNAPSHOT_RETRY_REPORT_S = 1.0
#: Snapshot polling period (s); the TF buffer lookup itself is cheap.
_SNAPSHOT_POLL_S = 0.05


def hold_servo(cycle: Any, seconds: float) -> None:
    """Hold the current Servo target (or just stay healthy) for ``seconds``.

    TCP observation moves have a live Servo target and must be re-streamed at
    ``limits.servo_hz`` (a plain sleep would trip the Servo refresh deadline);
    joint-space observation moves have no Servo target and only need health/stop
    checks.
    """
    limits = cycle.settings.limits
    deadline = time.monotonic() + seconds
    next_send = time.monotonic()
    while True:
        iteration = time.monotonic()
        cycle._check(check_workspace=False)
        target = cycle._servo_target
        if target is not None and time.monotonic() >= next_send:
            cycle._send_servo(target)
            next_send = time.monotonic() + 1 / limits.servo_hz
        if time.monotonic() >= deadline:
            return
        cycle._pause(max(0.0, 1 / limits.servo_hz - (time.monotonic() - iteration)))


def observe_and_plan(cycle: Any) -> None:
    """Move to the env observation pose, settle, snapshot once and plan the round."""
    env = cycle.settings.env_frames
    joints = env.observe_joint_positions
    if joints is not None:
        cycle._move_joints_to(
            joints,
            stage="env_observe_joints",
            event_name="env_observe_joints_complete",
            feedback_name="env_observe_joints_feedback",
            tolerance_rad=env.observe_joint_tolerance_rad,
        )
    else:
        cycle._move(
            "env_observe",
            Pose(env.observe_position_arm_m, env.observe_orientation_arm_xyzw),
            check_workspace=False,
        )
    hold_servo(cycle, env.observe_settle_s)
    snapshot_and_plan(cycle)


def snapshot_and_plan(
    cycle: Any,
    *,
    event_name: str = "pick_plan",
    extra_fields: dict[str, object] | None = None,
) -> None:
    """Read one env TF snapshot (bounded) and plan the whole round from it."""
    env = cycle.settings.env_frames
    timeout_s = env.snapshot_timeout_s
    started = time.monotonic()
    deadline = started + timeout_s
    next_report = started
    cycle._stage("env_snapshot")
    while True:
        cycle._cancel()
        cycle._check(check_workspace=False)
        try:
            scene = cycle.environment.snapshot()
        except (ContractError, ValueError) as exc:
            now = time.monotonic()
            if now >= next_report:
                event(
                    "env_snapshot_wait",
                    round_id=cycle.round_id,
                    stage="env_snapshot",
                    target_object_id=env.target_object_id,
                    object_frame=env.object_frame,
                    error=str(exc),
                    elapsed_s=now - started,
                )
                next_report = now + _SNAPSHOT_RETRY_REPORT_S
            if now >= deadline:
                raise ContractError(
                    f"env snapshot timed out after {now - started:.3f}s "
                    f"(timeout_s={timeout_s:g}): {exc}"
                ) from exc
            cycle._pause(min(_SNAPSHOT_POLL_S, max(0.0, deadline - now)))
            continue
        cycle.plan = plan_pick(cycle.settings, scene)
        cycle._scene = scene
        event(
            "env_snapshot",
            round_id=cycle.round_id,
            stage="env_snapshot",
            target_object_id=env.target_object_id,
            object_frame=env.object_frame,
            place_frame=env.place_frame,
            object_pose_arm=asdict(to_pose(scene.arm_to_object)),
            place_pose_arm=(
                asdict(to_pose(scene.arm_to_place)) if scene.arm_to_place is not None else None
            ),
            elapsed_s=time.monotonic() - started,
        )
        fields: dict[str, object] = {}
        if extra_fields:
            fields.update(extra_fields)
        event(
            event_name,
            round_id=cycle.round_id,
            **fields,
            **pick_plan_fields(cycle.settings, scene, cycle.plan),
        )
        print(format_pick_plan_log(cycle.settings, scene, cycle.plan), flush=True)
        return
