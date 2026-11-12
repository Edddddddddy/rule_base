"""Execute one guarded approach, grasp, lift and optional place cycle."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import asdict
from threading import Event
from typing import Literal, Protocol

from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import (
    ArmFeedbackError,
    ArmSessionError,
    ContractError,
    Feedback,
    Interrupted,
    PickArm,
    PickPlan,
    PickResult,
    PickScene,
    Pose,
)
from rule_base.infrastructure.transform.math import (
    compose,
    errors,
    from_pose,
    inverse,
    offset,
    rotate,
    split_line,
    to_pose,
)
from rule_base.observability.events import event

from . import joint_motion, vision_servo
from .planning import format_pick_plan_log, pick_plan_fields, plan_pick, workspace


class PickEnvironment(Protocol):
    def snapshot(self) -> PickScene: ...
    def request_new_observation(self) -> None: ...
    def check_health(self) -> None: ...
    def refresh_object_pose(self) -> str: ...
    def sample_age_s(self) -> float | None: ...


class PickRound:
    def __init__(
        self,
        settings: PickSettings,
        environment: PickEnvironment,
        arm_factory: Callable[[], PickArm],
        stop: Event,
    ) -> None:
        self.settings = settings
        self.environment = environment
        self.arm_factory = arm_factory
        self.stop_requested = stop
        self.stage = "validate"
        self.round_id = ""
        self.arm: PickArm | None = None
        self.plan: PickPlan | None = None
        self._scene: PickScene | None = None
        self.attempt = 0
        self.holding = False
        # Measured gripper width recorded when the close stage confirmed a grasp
        # (S6l). While unset the live contact window decides; once set, holding uses
        # this measured contact reference instead of the fixed cube-width interval.
        self._grasp_width_m: float | None = None
        self._gripper_target_m: float | None = None
        self._last_gripper_send = 0.0
        self._grasp_lost_since: float | None = None
        self._servo_target: Pose | None = None
        self._last_servo_send = 0.0

    def _stage(self, name: str) -> None:
        self.stage = name
        event("stage", round_id=self.round_id, stage=name, attempt=self.attempt)

    def run(self, round_id: str) -> PickResult:
        self.stage = "validate"
        self.round_id = round_id
        self.arm = None
        self.plan = None
        self._scene = None
        self.attempt = 0
        self.holding = False
        self._grasp_width_m = None
        self._gripper_target_m = None
        self._last_gripper_send = 0.0
        self._grasp_lost_since = None
        self._servo_target = None
        self._last_servo_send = 0.0
        status: Literal["completed", "failed", "interrupted"] = "failed"
        message = ""
        cleanup_errors: list[str] = []
        try:
            self._cancel()
            if not (self.settings.vision.enabled and self.settings.vision.observe_first):
                # Legacy flow: plan from the object TF visible at the start pose.
                # With observe_first the round deliberately plans only after the
                # fixed observation move + vision refresh (D-127).
                scene = self.environment.snapshot()
                self.plan = plan_pick(self.settings, scene)
                self._scene = scene
                event(
                    "pick_plan",
                    round_id=round_id,
                    **pick_plan_fields(self.settings, scene, self.plan),
                )
                # User-facing console log: target object / place / grasp poses.
                print(format_pick_plan_log(self.settings, scene, self.plan), flush=True)
            self.arm = self.arm_factory()
            self._execute()
            status, message = (
                "completed",
                "object grasped, moved to place pose, released"
                if self.settings.place_enabled
                else "object grasped, moved to post-grasp pose, held and released",
            )
        except Interrupted as exc:
            status, message = "interrupted", str(exc)
        except Exception as exc:
            status, message = "failed", str(exc)
        finally:
            if self.arm is not None:
                for name, action in (
                    ("stop", self.arm.stop),
                    ("release", self.arm.release),
                    ("close", self.arm.close),
                ):
                    try:
                        action()
                    except Exception as exc:
                        cleanup_errors.append(f"{name}: {exc}")
            if cleanup_errors:
                status = "failed"
                message += "; cleanup unconfirmed: " + "; ".join(cleanup_errors)
        result = PickResult(round_id, status, self.stage, message, not cleanup_errors)
        event("round_result", **asdict(result), grasp_attempts=self.attempt)
        return result

    def _cancel(self) -> None:
        if self.stop_requested.is_set():
            raise Interrupted("stop requested; proceeding to IDLE and release cleanup")

    def _pause(self, seconds: float) -> None:
        if self.stop_requested.wait(seconds):
            self._cancel()

    def _check(self, *, check_workspace: bool = True) -> Feedback:
        self._cancel()
        assert self.arm is not None
        self._check_servo_deadline("before feedback")
        feedback = self.arm.feedback()
        self._check_servo_deadline("feedback")
        if check_workspace:
            workspace(feedback.pose, self.settings)
        if self.holding:
            now = time.monotonic()
            if self._grasped(feedback):
                self._grasp_lost_since = None
            else:
                if self._grasp_lost_since is None:
                    self._grasp_lost_since = now
                    self._report_grasp_feedback(feedback, lost_for_s=0.0)
                lost_for_s = now - self._grasp_lost_since
                if lost_for_s >= self.settings.limits.stable_time_s:
                    diagnostics = self._report_grasp_feedback(feedback, lost_for_s=lost_for_s)
                    raise ContractError(
                        "grasp evidence lost while holding; "
                        f"lost_for_s={lost_for_s:.3f}; grasp_feedback={diagnostics}"
                    )
        if (
            self._gripper_target_m is not None
            and time.monotonic() - self._last_gripper_send
            >= 1 / self.settings.limits.gripper_hold_hz
        ):
            self._command_gripper(self._gripper_target_m)
        if self._servo_target is not None:
            self._send_servo(self._servo_target)
        return feedback

    def _command_gripper(self, width_m: float) -> None:
        assert self.arm is not None
        # A command can reach the device even if its send acknowledgement fails.
        self._gripper_target_m = width_m
        self.arm.gripper(width_m, self.settings.limits.gripper_effort)
        self._last_gripper_send = time.monotonic()
        self._check_servo_deadline("gripper command")

    def _initialize_servo(self) -> None:
        assert self.arm is not None
        self._stage("initialize_servo")
        started = time.monotonic()
        # Only confirm the Servo session: acquisition already switched the server
        # into Servo. Turning the measured pose into a refreshed PlanRequest target
        # was rejected by the real back-end top-grasp gate (D-090), so it is gone.
        # The arm may still be at the folded rest pose, outside the Pick workspace.
        hold_pose = self._check(check_workspace=False).pose
        self.arm.start_servo(hold_pose)
        self._cancel()
        elapsed = time.monotonic() - started
        if elapsed > self.settings.limits.stage_timeout_s:
            raise ContractError("Servo initialization timed out")
        event("servo_ready", round_id=self.round_id, initialization_s=elapsed)

    def _check_servo_deadline(self, operation: str) -> None:
        if self._servo_target is None:
            return
        gap = time.monotonic() - self._last_servo_send
        limit = self.settings.limits.servo_max_gap_s
        if gap > limit:
            event(
                "servo_deadline_missed",
                round_id=self.round_id,
                stage=self.stage,
                operation=operation,
                gap_s=gap,
                max_gap_s=limit,
            )
            raise ArmSessionError(
                f"Servo refresh deadline missed during {operation}: {gap:.3f}s > {limit:.3f}s"
            )

    def _send_servo(self, target: Pose) -> None:
        assert self.arm is not None
        self._cancel()
        self._check_servo_deadline("before send")
        workspace(target, self.settings)
        started = time.monotonic()
        self.arm.move(target)
        self._servo_target = target
        self._last_servo_send = started
        self._check_servo_deadline("send")

    def _home(self, stage: str) -> None:
        """Stream the folded rest joint target until the measured arm joints arrive.

        The gripper is held at its measured angle: homing must never move the jaws.
        Joint streaming deliberately bypasses the Pick TCP workspace gate (the folded
        rest pose is outside the grasp workspace) and stays interruptible.
        """
        self._move_joints_to(
            self.settings.ros2.home_joint_positions, stage=stage,
            event_name="home_complete", feedback_name="home_feedback")

    def _move_joints_to(
        self,
        target_joint_positions: tuple[float, ...],
        *,
        stage: str,
        event_name: str = "joint_move_complete",
        feedback_name: str = "joint_move_feedback",
        tolerance_rad: float | None = None,
        timeout_s: float | None = None,
    ) -> None:
        """Joint-space move (shared home/observe implementation in ``joint_motion``)."""
        joint_motion.stream_joints(
            self, target_joint_positions, stage=stage, event_name=event_name,
            feedback_name=feedback_name, tolerance_rad=tolerance_rad,
            timeout_s=timeout_s, emit=event)

    def _joints_settled(self, feedback: Feedback) -> bool:
        return bool(feedback.joint_velocities) and all(
            abs(value) <= self.settings.limits.motion_joint_velocity_rad_s
            for value in feedback.joint_velocities
        )

    def _reached(
        self, feedback: Feedback, target: Pose, *, require_joint_velocity: bool = True
    ) -> bool:
        distance, angle = errors(feedback.pose, target)
        limits = self.settings.limits
        return (
            distance <= limits.position_tolerance_m
            and angle <= limits.orientation_tolerance_rad
            and (not require_joint_velocity or self._joints_settled(feedback))
        )

    def _motion_diagnostics(
        self, feedback: Feedback, target: Pose, *, require_joint_velocity: bool = True
    ) -> dict[str, object]:
        limits = self.settings.limits
        distance, angle = errors(feedback.pose, target)
        checks = {
            "position": distance <= limits.position_tolerance_m,
            "orientation": angle <= limits.orientation_tolerance_rad,
            "joint_velocity": not require_joint_velocity or self._joints_settled(feedback),
        }
        return {
            "target_position_m": target.position,
            "actual_position_m": feedback.pose.position,
            "position_error_m": distance,
            "position_tolerance_m": limits.position_tolerance_m,
            "orientation_error_rad": angle,
            "orientation_tolerance_rad": limits.orientation_tolerance_rad,
            "max_joint_velocity_rad_s": max(map(abs, feedback.joint_velocities), default=None),
            "motion_joint_velocity_rad_s": limits.motion_joint_velocity_rad_s,
            "waiting_for": [name for name, ok in checks.items() if not ok] or ["stable_time"],
        }

    def _wait(
        self,
        predicate: Callable[[Feedback], bool],
        *,
        target: Pose | None = None,
        stable_time_s: float = 0.0,
        deadline: float | None = None,
        timeout_s: float | None = None,
        check_workspace: bool = True,
    ) -> tuple[Feedback, float]:
        """Block until ``predicate`` held for ``stable_time_s``.

        Returns the confirming feedback and the observed stable duration so a
        caller (close_gripper confirmation) can report an accurate
        ``stable_for_s`` in its active event.  Timeout/failure semantics are
        unchanged: the only other exit is the stage-timeout ``ContractError``.
        """
        started = time.monotonic()
        if deadline is None:
            deadline = started + (timeout_s or self.settings.limits.stage_timeout_s)
        else:
            started = deadline - (timeout_s or self.settings.limits.stage_timeout_s)
        stable_since: float | None = None
        width_min = width_max = 0.0
        next_report = started
        last_diagnostics: dict[str, object] | None = None
        while True:
            iteration = time.monotonic()
            feedback = self._check(check_workspace=check_workspace)
            now = time.monotonic()
            if predicate(feedback):
                if self.stage == "close_gripper":
                    width = feedback.gripper_width_m
                    if stable_since is None:
                        width_min = width_max = width
                    width_min, width_max = min(width_min, width), max(width_max, width)
                    if width_max - width_min > 0.0005:
                        stable_since = None
                        width_min = width_max = width
                if stable_since is None:
                    stable_since = now
                if now < deadline and now - stable_since >= stable_time_s:
                    return feedback, now - stable_since
            else:
                stable_since = None
            if now >= next_report or now >= deadline:
                if target is not None:
                    last_diagnostics = self._motion_diagnostics(
                        feedback, target, require_joint_velocity=stable_time_s > 0
                    )
                    event_name = "motion_feedback"
                elif self.stage == "close_gripper":
                    last_diagnostics = self._grasp_diagnostics(feedback)
                    last_diagnostics["width_span_m"] = width_max - width_min
                    last_diagnostics["width_stability_tolerance_m"] = 0.0005
                    event_name = "grasp_feedback"
                elif self.stage in {"wait_open_gripper", "release_gripper", "retry_open_gripper"}:
                    last_diagnostics = {
                        "width_m": feedback.gripper_width_m,
                        "target_width_m": self.settings.profile.gripper_open_m,
                        "width_error_m": abs(
                            feedback.gripper_width_m - self.settings.profile.gripper_open_m
                        ),
                        "width_tolerance_m": 0.002,
                        "width_ok": self._gripper_open(feedback),
                        "velocity_m_s": feedback.gripper_velocity_m_s,
                        "effort": feedback.gripper_effort,
                    }
                    event_name = "gripper_feedback"
                else:
                    event_name = ""
                if last_diagnostics is not None and event_name:
                    event(
                        event_name,
                        round_id=self.round_id,
                        stage=self.stage,
                        **last_diagnostics,
                        stable_for_s=0.0 if stable_since is None else now - stable_since,
                        required_stable_s=stable_time_s,
                        elapsed_s=now - started,
                    )
                next_report = now + 1.0
            if now >= deadline:
                detail = (
                    f"; {event_name}={last_diagnostics}" if last_diagnostics is not None else ""
                )
                raise ContractError(f"stage timed out: {self.stage}{detail}")
            self._pause(
                max(0.0, 1 / self.settings.limits.servo_hz - (time.monotonic() - iteration))
            )

    def _move(
        self, stage: str, target: Pose, *, settle: bool = True, check_workspace: bool = True
    ) -> None:
        self._stage(stage)
        feedback = self._check(check_workspace=check_workspace)
        if self.holding and not self._grasped(feedback):
            # Keep the current grasp target until contact feedback has recovered.
            self._wait(
                self._grasped,
                stable_time_s=self.settings.limits.stable_time_s,
                check_workspace=check_workspace,
            )
        self._send_servo(target)
        self._wait(
            lambda feedback: self._reached(feedback, target, require_joint_velocity=settle),
            target=target,
            stable_time_s=self.settings.limits.motion_stable_time_s if settle else 0,
            check_workspace=check_workspace,
        )

    def _move_linear(
        self, stage: str, target: Pose, *, start: Pose | None = None, settle: bool = True
    ) -> None:
        self._stage(stage)
        limits = self.settings.limits
        deadline = time.monotonic() + limits.stage_timeout_s
        feedback = self._check()
        if self.holding and not self._grasped(feedback):
            self._wait(self._grasped, stable_time_s=limits.stable_time_s, deadline=deadline)
            feedback = self._check()
        step_m = min(limits.linear_step_m, limits.linear_speed_m_s / limits.servo_hz)
        if start is not None and not self._reached(feedback, start, require_joint_velocity=False):
            raise ContractError("TCP left the planned pregrasp pose before vertical descent")
        start_pose = feedback.pose if start is None else start
        positions = split_line(start_pose.position, target.position, step_m)
        event(
            "linear_motion",
            round_id=self.round_id,
            stage=stage,
            start_pose_arm=asdict(start_pose),
            actual_start_pose_arm=asdict(feedback.pose),
            target_pose_arm=asdict(target),
            step_m=step_m,
            waypoint_count=len(positions),
            linear_speed_m_s=limits.linear_speed_m_s,
            servo_hz=limits.servo_hz,
        )
        for index, position in enumerate(positions):
            iteration = time.monotonic()
            self._check()
            if time.monotonic() >= deadline:
                raise ContractError(f"stage timed out: {self.stage}")
            self._send_servo(Pose(position, target.orientation))
            if index < len(positions) - 1:
                self._pause(max(0.0, 1 / limits.servo_hz - (time.monotonic() - iteration)))
        self._wait(
            lambda current: self._reached(current, target, require_joint_velocity=settle),
            target=target,
            stable_time_s=limits.motion_stable_time_s if settle else 0,
            deadline=deadline,
        )

    def _grasped(self, feedback: Feedback) -> bool:
        """True when the measured grip width proves contact with an object.

        Before confirmation (``_grasp_width_m`` unset) the jaws must stop between
        the closed and the open target: wider than ``gripper_closed_m`` by
        ``grasp_contact_margin_m`` (not fully closed) and narrower than
        ``gripper_open_m`` by the same margin (not still fully open).  After
        confirmation the live width must stay within ``grasp_width_tolerance_m`` of
        the measured contact width recorded at confirmation; a larger deviation
        keeps the existing lost-grasp failure semantics.  The fixed cube-width
        interval (``grasp_min/max_width_m``) is diagnostic only.
        """
        profile = self.settings.profile
        width = feedback.gripper_width_m
        if self._grasp_width_m is None:
            margin = profile.grasp_contact_margin_m
            return (
                profile.gripper_closed_m + margin
                < width
                < profile.gripper_open_m - margin
            )
        return abs(width - self._grasp_width_m) <= profile.grasp_width_tolerance_m

    def _grasp_diagnostics(self, feedback: Feedback) -> dict[str, object]:
        assert self.plan is not None
        p = self.plan
        profile = self.settings.profile
        width = feedback.gripper_width_m
        margin = profile.grasp_contact_margin_m
        return {
            "width_m": width,
            "velocity_m_s": feedback.gripper_velocity_m_s,
            "effort": feedback.gripper_effort,
            "expected_cube_width_m": p.expected_cube_width_m,
            "min_width_m": p.grasp_min_width_m,
            "max_width_m": p.grasp_max_width_m,
            # S6l live contact diagnostics: the fixed cube interval above and
            # ``width_in_expected_interval`` are diagnostic only; ``width_ok`` now
            # follows the measured contact evidence.
            "contact_threshold_m": profile.gripper_closed_m + margin,
            "contact_ceiling_m": profile.gripper_open_m - margin,
            "contact_width_m": self._grasp_width_m,
            "grasp_width_tolerance_m": profile.grasp_width_tolerance_m,
            "width_in_expected_interval": p.grasp_min_width_m <= width <= p.grasp_max_width_m,
            "width_ok": self._grasped(feedback),
        }

    def _report_grasp_feedback(self, feedback: Feedback, **fields: object) -> dict[str, object]:
        diagnostics = self._grasp_diagnostics(feedback)
        event(
            "grasp_feedback",
            round_id=self.round_id,
            stage=self.stage,
            **diagnostics,
            **fields,
        )
        return diagnostics

    def _gripper_open(self, feedback: Feedback) -> bool:
        return abs(feedback.gripper_width_m - self.settings.profile.gripper_open_m) <= 0.002

    def _execute_attempt(self) -> None:
        assert self.arm is not None and self.plan is not None
        p, limits = self.settings.profile, self.settings.limits
        # Every attempt starts without contact evidence (live close-contact mode).
        self._grasp_width_m = None
        self._grasp_lost_since = None
        self._stage("open_gripper")
        self._check()
        self._command_gripper(p.gripper_open_m)
        reduced = vision_servo.move_to_pregrasp(self)
        vision_servo.align_or_lock_at_pregrasp(self, reduced)
        self._stage("wait_open_gripper")
        self._wait(self._gripper_open)
        # S6e: verify the locked target at the pregrasp pose, where the marker is
        # still visible; the close-range view at the grasp pose is occluded by the
        # gripper and its re-detection jumped by 70 mm on the real machine.  The
        # final grasp is covered by the close-width gate instead.
        if reduced:
            # The arm is already at the grasp pose (no separate pregrasp view).
            event(
                "grasp_verify_skipped",
                round_id=self.round_id,
                stage="pregrasp_verify",
                reason="reduced pregrasp fallback",
            )
        else:
            vision_servo.verify_grasp_target(self)
        # Keep the pre-motion plan: the arm can occlude the object during approach.
        # With the reduced fallback the arm is already at the grasp pose.
        self._move_linear(
            "grasp_pose",
            self.plan.grasp,
            start=self.plan.grasp if reduced else self.plan.pregrasp,
            settle=False,
        )
        self._stage("close_gripper")
        self._check()
        close_started = time.monotonic()
        self._command_gripper(p.gripper_closed_m)
        _, stable_for_s = self._wait(
            self._grasped,
            stable_time_s=limits.stable_time_s,
            deadline=close_started + limits.grasp_confirmation_timeout_s,
            timeout_s=limits.grasp_confirmation_timeout_s,
        )
        # S6l（用户现场确认，2026-11-12）：确认抓取以**实测闭爪宽度**停在闭合与张开目标
        # 之间为准，固定方块区间只作诊断。确认后再取一帧反馈，把该实测接触宽度锁为 holding
        # 参考，并主动发一条 grasp_feedback：L6 `capture_rounds.judge_scripted` 取最后一条的
        # width/width_ok；旧实现只留下 close 起点（≈张开宽度 0.0799）样本，导致确实夹住物体
        # 也被判 task_success=false。确认后按既有 place 流程继续，不再中途松爪丢弃。
        contact = self._check()
        self._grasp_width_m = contact.gripper_width_m
        self.holding = True
        self._grasp_lost_since = None
        diagnostics = self._grasp_diagnostics(contact)
        event(
            "grasp_feedback",
            round_id=self.round_id,
            stage=self.stage,
            **diagnostics,
            stable_for_s=stable_for_s,
            required_stable_s=limits.stable_time_s,
            elapsed_s=time.monotonic() - close_started,
        )
        if not diagnostics["width_in_expected_interval"]:
            # The calibrated cube interval no longer decides success; record the
            # deviation so field/offline post-analysis can spot calibration drift.
            event(
                "grasp_width_outside_expected",
                round_id=self.round_id,
                stage=self.stage,
                width_m=contact.gripper_width_m,
                expected_cube_width_m=self.plan.expected_cube_width_m,
                min_width_m=self.plan.grasp_min_width_m,
                max_width_m=self.plan.grasp_max_width_m,
                contact_threshold_m=diagnostics["contact_threshold_m"],
                contact_ceiling_m=diagnostics["contact_ceiling_m"],
                width_in_expected_interval=False,
            )

    def _recover_failed_grasp(self, failure: Exception) -> None:
        assert self.arm is not None and self._scene is not None
        failed_stage = self.stage
        event(
            "recovery_started",
            round_id=self.round_id,
            attempt=self.attempt,
            failed_stage=failed_stage,
            error=str(failure),
        )
        # Width loss must not recursively abort recovery. Preserve the commanded
        # gripper width until the return pose is reached: an object may remain held.
        self.holding = False
        self._grasp_lost_since = None
        restart_session = isinstance(failure, (ArmFeedbackError, ArmSessionError))
        if self._servo_target is None:
            restart_session = True
        else:
            restart_session |= (
                time.monotonic() - self._last_servo_send > self.settings.limits.servo_max_gap_s
            )
        try:
            actual = self.arm.feedback().pose
        except ContractError:
            restart_session = True
        if restart_session:
            self._stage("retry_restore_session")
            self._servo_target = None
            self.arm.recover(
                self.settings.limits.recovery_feedback_timeout_s,
                self.settings.limits.recovery_feedback_stable_time_s,
            )
            self._cancel()
            actual = self.arm.feedback().pose
            workspace(actual, self.settings)
            self.arm.start_servo(actual)
            self._servo_target = actual
            self._last_servo_send = time.monotonic()
        else:
            # Freeze at the measured position, never refresh the failed waypoint.
            self._send_servo(actual)
        initial = Pose(
            self.settings.profile.post_grasp_position_arm_m,
            self.settings.profile.post_grasp_orientation_arm_xyzw,
        )
        needs_retreat = failed_stage in {
            "grasp_pose",
            "close_gripper",
            "move_to_post_grasp",
            "hold_object",
            # Failure while lowering to the place pose leaves the object near the
            # table; retreat vertically before the lateral return to the initial pose.
            "lower_to_place",
        }
        if needs_retreat and not self._reached(
            self._check(), initial, require_joint_velocity=False
        ):
            actual = self._check().pose
            upward = rotate(
                inverse(self._scene.root_to_arm),
                (0, 0, self.settings.profile.pregrasp_clearance_m),
            )
            # Shorten the vertical retreat at the TCP workspace boundary. Never
            # invent an out-of-workspace escape target or a zero-clearance move.
            scale = 1.0
            profile = self.settings.profile
            for position, delta, low, high in zip(
                actual.position,
                upward,
                profile.workspace_min_m,
                profile.workspace_max_m,
                strict=True,
            ):
                if delta > 0:
                    scale = min(scale, (high - position) / delta)
                elif delta < 0:
                    scale = min(scale, (low - position) / delta)
            if scale * profile.pregrasp_clearance_m <= self.settings.limits.position_tolerance_m:
                raise ContractError("No vertical retreat clearance inside the TCP workspace")
            dx, dy, dz = upward
            retreat = offset(actual, (dx * scale, dy * scale, dz * scale))
            workspace(retreat, self.settings)
            self._move_linear("retry_retreat", retreat)
        self._move_linear("retry_return_initial", initial)
        self._stage("retry_open_gripper")
        self._command_gripper(self.settings.profile.gripper_open_m)
        self._wait(self._gripper_open)
        event("recovery_completed", round_id=self.round_id, attempt=self.attempt)

    def _reidentify(self) -> None:
        assert self._scene is not None
        previous = (
            to_pose(compose(self._scene.root_to_arm, from_pose(self.plan.grasp))).orientation
            if self.plan is not None
            else None
        )
        self.plan = None
        self._stage("reidentify")
        self.environment.request_new_observation()
        deadline = time.monotonic() + self.settings.limits.warmup_timeout_s
        while True:
            iteration = time.monotonic()
            self._check()
            self.environment.check_health()
            try:
                scene = self.environment.snapshot()
                plan = plan_pick(self.settings, scene, previous_grasp_root=previous)
            except (ContractError, ValueError) as exc:
                if time.monotonic() >= deadline:
                    raise ContractError(
                        f"timed out waiting for a new Pick observation after return: {exc}"
                    ) from exc
                self._pause(
                    max(0.0, 1 / self.settings.limits.servo_hz - (time.monotonic() - iteration))
                )
                continue
            if time.monotonic() >= deadline:
                raise ContractError("timed out waiting for a new Pick observation after return")
            self.plan = plan
            self._scene = scene
            event(
                "pick_retry_plan",
                round_id=self.round_id,
                attempt=self.attempt,
                **pick_plan_fields(self.settings, scene, self.plan),
            )
            return

    def _execute(self) -> None:
        assert self.arm is not None
        assert self.plan is not None or (
            self.settings.vision.enabled and self.settings.vision.observe_first
        )
        limits = self.settings.limits
        for attempt in range(1, limits.max_grasp_attempts + 1):
            self.attempt = attempt
            event(
                "grasp_attempt",
                round_id=self.round_id,
                attempt=attempt,
                maximum=limits.max_grasp_attempts,
            )
            try:
                if attempt == 1:
                    self._stage("connect")
                    assert self.arm is not None
                    self.arm.connect()
                    # The arm may start at the folded rest pose, outside the Pick
                    # TCP workspace; home_start is what brings it to the rest.
                    self._check(check_workspace=False)
                    self._stage("acquire_control")
                    self.arm.acquire()
                    self._home("home_start")
                    self._initialize_servo()
                    if self.settings.vision.enabled and self.settings.vision.observe_first:
                        observe_joints = self.settings.vision.observe_joint_positions
                        if observe_joints is not None:
                            # task-33：rest → 关节空间到用户手动观察位（绕开 IK_FAILED），
                            # 到达后 refresh/TF 快照→抓取；不调用 observe_first_plan 的 TCP 移动。
                            # S6g：观察位到达用更严的 observe_joint_tolerance_rad（home 不变）。
                            vision = self.settings.vision
                            self._move_joints_to(
                                observe_joints,
                                stage="vision_observe_joints",
                                event_name="vision_observe_joints_complete",
                                feedback_name="vision_observe_joints_feedback",
                                tolerance_rad=vision.observe_joint_tolerance_rad,
                            )
                            vision_servo.observe_first_plan_from_joints(self)
                        else:
                            vision_servo.observe_first_plan(self)
                    elif vision_servo.move_to_observation(self):
                        vision_servo.align_at_observation(self)
                else:
                    self._reidentify()
                self._execute_attempt()
                self._finish_attempt()
                self._home("home_end")
                return
            except Interrupted:
                raise
            except Exception as exc:
                failed_stage = self.stage
                if (
                    failed_stage
                    in {"home_start", "home_end", "grasp_verify", "pregrasp_verify"}
                    or failed_stage.startswith("vision_")
                    or vision_servo.is_ik_denial(str(exc))
                ):
                    # Homing is not a grasp attempt: the grasp recovery/retry path
                    # would command pick poses unrelated to the folded rest, so a
                    # home failure (timeout etc.) fails the round explicitly.
                    # Vision alignment / IK-unreachable poses are likewise input or
                    # workspace failures: fail without inventing recovery motion
                    # (contract §6.2 failure semantics).
                    raise
                event(
                    "grasp_attempt_failed",
                    round_id=self.round_id,
                    attempt=attempt,
                    failed_stage=failed_stage,
                    error=str(exc),
                )
                try:
                    self._recover_failed_grasp(exc)
                except Interrupted:
                    raise
                except Exception as recovery_error:
                    event(
                        "recovery_failed",
                        round_id=self.round_id,
                        attempt=attempt,
                        failed_stage=failed_stage,
                        recovery_stage=self.stage,
                        error=str(recovery_error),
                    )
                    raise ContractError(
                        f"{failed_stage}: {exc}; recovery failed at {self.stage}: {recovery_error}"
                    ) from recovery_error
                if attempt >= limits.max_grasp_attempts:
                    self._stage("grasp_attempts_exhausted")
                    raise ContractError(f"grasp failed after {attempt} attempts; {exc}") from exc

    def _release_object(self) -> None:
        p, limits = self.settings.profile, self.settings.limits
        # Opening the jaws is only allowed while contact evidence is still present;
        # a lost grasp fails here instead of silently dropping the object.
        if not self._grasped(self._check()):
            self._wait(self._grasped, stable_time_s=limits.stable_time_s)
        self._stage("release_gripper")
        self.holding = False
        self._command_gripper(p.gripper_open_m)
        self._wait(self._gripper_open)

    def _finish_attempt(self) -> None:
        p, limits = self.settings.profile, self.settings.limits
        post_grasp_pose = Pose(p.post_grasp_position_arm_m, p.post_grasp_orientation_arm_xyzw)
        self._move_linear("move_to_post_grasp", post_grasp_pose)
        if not self.settings.place_enabled:
            # Legacy profile (frames.place absent): hold at the post-grasp pose, then
            # release there. This keeps old configs and the mock chain unchanged.
            self._stage("hold_object")
            hold_until = time.monotonic() + p.hold_duration_s
            while time.monotonic() < hold_until:
                self._check()
                self._pause(min(1 / limits.servo_hz, max(0.0, hold_until - time.monotonic())))
            self._release_object()
            return
        assert self.plan is not None
        place = self.plan.place
        if place is None:
            raise ContractError("Pick place frame is configured but the plan has no place pose")
        # Approach from directly above the place pose: same XY, +Z in the command frame.
        approach = offset(place, (0.0, 0.0, p.place_approach_clearance_m))
        self._move_linear("move_to_place_approach", approach)
        self._move_linear("lower_to_place", place)
        self._release_object()
        # Retreat along root +Z so the just-released object is not dragged sideways.
        assert self._scene is not None
        upward = rotate(inverse(self._scene.root_to_arm), (0.0, 0.0, p.place_retreat_clearance_m))
        self._move_linear("retreat_from_place", offset(place, upward))
