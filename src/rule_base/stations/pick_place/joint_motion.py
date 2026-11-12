"""Joint-space servo streaming for the Pick station（S6b/task-33 拆出，保持单文件 <800 行）。

`_home` 与「rest → 用户手动观察关节位」共用同一套关节流伺服：
- 20 Hz 流式 `move_joints`（gripper 保持实测角，不动爪）；
- 到达判据 = 各臂关节误差 ≤ tolerance（默认 home_tolerance_rad）且稳定
  motion_stable_time_s；
- 超时明确失败；循环内仍检查健康/中断（由 PickRound._check 负责），可被 stop/Ctrl-C 打断。
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import Any

from rule_base.domain.models import ContractError
from rule_base.observability.events import event


def stream_joints(
    cycle: Any,
    target_joint_positions: tuple[float, ...],
    *,
    stage: str,
    event_name: str = "joint_move_complete",
    feedback_name: str = "joint_move_feedback",
    tolerance_rad: float | None = None,
    timeout_s: float | None = None,
    emit: Callable[..., None] | None = None,
) -> None:
    """Stream an arm joint target until measured joints arrive (shared home/observe).

    ``emit`` lets the caller pass its module-level ``event`` (tests monkeypatch it and
    existing home-event contracts keep working); defaults to the observability sink.
    """
    emit = emit or event
    arm = cycle.arm
    assert arm is not None
    cycle._stage(stage)
    cycle._servo_target = None
    cycle._gripper_target_m = None
    limits = cycle.settings.limits
    spec = cycle.settings.ros2
    gripper_index = spec.gripper_joint_index
    target = tuple(float(value) for value in target_joint_positions)
    if len(target) == len(spec.joint_names):
        target = target[:gripper_index]      # 兼容完整 6 关节（gripper 位忽略/保持实测）
    if len(target) != gripper_index:
        raise ContractError(
            f"stage {stage}: joint target must hold {gripper_index} arm values "
            f"(or {len(spec.joint_names)} with gripper), got {len(target)}")
    if not all(math.isfinite(value) for value in target):
        raise ContractError(f"stage {stage}: joint target must be finite")
    tolerance = float(limits.home_tolerance_rad if tolerance_rad is None else tolerance_rad)
    timeout_s = float(limits.home_timeout_s if timeout_s is None else timeout_s)
    started = time.monotonic()
    deadline = started + timeout_s
    stable_since: float | None = None
    next_report = started
    last_diagnostics: dict[str, object] | None = None
    while True:
        iteration = time.monotonic()
        feedback = cycle._check(check_workspace=False)
        now = time.monotonic()
        command = (*target, feedback.joint_positions[gripper_index])
        arm.move_joints(command)
        joint_errors = tuple(
            abs(feedback.joint_positions[index] - target[index])
            for index in range(gripper_index)
        )
        max_error = max(joint_errors, default=0.0)
        if max_error <= tolerance:
            if stable_since is None:
                stable_since = now
            if now < deadline and now - stable_since >= limits.motion_stable_time_s:
                emit(event_name, round_id=cycle.round_id, stage=stage,
                     elapsed_s=now - started)
                return
        else:
            stable_since = None
        if now >= next_report or now >= deadline:
            last_diagnostics = {
                "joint_errors_rad": joint_errors,
                "max_error_rad": max_error,
                "tolerance_rad": tolerance,
                "elapsed_s": now - started,
            }
            emit(feedback_name, round_id=cycle.round_id, stage=stage,
                     **last_diagnostics)
            next_report = now + 1.0
        if now >= deadline:
            detail = (f"; {feedback_name}={last_diagnostics}"
                      if last_diagnostics is not None else "")
            raise ContractError(f"stage timed out: {stage}{detail}")
        cycle._pause(max(0.0, 1 / limits.servo_hz - (time.monotonic() - iteration)))
