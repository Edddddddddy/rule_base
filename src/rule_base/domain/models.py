"""Transport-independent observations, plans and execution contracts; SI units."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Protocol

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Matrix = tuple[tuple[float, ...], ...]


class ContractError(RuntimeError):
    """Unsafe, unavailable or inconsistent input at an owner boundary."""


class InactiveLeaseError(ContractError):
    """Authority release rejected because the local lease is no longer active."""


class ArmFeedbackError(ContractError):
    """Missing or stale Arm feedback; movement needs fresh observations."""


class ArmSessionError(ContractError):
    """The current Servo session cannot continue without recovery."""


class Interrupted(RuntimeError):
    """Cooperative cancellation, distinct from hardware stop confirmation."""


@dataclass(frozen=True)
class Pose:
    position: Vector3
    orientation: Quaternion


@dataclass(frozen=True)
class Scene:
    panel_id: int
    arm_to_handle: Matrix
    root_to_arm: Matrix


@dataclass(frozen=True)
class OpenPlan:
    panel_id: int
    calibration_revision: str
    pull_distance_m: float
    pregrasp: Pose
    grasp: Pose
    pull_end: Pose
    pull_axis: Vector3
    retreat_offset: Vector3


@dataclass(frozen=True)
class Feedback:
    pose: Pose
    joint_positions: tuple[float, ...]
    gripper_width_m: float
    gripper_velocity_m_s: float
    gripper_effort: float
    joint_velocities: tuple[float, ...]


@dataclass(frozen=True)
class RoundResult:
    round_id: str
    status: Literal["completed", "failed", "interrupted"]
    stage: str
    message: str
    commanded_pull_distance_m: float
    actual_pull_distance_m: float | None
    cleanup_confirmed: bool


@dataclass(frozen=True)
class PickResult:
    round_id: str
    status: Literal["completed", "failed", "interrupted"]
    stage: str
    message: str
    cleanup_confirmed: bool


@dataclass(frozen=True)
class PickScene:
    root_to_arm: Matrix
    arm_to_object: Matrix
    arm_to_place: Matrix | None = None
    # Vision target-only extension (kept last for positional-argument compatibility):
    # the target TCP expressed in the arm command frame, read through the ArUco chain
    # arm_base <- objects_cube_01 <- grasp_target.
    arm_to_target: Matrix | None = None


@dataclass(frozen=True)
class PickPlan:
    grasp: Pose
    pregrasp: Pose
    observation: Pose
    calibration_revision: str
    expected_cube_width_m: float
    grasp_min_width_m: float
    grasp_max_width_m: float
    place: Pose | None = None


RoundOutcome = RoundResult | PickResult


class Environment(Protocol):
    def snapshot(self) -> Scene: ...


class SnapshotEnvironment(Protocol):
    def snapshot(self) -> Scene | PickScene: ...

    def robot_ready(self) -> bool: ...


class LifecycleTransport(Protocol):
    def open(self) -> None: ...
    def close(self) -> None: ...
    def check_health(self) -> None: ...


class Arm(Protocol):
    def connect(self) -> None: ...
    def acquire(self) -> None: ...
    def feedback(self) -> Feedback: ...
    def start_servo(self, hold_pose: Pose) -> None: ...
    def move(self, target: Pose) -> None: ...
    def move_joints(self, positions: tuple[float, ...]) -> None: ...
    def gripper(self, width_m: float, effort: float) -> None: ...
    def stop(self) -> None: ...
    def release(self) -> None: ...
    def close(self) -> None: ...


class PressArm(Arm, Protocol):
    def confirm_idle(self) -> None: ...


class PickArm(Arm, Protocol):
    def recover(self, timeout_s: float, stable_time_s: float) -> None: ...
