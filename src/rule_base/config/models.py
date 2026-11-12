"""Strict configuration models; all physical defaults are offline candidates."""

from __future__ import annotations

import math

from pydantic import BaseModel, ConfigDict, Field, model_validator
from typing_extensions import Self

# D-009: the gripper width(m) <-> joint angle(rad) map is shared with the arm server.
DEFAULT_GRIPPER_MAX_ANGLE_RAD = 1.74533
DEFAULT_GRIPPER_WIDTH_PER_RAD = 0.1 / DEFAULT_GRIPPER_MAX_ANGLE_RAD
DEFAULT_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class Model(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)


class Limits(Model):
    tf_age_s: float = Field(default=0.3, gt=0)
    tf_skew_s: float = Field(default=0.05, ge=0)
    tf_future_tolerance_s: float = Field(default=0, ge=0, le=0.2)
    stage_timeout_s: float = Field(default=15, gt=0)
    warmup_timeout_s: float = Field(default=10, gt=0)
    stable_time_s: float = Field(default=0.5, gt=0)
    motion_stable_time_s: float = Field(default=0.15, gt=0)
    motion_joint_velocity_rad_s: float = Field(default=0.02, gt=0, le=0.1)
    servo_hz: float = Field(default=50, ge=1, le=100)
    linear_step_m: float = Field(default=0.02, gt=0, le=1)
    linear_speed_m_s: float = Field(default=0.05, gt=0, le=1)
    servo_max_gap_s: float = Field(default=0.15, gt=0)
    position_tolerance_m: float = Field(default=0.003, gt=0)
    orientation_tolerance_rad: float = Field(default=0.05, gt=0, lt=math.pi)
    platform_drift_m: float = Field(default=0.002, gt=0)
    platform_drift_rad: float = Field(default=0.01, gt=0)
    gripper_effort: float = Field(default=8, gt=0, le=100)
    gripper_hold_hz: float = Field(default=10, gt=0, le=50)

    @model_validator(mode="after")
    def validate_timing(self) -> Self:
        if self.servo_max_gap_s <= 1 / self.servo_hz:
            raise ValueError("servo_max_gap_s must exceed one servo period")
        return self


class ArmSettings(Model):
    """Authority/lease policy and feedback timing; the ROS 2 transport performs I/O."""

    rpc_timeout_ms: int = Field(default=1000, ge=10, le=10000)
    cleanup_rpc_timeout_ms: int | None = Field(default=None, ge=10, le=10000)
    acquire_control: bool = True
    manage_lease: bool = Field(default=True, strict=True)
    feedback_timeout_s: float = Field(default=0.3, gt=0, le=2)
    state_timeout_s: float = Field(default=1.5, gt=0, le=5)


class GripperSettings(Model):
    """Linear width(m) <-> gripper joint angle(rad) map; must match the arm server (D-009)."""

    width_per_rad: float = Field(default=DEFAULT_GRIPPER_WIDTH_PER_RAD, gt=0)
    max_width_m: float = Field(default=0.1, gt=0)
    min_angle_rad: float = Field(default=0.0, ge=-math.pi, le=math.pi)
    max_angle_rad: float = Field(default=DEFAULT_GRIPPER_MAX_ANGLE_RAD, gt=0, le=math.pi)

    @model_validator(mode="after")
    def validate_angles(self) -> Self:
        if self.min_angle_rad >= self.max_angle_rad:
            raise ValueError("gripper minimum angle must be below maximum")
        return self

    def width_from_angle(self, angle_rad: float) -> float:
        """D-009: width_m = clamp(angle_rad * k, 0, max_width_m)."""
        return min(max(angle_rad * self.width_per_rad, 0.0), self.max_width_m)

    def angle_from_width(self, width_m: float) -> float:
        """D-009: angle_rad = clamp(width_m / k, min_angle_rad, max_angle_rad)."""
        return min(max(width_m / self.width_per_rad, self.min_angle_rad), self.max_angle_rad)


class Ros2Settings(Model):
    """ROS 2 channel names, frames and SO-101 parameters for the command adapter."""

    prefix: str = "/arm"
    base_frame: str = "base_link"
    tcp_frame: str = "gripper_frame_link"
    joint_names: tuple[str, ...] = DEFAULT_JOINT_NAMES
    gripper: GripperSettings = Field(default_factory=GripperSettings)
    home_joint_positions: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    recover_to_home: bool = False

    @model_validator(mode="after")
    def validate_channels(self) -> Self:
        if not self.prefix.startswith("/") or self.prefix.endswith("/") or "//" in self.prefix:
            raise ValueError("ros2.prefix must be an absolute name such as /arm")
        if any(c.isspace() for c in self.prefix):
            raise ValueError("ros2.prefix must not contain whitespace")
        names = self.joint_names
        if (
            len(names) != 6
            or len(set(names)) != 6
            or any(not name or any(c.isspace() for c in name) for name in names)
        ):
            raise ValueError("ros2.joint_names must be six distinct nonblank SO-101 joint names")
        if self.tcp_frame == self.base_frame:
            raise ValueError("ros2 tcp_frame must differ from base_frame")
        if any(
            not name or any(c.isspace() for c in name) for name in (self.base_frame, self.tcp_frame)
        ):
            raise ValueError("ros2 frames must be nonblank TF frame IDs")
        if len(self.home_joint_positions) != len(names) or not all(
            math.isfinite(value) for value in self.home_joint_positions
        ):
            raise ValueError("ros2.home_joint_positions must hold one finite value per joint")
        return self

    @property
    def joint_states_topic(self) -> str:
        return f"{self.prefix}/joint_states"

    @property
    def joint_command_topic(self) -> str:
        return f"{self.prefix}/joint_command"

    @property
    def state_topic(self) -> str:
        return f"{self.prefix}/state"

    @property
    def plan_request_service(self) -> str:
        return f"{self.prefix}/plan_request"

    @property
    def switch_state_service(self) -> str:
        return f"{self.prefix}/switch_state"

    @property
    def acquire_authority_service(self) -> str:
        return f"{self.prefix}/authority/acquire"

    @property
    def release_authority_service(self) -> str:
        return f"{self.prefix}/authority/release"

    @property
    def gripper_joint_index(self) -> int:
        """The gripper is the last commanded joint in the frozen six-joint order."""
        return len(self.joint_names) - 1
