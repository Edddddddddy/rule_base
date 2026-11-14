"""Calibrated cube grasp/place geometry and observation pose for the SO-101 Pick station."""

from __future__ import annotations

import math
import re
from typing import Literal

from pydantic import Field, model_validator
from typing_extensions import Self

from rule_base.domain.models import Quaternion, Vector3

from .models import ArmSettings, Limits, Model, Ros2Settings


def _valid_tf_frame(name: str) -> bool:
    """A plain TF frame id (no leading '/', no empty segments, no whitespace)."""
    return (
        bool(name)
        and not name.startswith("/")
        and not name.endswith("/")
        and "//" not in name
        and not any(char.isspace() for char in name)
    )


class PickFrames(Model):
    root: str
    arm_base: str = Field(
        description="TF name of the arm command base; PlanRequest targets use this frame"
    )
    object: str | None = Field(
        default=None,
        description=(
            "TF name of the object/scene frame; required unless env_frames.enabled, where the "
            "object frame is derived from object_tf_prefix + target_object_id"
        ),
    )
    place: str | None = Field(
        default=None,
        description="TF name of the place target; configuring it enables the place stage",
    )
    grasp_target: str | None = Field(
        default=None,
        description=(
            "TF name of the target TCP in the ArUco object frame; configuring it makes the "
            "vision node's target pose the grasp pose instead of the cube-face heuristic"
        ),
    )

    @model_validator(mode="after")
    def validate_names(self) -> Self:
        names = (self.root, self.arm_base)
        optional = tuple(
            name for name in (self.object, self.place, self.grasp_target) if name is not None
        )
        if len(set(names + optional)) != len(names + optional) or any(
            not _valid_tf_frame(name) for name in names + optional
        ):
            raise ValueError("distinct nonblank Pick TF frames are required")
        return self


class PickProfile(Model):
    calibration_revision: str = Field(min_length=1)
    field_validated: bool = False
    cube_side_m: float = Field(gt=0)
    cube_axes_in_object_xyzw: Quaternion
    tcp_to_gripper_rotation_xyzw: Quaternion
    grasp_contact_offset_tcp_m: Vector3
    # S6j（现场）：观察位锁定一次抓取位置后，手眼/夹爪的横向系统偏差用 arm_base 下的
    # 固定平移补偿（只改位置、不改姿态）。用户现场反馈“抓取位置基本正确，向左移 1.5cm”：
    # base_link +Y 即机器人左侧。仅 vision grasp_target 路径使用，legacy 不受影响。
    grasp_target_offset_arm_m: Vector3 = Field(
        default=(0.0, 0.0, 0.0),
        description="Fixed arm-frame translation added to the vision grasp_target position "
        "(orientation unchanged; |each value| <= 0.05 m)",
    )
    reference_tcp_orientation_root_xyzw: Quaternion = Field(
        description="Reference world TCP orientation before horizontal cube alignment"
    )
    pregrasp_clearance_m: float = Field(gt=0, le=0.3)
    gripper_open_m: float = Field(gt=0, le=0.2)
    gripper_closed_m: float = Field(ge=0, le=0.2)
    grasp_min_width_m: float = Field(ge=0, description="Minimum contact width for a cube face")
    grasp_max_width_m: float = Field(gt=0, description="Maximum contact width for a cube face")
    # S6l（现场，2026-11-12；用户选择方案 A）：抓取确认改为**实测闭合宽度**的 live 接触判据。
    # 合爪后夹爪停在中位、没有真正全闭即认为两指间有物体：
    #   gripper_closed_m + grasp_contact_margin_m < 实测宽度 < gripper_open_m - 该 margin。
    # 固定方块区间 [grasp_min_width_m, grasp_max_width_m] 只作诊断/日志，不再决定成败。
    grasp_contact_margin_m: float = Field(
        default=0.0015,
        gt=0,
        le=0.02,
        description="Contact-evidence margin over gripper_closed_m and under gripper_open_m "
        "for the live measured grip width (the fixed cube interval is diagnostic only)",
    )
    # 确认抓取后进入 holding：实测宽度与「确认时刻记录的实测接触宽度」偏差超过该值即抓取丢失
    # （保持既有 lost-grasp 失败语义，不放宽容差）。
    grasp_width_tolerance_m: float = Field(
        default=0.002,
        gt=0,
        le=0.02,
        description="Holding tolerance around the measured contact width recorded at grasp "
        "confirmation; a larger deviation keeps the lost-grasp failure semantics",
    )
    post_grasp_position_arm_m: Vector3
    post_grasp_orientation_arm_xyzw: Quaternion
    # "Observation pose" seen before grasping: the grasp TCP point shifted by the
    # configured root-frame offset plus observation_clearance_m along root +Z,
    # keeping the fixed reference top-grasp orientation so the wrist camera looks
    # straight down at the cube. The offset compensates the nominal camera mount.
    observation_clearance_m: float = Field(
        default=0.06, gt=0, le=0.2, description="Observation height above the grasp TCP (root +Z)"
    )
    observation_offset_root_m: Vector3 = Field(
        default=(0.0, 0.0, 0.0),
        description="Root-frame offset of the observation pose (wrist-camera mount)",
    )
    # The place pose itself comes from frames.place (TF); only approach/retreat
    # clearances stay calibrated in the profile.
    place_approach_clearance_m: float = Field(
        default=0.04, gt=0, le=0.3, description="Place approach height above the place pose"
    )
    place_retreat_clearance_m: float = Field(
        default=0.04, gt=0, le=0.3, description="Lift after releasing at the place pose"
    )
    workspace_min_m: Vector3
    workspace_max_m: Vector3
    hold_duration_s: float = Field(default=2.0, ge=0, le=30)

    @model_validator(mode="after")
    def validate_grasp(self) -> Self:
        if not (
            self.gripper_closed_m
            < self.grasp_min_width_m
            <= self.cube_side_m
            <= self.grasp_max_width_m
            < self.gripper_open_m
            and self.grasp_min_width_m < self.grasp_max_width_m
        ):
            raise ValueError(
                "cube side must be within the grasp width interval below the open width; "
                "closure target must be below the minimum grasp width"
            )
        quaternions = {
            "cube_axes_in_object_xyzw": self.cube_axes_in_object_xyzw,
            "tcp_to_gripper_rotation_xyzw": self.tcp_to_gripper_rotation_xyzw,
            "reference_tcp_orientation_root_xyzw": self.reference_tcp_orientation_root_xyzw,
            "post_grasp_orientation_arm_xyzw": self.post_grasp_orientation_arm_xyzw,
        }
        for name, quaternion in quaternions.items():
            if not math.isclose(math.sqrt(sum(v * v for v in quaternion)), 1, abs_tol=1e-3):
                raise ValueError(f"{name} must be a unit xyzw quaternion")
        if any(a >= b for a, b in zip(self.workspace_min_m, self.workspace_max_m, strict=True)):
            raise ValueError("workspace bounds must be ordered")
        offset = self.observation_offset_root_m
        if (
            not all(math.isfinite(value) for value in offset)
            or math.dist(offset, (0.0, 0.0, 0.0)) > 0.2
        ):
            raise ValueError("observation_offset_root_m must be finite and at most 0.2 m long")
        grasp_offset = self.grasp_target_offset_arm_m
        if not all(math.isfinite(value) for value in grasp_offset) or any(
            abs(value) > 0.05 for value in grasp_offset
        ):
            raise ValueError(
                "grasp_target_offset_arm_m must be finite with every |value| <= 0.05 m"
            )
        if any(
            not low <= value <= high
            for value, low, high in zip(
                self.post_grasp_position_arm_m,
                self.workspace_min_m,
                self.workspace_max_m,
                strict=True,
            )
        ):
            raise ValueError("post-grasp position must be inside the configured workspace")
        return self

    @model_validator(mode="after")
    def validate_grasp_contact_window(self) -> Self:
        margin = self.grasp_contact_margin_m
        tolerance = self.grasp_width_tolerance_m
        if not (
            self.gripper_closed_m + margin < self.gripper_open_m - margin
            and margin <= self.grasp_min_width_m - self.gripper_closed_m
            and tolerance < self.gripper_open_m - self.gripper_closed_m
        ):
            raise ValueError(
                "grasp contact margins must leave a usable measured-width window: require "
                "gripper_closed_m + grasp_contact_margin_m < gripper_open_m - "
                "grasp_contact_margin_m, grasp_contact_margin_m <= grasp_min_width_m - "
                "gripper_closed_m, and grasp_width_tolerance_m < gripper_open_m - "
                "gripper_closed_m"
            )
        return self


class PickLimits(Limits):
    grasp_confirmation_timeout_s: float = Field(default=1.5, gt=0)
    max_grasp_attempts: int = Field(default=3, ge=1, le=3, strict=True)
    max_cube_tilt_rad: float = Field(default=math.radians(15), gt=0, le=math.pi / 4)
    recovery_feedback_timeout_s: float = Field(default=3.0, gt=0)
    recovery_feedback_stable_time_s: float = Field(default=0.2, gt=0)
    # Per-round home (folded rest): joint-error band plus stability window; the
    # timeout turns a blocked home into an explicit failure instead of a stall.
    home_tolerance_rad: float = Field(default=0.08, gt=0, le=0.3)
    home_timeout_s: float = Field(default=25, gt=0)

    @model_validator(mode="after")
    def validate_grasp_timing(self) -> Self:
        if self.grasp_confirmation_timeout_s <= self.stable_time_s:
            raise ValueError("grasp confirmation timeout must exceed the width stability window")
        if self.recovery_feedback_timeout_s <= self.recovery_feedback_stable_time_s:
            raise ValueError("recovery timeout must exceed the stability window")
        return self


#: SO-101 五臂关节硬件限位（rad；来自 URDF；gripper 不在其中）。用于观察关节位安全校验。
ARM_JOINT_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-1.91986, 1.91986),  # shoulder_pan
    (-1.6964, 1.74533),  # shoulder_lift
    (-1.69, 1.5323),  # elbow_flex
    (-1.74533, 1.74533),  # wrist_flex（真机实测可到 1.738；按硬件范围放宽到 ±1.74533）
    (-2.74385, 2.84121),  # wrist_roll（真机实测限位；π 不可达）
)


class VisionSettings(Model):
    """Bounded visual-servo loop around the vision node's target TF (contract §6).

    Disabled by default: without `frames.grasp_target` and this section the station
    keeps the legacy static/object-frame behaviour and emits no vision stages.
    """

    enabled: bool = False
    refresh_service: str = Field(
        default="/vision/refresh_object_pose",
        description="std_srvs/Trigger service that forces one detection cycle",
    )
    refresh_timeout_s: float = Field(
        default=3.0,
        gt=0,
        description="Deadline for the refresh call plus one new object/target TF sample",
    )
    max_corrections: int = Field(
        default=2,
        ge=1,
        le=3,
        strict=True,
        description="Bounded pregrasp corrections before an explicit alignment failure",
    )
    position_tolerance_m: float = Field(default=0.006, gt=0)
    orientation_tolerance_rad: float = Field(default=0.05, gt=0, lt=math.pi)
    correction_settle_s: float = Field(
        default=0.4,
        ge=0,
        le=10,
        description="Servo-kept settle time after a correction move before re-detecting",
    )
    # D-127: "observe first" round flow.  The object may not be visible from the
    # folded rest pose (and the arm_app top-grasp gate rejects arbitrary PlanRequest
    # seeds); move to a fixed, wide-view observation pose first, then detect and
    # plan from that view.  disabled = legacy "plan from initial TF" flow.
    observe_first: bool = Field(
        default=False,
        description="Move to a fixed observation pose and detect/plan there "
        "(vision target is not required before the first move)",
    )
    observe_position_arm_m: Vector3 = Field(
        default=(0.28, 0.0, 0.20),
        description="Fixed observation TCP position in frames.arm_base (m)",
    )
    # S6b（task-33）：用户手动观察位的关节角（5 臂关节，rad；gripper 保持实测）。
    # 配置后 observe_first 流程从 rest 直接关节空间伺服到该位（不走向上 IK PlanRequest，
    # 避免 rest 种子 IK_FAILED），到达后取 TF 快照再抓取；未配置则保持旧 IK 观察位路径。
    observe_joint_positions: tuple[float, ...] | None = Field(
        default=None,
        description="Optional 5 arm-joint observation target (rad). When set, "
        "observe_first moves there in joint space instead of planning "
        "the configured observe TCP pose; gripper stays measured.",
    )
    observe_orientation_arm_xyzw: Quaternion = Field(
        default=(0.7071067811865476, 0.0, -0.7071067811865476, 0.0),
        description="Fixed observation TCP orientation in frames.arm_base (xyzw; "
        "top-grasp reference, camera looking down at the workspace)",
    )
    # D-132: sample the vision TF at the observation pose, reject unstable frames,
    # lock one robust target for the whole round and stop applying pregrasp
    # corrections from later (often occluded / ambiguous) detections.
    observe_samples: int = Field(
        default=3,
        ge=1,
        le=5,
        strict=True,
        description="Number of vision refreshes sampled at the observation pose",
    )
    # S6i（run 20261112-123821 现场）：观察位刚到位时视觉节点仍有旧 coast 目标/平滑
    # 窗口未收敛，首帧 object TF 与后续帧差 64 mm，被稳定性门误杀。因此 settle 后先
    # 丢弃 observe_warmup_samples 帧预热（不计入 median/spread 统计），再采正式样本。
    observe_warmup_samples: int = Field(
        default=2,
        ge=0,
        le=5,
        strict=True,
        description="Observation refreshes discarded after settle before sampling "
        "(stale/coasting vision target; never enters the stability gate)",
    )
    # S6g（现场）：观察位必须**停稳 + 多次间隔采样确认 TF 稳定**后再锁定抓取。
    # wrist_flex 曾残留 0.084 rad（≈4.8°）使相机倾斜，物体 TF 出现 ~2 cm 系统偏差；
    # 观察关节到位容差因此收紧到 0.02 rad（home 仍用 limits.home_tolerance_rad）。
    observe_joint_tolerance_rad: float = Field(
        default=0.02,
        gt=0,
        le=0.08,
        description="Arrival tolerance of the joint-space observation move (rad); "
        "stricter than limits.home_tolerance_rad",
    )
    observe_settle_s: float = Field(
        default=0.8,
        ge=0,
        le=5,
        description="Servo-kept settle time at the observation pose before the first sample",
    )
    observe_sample_interval_s: float = Field(
        default=0.2,
        ge=0,
        le=1,
        description="Wait between observation refreshes so the arm and TF are stable",
    )
    observe_stability_m: float = Field(
        default=0.008,
        gt=0,
        le=0.05,
        description="Max accepted translation spread of the observation samples (m)",
    )
    observe_stability_rad: float = Field(
        default=0.02,
        gt=0,
        lt=math.pi,
        description="Max accepted rotation spread of the observation samples (rad)",
    )
    observe_max_reprojection_px: float = Field(
        default=1.5,
        gt=0,
        le=10,
        description="Reject observation samples whose refresh re-projection exceeds this",
    )
    lock_target_after_observation: bool = Field(
        default=False,
        description="Do not refresh/apply corrections after the observation target is "
        "locked (the grasp approach uses the locked plan)",
    )
    grasp_verify_enabled: bool = Field(
        default=False,
        description="One-shot detection at the pregrasp pose before descending to the grasp "
        "pose and closing the jaws (safety/diagnostic check; does not correct "
        "the locked plan)",
    )
    grasp_verify_tolerance_m: float = Field(
        default=0.020,
        gt=0,
        le=0.10,
        description="Max detected-vs-locked grasp position error at the pregrasp pose",
    )
    grasp_verify_tolerance_rad: float = Field(
        default=0.15,
        gt=0,
        lt=math.pi,
        description="Max detected-vs-locked grasp orientation error at the pregrasp pose",
    )

    @model_validator(mode="after")
    def validate_observe_first(self) -> Self:
        if self.observe_first and not self.enabled:
            raise ValueError("vision.observe_first requires vision.enabled=true")
        joints = self.observe_joint_positions
        if joints is not None:
            if not (self.enabled and self.observe_first):
                raise ValueError(
                    "vision.observe_joint_positions requires vision.enabled=true and "
                    "vision.observe_first=true"
                )
            if len(joints) != len(ARM_JOINT_LIMITS_RAD):
                raise ValueError(
                    f"vision.observe_joint_positions must hold {len(ARM_JOINT_LIMITS_RAD)} "
                    f"arm-joint values, got {len(joints)}"
                )
            if not all(math.isfinite(float(value)) for value in joints):
                raise ValueError("vision.observe_joint_positions must be finite")
            for index, (value, (lower, upper)) in enumerate(
                zip(joints, ARM_JOINT_LIMITS_RAD, strict=True)
            ):
                if not lower <= float(value) <= upper:
                    raise ValueError(
                        f"vision.observe_joint_positions[{index}]={value!r} outside "
                        f"hardware limit [{lower}, {upper}] rad"
                    )
        if not all(math.isfinite(value) for value in self.observe_position_arm_m):
            raise ValueError("vision.observe_position_arm_m must be finite")
        norm = math.sqrt(sum(value * value for value in self.observe_orientation_arm_xyzw))
        if not math.isclose(norm, 1.0, abs_tol=1e-3):
            raise ValueError(
                "vision.observe_orientation_arm_xyzw must be a unit xyzw quaternion, "
                f"|q|={norm:.6f}"
            )
        return self

    @model_validator(mode="after")
    def validate_service(self) -> Self:
        if (
            not self.refresh_service.startswith("/")
            or self.refresh_service.endswith("/")
            or "//" in self.refresh_service
            or any(c.isspace() for c in self.refresh_service)
        ):
            raise ValueError(
                "vision.refresh_service must be an absolute name such as "
                "/vision/refresh_object_pose"
            )
        return self


class EnvFramesSettings(Model):
    """Env position-subscription mode (third coordinate source, additive & default off).

    Instead of station_support static teach points (teach mode) or the wrist-ArUco
    vision node (vision mode), this mode consumes the environment node (env_app):
    the object id/pose comes from the dynamic ``/tf`` frame
    ``<object_tf_prefix><target_object_id>`` (parent ``frames.arm_base``, e.g. 20 Hz)
    and the place target from the ``/tf_static`` frame ``place_frame``.  The pose is
    read **once** after the arm reaches the fixed observation pose and the same
    snapshot is used for the whole round (no per-frame following, no camera/servo
    refresh dependency).  ``target_object_id`` may be overridden by the
    ``RULE_BASE_TARGET_OBJECT_ID`` environment variable (upstream selection; the
    026 launch-service pass-through is a later contract change).
    """

    enabled: bool = False
    object_tf_prefix: str = Field(
        default="env_marker_",
        description="TF child-frame prefix of the env object frame (env_app default)",
    )
    target_object_id: str = Field(
        default="",
        description="env_app marker/object id appended to object_tf_prefix; "
        "overridable by RULE_BASE_TARGET_OBJECT_ID",
    )
    place_frame: str = Field(
        default="place_target_01",
        description="tf_static place-target frame published by env_app (parent arm_base)",
    )
    grasp_offset_arm_m: Vector3 = Field(
        default=(0.0, 0.0, 0.0),
        description="Field-calibrated fixed arm-frame translation added to the env "
        "marker position before the top grasp (marker placement / contact-depth / "
        "extrinsics bias; orientation unchanged; |each value| <= 0.1 m).  This is "
        "the env-mode analogue of profile.grasp_target_offset_arm_m.",
    )
    snapshot_timeout_s: float = Field(
        default=2.0,
        gt=0,
        le=10.0,
        description="Deadline to obtain the env object+place TF snapshot at the "
        "observation pose before the round fails explicitly",
    )
    observe_joint_positions: tuple[float, ...] | None = Field(
        default=None,
        description="Optional 5 arm-joint observation target (rad); when set the env "
        "mode moves there in joint space and snapshots after arrival",
    )
    observe_joint_tolerance_rad: float = Field(
        default=0.02,
        gt=0,
        le=0.08,
        description="Arrival tolerance of the joint-space observation move (rad)",
    )
    observe_settle_s: float = Field(
        default=1.0,
        ge=0,
        le=5,
        description="Servo-kept settle time at the observation pose before the snapshot",
    )
    observe_position_arm_m: Vector3 = Field(
        default=(0.28, 0.0, 0.20),
        description="Fixed observation TCP position in frames.arm_base (m) when no "
        "observe_joint_positions is configured",
    )
    observe_orientation_arm_xyzw: Quaternion = Field(
        default=(0.7071067811865476, 0.0, -0.7071067811865476, 0.0),
        description="Fixed observation TCP orientation in frames.arm_base (xyzw)",
    )

    @property
    def object_frame(self) -> str:
        """The env object frame derived from prefix + target id."""
        return f"{self.object_tf_prefix}{self.target_object_id}"

    @model_validator(mode="after")
    def validate_env_frames(self) -> Self:
        prefix = self.object_tf_prefix
        if not _valid_tf_frame(f"{prefix}x") or not prefix:
            raise ValueError(
                "env_frames.object_tf_prefix must be a non-empty TF frame prefix "
                "(no whitespace, no '/', e.g. 'env_marker_')"
            )
        target_id = self.target_object_id
        if target_id != target_id.strip():
            raise ValueError("env_frames.target_object_id must not contain surrounding whitespace")
        if self.enabled and not target_id:
            raise ValueError(
                "env_frames.target_object_id is required when env_frames.enabled (or set "
                "RULE_BASE_TARGET_OBJECT_ID)"
            )
        if target_id and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_]*", target_id):
            raise ValueError(
                "env_frames.target_object_id must be a frame-safe token such as '1' "
                "(letters/digits/underscore, no leading '/', no whitespace)"
            )
        if self.enabled and not _valid_tf_frame(self.object_frame):
            raise ValueError(
                f"env_frames derived object frame {self.object_frame!r} is not a valid TF frame"
            )
        if not _valid_tf_frame(self.place_frame):
            raise ValueError(
                "env_frames.place_frame must be a valid TF frame id such as 'place_target_01'"
            )
        if self.enabled and self.object_frame == self.place_frame:
            raise ValueError(
                "env_frames derived object frame must differ from env_frames.place_frame"
            )
        joints = self.observe_joint_positions
        if joints is not None:
            if not self.enabled:
                raise ValueError("env_frames.observe_joint_positions requires enabled=true")
            if len(joints) != len(ARM_JOINT_LIMITS_RAD):
                raise ValueError(
                    f"env_frames.observe_joint_positions must hold "
                    f"{len(ARM_JOINT_LIMITS_RAD)} arm-joint values, got {len(joints)}"
                )
            if not all(math.isfinite(float(value)) for value in joints):
                raise ValueError("env_frames.observe_joint_positions must be finite")
            for index, (value, (lower, upper)) in enumerate(
                zip(joints, ARM_JOINT_LIMITS_RAD, strict=True)
            ):
                if not lower <= float(value) <= upper:
                    raise ValueError(
                        f"env_frames.observe_joint_positions[{index}]={value!r} outside "
                        f"hardware limit [{lower}, {upper}] rad"
                    )
        if not all(math.isfinite(value) for value in self.observe_position_arm_m):
            raise ValueError("env_frames.observe_position_arm_m must be finite")
        norm = math.sqrt(sum(value * value for value in self.observe_orientation_arm_xyzw))
        if not math.isclose(norm, 1.0, abs_tol=1e-3):
            raise ValueError(
                "env_frames.observe_orientation_arm_xyzw must be a unit xyzw quaternion, "
                f"|q|={norm:.6f}"
            )
        if not all(math.isfinite(float(value)) for value in self.grasp_offset_arm_m):
            raise ValueError("env_frames.grasp_offset_arm_m must be finite")
        if any(abs(float(value)) > 0.1 for value in self.grasp_offset_arm_m):
            raise ValueError(
                "env_frames.grasp_offset_arm_m must have |each value| <= 0.1 m "
                f"(got {self.grasp_offset_arm_m!r})"
            )
        return self


class PickSettings(Model):
    task: Literal["pick"]
    fixture_only: bool = True
    frames: PickFrames
    env_frames: EnvFramesSettings = Field(default_factory=EnvFramesSettings)
    profile: PickProfile
    limits: PickLimits = Field(default_factory=PickLimits)
    arm: ArmSettings = Field(default_factory=ArmSettings)
    ros2: Ros2Settings = Field(default_factory=Ros2Settings)
    vision: VisionSettings = Field(default_factory=VisionSettings)

    @property
    def object_frame(self) -> str | None:
        """Effective object frame: env-derived in env mode, else frames.object."""
        return self.env_frames.object_frame if self.env_frames.enabled else self.frames.object

    @property
    def place_frame(self) -> str | None:
        """Effective place frame: env tf_static frame in env mode, else frames.place."""
        return self.env_frames.place_frame if self.env_frames.enabled else self.frames.place

    @property
    def grasp_target_frame(self) -> str | None:
        """Effective vision target frame; env mode never uses a grasp_target TF."""
        return None if self.env_frames.enabled else self.frames.grasp_target

    @property
    def place_enabled(self) -> bool:
        """True when the place pose is read from TF (frames.place or env_frames)."""
        return self.env_frames.enabled or self.frames.place is not None

    @property
    def vision_enabled(self) -> bool:
        """True when the bounded visual-servo stages run (vision.enabled)."""
        return self.vision.enabled

    @model_validator(mode="after")
    def validate_command_frame(self) -> Self:
        # Plans are expressed in frames.arm_base, so the ROS 2 feedback/command
        # frame must be the same frame; otherwise poses would be misread.
        if self.ros2.base_frame != self.frames.arm_base:
            raise ValueError(
                "ros2.base_frame must equal frames.arm_base: plans are produced in "
                f"{self.frames.arm_base}, adapter TF reads {self.ros2.base_frame}"
            )
        if self.env_frames.enabled:
            if self.vision.enabled:
                raise ValueError(
                    "env_frames.enabled and vision.enabled are mutually exclusive: env mode "
                    "reads one /tf snapshot (no visual servo / refresh service)"
                )
            if self.frames.object is not None:
                raise ValueError(
                    "frames.object must be omitted when env_frames.enabled: the object frame "
                    f"is derived as {self.env_frames.object_frame!r}"
                )
            if self.frames.grasp_target is not None:
                raise ValueError(
                    "frames.grasp_target must be omitted when env_frames.enabled: env mode "
                    "uses the object TF position plus the top-grasp reference orientation"
                )
            if self.frames.place is not None:
                raise ValueError(
                    "frames.place must be omitted when env_frames.enabled: the place frame "
                    f"comes from env_frames.place_frame ({self.env_frames.place_frame!r})"
                )
            if self.env_frames.object_frame in (self.frames.root, self.frames.arm_base):
                raise ValueError(
                    "env_frames derived object frame must differ from frames.root/arm_base"
                )
        elif self.frames.object is None:
            raise ValueError(
                "frames.object is required unless env_frames.enabled supplies the object frame"
            )
        if self.vision.enabled and self.frames.grasp_target is None:
            raise ValueError(
                "vision.enabled requires frames.grasp_target (the target TCP TF published "
                "by the vision node); the servo loop cannot correct an object-frame heuristic"
            )
        return self
