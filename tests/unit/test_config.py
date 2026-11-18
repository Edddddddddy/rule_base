"""Purpose: Ensure unsafe configs and unknown tasks fail before any I/O.

Usage: source ros2_ws/setup_ros_env.sh && python -m pytest tests/unit/test_config.py
"""

import math
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

from rule_base.config.load import load_settings, task_type
from rule_base.config.models import (
    DEFAULT_GRIPPER_MAX_ANGLE_RAD,
    DEFAULT_GRIPPER_WIDTH_PER_RAD,
    DEFAULT_JOINT_NAMES,
    GripperSettings,
    Ros2Settings,
)
from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import Pose
from rule_base.infrastructure.transform.math import from_pose

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "configs/pick_place/offline.yaml"
LIVE = ROOT / "configs/pick_place/live.yaml"
REAL = ROOT / "configs/pick_place/live_ros2.yaml"
VISION = ROOT / "configs/pick_place/live_ros2_vision.yaml"

# D-047 frozen geometry: the real top grasp before/after the 180° wrist roll (xyzw).
TOP_GRASP_REFERENCE_XYZW = (0.0, 0.7071067811865476, 0.0, 0.7071067811865476)
TOP_GRASP_ROLL180_XYZW = (0.7071067811865476, 0.0, -0.7071067811865476, 0.0)
# S6a（用户确认）：真实视觉链路腕部 180° 用于观测/参考姿态；折叠 rest 的 wrist_roll=0
# （真机限位 [-2.74385, 2.84121]，π 不可达，写 π 会使 home_start 超时）。
WRIST180_REFERENCE_XYZW = (0.0, -0.7071067811865476, 0.0, -0.7071067811865476)
WRIST180_REST_JOINTS = (0.0, -1.52, 1.45, 0.75, 0.0, 0.0)
# S6b（task-29）：观察位=高位置看全局（wrist_roll≈-1.5 与用户 5 号电机角度一致）；
# 抓取 reference/post_grasp 回原顶抓精确朝下 q0（TOP_GRASP_ROLL180_XYZW）；rest 仍 0。
HIGH_OBSERVE_POSITION_ARM_M = (0.24613, 0.015309, 0.109946)
HIGH_OBSERVE_ORIENTATION_ARM_XYZW = (0.740458356, 0.045132322, -0.668923796, 0.047174696)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate(quaternion, vector):
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    conjugate = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    return quat_multiply(quat_multiply(quaternion, vector_quaternion), conjugate)[:3]


def top_grasp_orientation(reference):
    """D-047: roll the reference top grasp 180° about the TCP approach axis (+X)."""
    return quat_multiply(reference, (1.0, 0.0, 0.0, 0.0))


def test_top_grasp_rolls_180_degrees_about_the_tcp_approach_axis():
    rolled = top_grasp_orientation(TOP_GRASP_REFERENCE_XYZW)
    assert rolled == pytest.approx(TOP_GRASP_ROLL180_XYZW)
    # The approach axis (TCP +X) is invariant: it still points at root -Z.
    assert rotate(TOP_GRASP_REFERENCE_XYZW, (1, 0, 0)) == pytest.approx((0, 0, -1), abs=1e-9)
    assert rotate(rolled, (1, 0, 0)) == pytest.approx((0, 0, -1), abs=1e-9)
    # The closing axis (TCP +Y) moves to the other side of the jaws.
    assert rotate(TOP_GRASP_REFERENCE_XYZW, (0, 1, 0)) == pytest.approx((0, 1, 0), abs=1e-9)
    assert rotate(rolled, (0, 1, 0)) == pytest.approx((0, -1, 0), abs=1e-9)
    # A second 180° roll restores the original rotation (quaternion sign aside).
    twice = np.asarray(from_pose(Pose((0, 0, 0), top_grasp_orientation(rolled))))[:3, :3]
    once = np.asarray(from_pose(Pose((0, 0, 0), TOP_GRASP_REFERENCE_XYZW)))[:3, :3]
    assert twice == pytest.approx(once)


def test_offline_and_live_configs_load():
    offline = load_settings(CONFIG)
    live = load_settings(LIVE)
    real = load_settings(REAL)
    assert all(isinstance(item, PickSettings) for item in (offline, live, real))
    assert task_type(CONFIG) == task_type(LIVE) == task_type(REAL) == "pick"


def test_live_ros2_real_template_is_conservative():
    settings = load_settings(REAL)
    assert settings.frames.root == "world"
    assert settings.frames.arm_base == "base_link"
    assert settings.frames.object == "objects_cube_01"
    assert settings.ros2.prefix == "/arm"
    assert settings.ros2.base_frame == "base_link"
    # Frozen real-stack contract: the p7-compatible top-grasp TCP frame.
    assert settings.ros2.tcp_frame == "grasp_tcp_link"
    # 真机保守时序：放慢目标刷新、减小步进/速度、放宽容差、失败不自动重试。
    limits = settings.limits
    assert limits.servo_hz == pytest.approx(20)
    assert limits.servo_max_gap_s > 1 / limits.servo_hz
    # S6b（task-25）：真机 WSL/5V 发送抖动容差 0.45s（仍 < arm_app command_timeout 500ms）
    assert limits.servo_max_gap_s == pytest.approx(0.45)
    assert limits.linear_speed_m_s == pytest.approx(0.05)
    assert limits.linear_step_m == pytest.approx(0.01)
    assert limits.position_tolerance_m == pytest.approx(0.01)
    assert limits.orientation_tolerance_rad == pytest.approx(0.1)
    assert limits.stage_timeout_s == pytest.approx(10)
    assert limits.stable_time_s == pytest.approx(0.2)
    assert limits.max_grasp_attempts == 1
    assert settings.arm.feedback_timeout_s == pytest.approx(0.5)
    assert settings.arm.state_timeout_s == pytest.approx(2.0)
    # D-093/D-094 per-round home (folded rest) and TF-driven place stage。
    # S6a：动作/观测腕部 180°；折叠 rest wrist_roll=0（π 超限位）。
    assert settings.ros2.home_joint_positions == pytest.approx(WRIST180_REST_JOINTS)
    assert limits.home_tolerance_rad == pytest.approx(0.08)
    assert limits.home_timeout_s == pytest.approx(30)
    assert settings.frames.place == "place_target_01"
    assert settings.place_enabled


def test_live_ros2_profile_keeps_top_grasp_geometry():
    settings = load_settings(REAL)
    profile = settings.profile
    # S6b（task-29）：抓取参考/后抓回原顶抓精确朝下 q0（观察位单独用高位置姿态）。
    assert profile.reference_tcp_orientation_root_xyzw == pytest.approx(
        TOP_GRASP_ROLL180_XYZW
    )
    assert profile.post_grasp_orientation_arm_xyzw == pytest.approx(
        TOP_GRASP_ROLL180_XYZW
    )
    rotation = np.asarray(
        from_pose(Pose((0.0, 0.0, 0.0), profile.reference_tcp_orientation_root_xyzw))
    )
    # top-grasp 门：TCP +X 精确朝下、闭合轴水平（approach/closing tilt 均为 0）。
    assert rotation[:3, 0] == pytest.approx((0.0, 0.0, -1.0), abs=1e-6)
    assert rotation[:3, 1] == pytest.approx((0.0, -1.0, 0.0), abs=1e-6)
    assert math.acos(float(np.clip(-rotation[2, 0], -1.0, 1.0))) == pytest.approx(0.0, abs=1e-6)
    assert math.asin(float(np.clip(abs(rotation[2, 1]), 0.0, 1.0))) == pytest.approx(0.0, abs=1e-6)
    # D-095: lift 6.4 cm after the grasp, then pull back 2 cm (IK margin 0.19 rad).
    assert profile.post_grasp_position_arm_m == pytest.approx((0.26, 0.0, 0.05))
    # Observation view: fixed reference orientation + nominal wrist-camera mount offset.
    assert profile.observation_clearance_m == pytest.approx(0.06)
    assert profile.observation_offset_root_m == pytest.approx((-0.039, 0.0281, 0.0))
    # post-grasp 点必须落在业务层 workspace 盒内（pydantic 已校验，这里锁定模板值）。
    assert all(
        low <= value <= high
        for value, low, high in zip(
            profile.post_grasp_position_arm_m,
            profile.workspace_min_m,
            profile.workspace_max_m,
            strict=True,
        )
    )
    assert profile.grasp_min_width_m <= profile.cube_side_m <= profile.grasp_max_width_m
    # S6l live measured-width confirmation: defaults fit the real 1.5 cm profile and
    # the fixed cube interval is diagnostic only.
    assert profile.grasp_contact_margin_m == pytest.approx(0.0015)
    assert profile.grasp_width_tolerance_m == pytest.approx(0.002)
    assert (
        profile.gripper_closed_m + profile.grasp_contact_margin_m
        < profile.gripper_open_m - profile.grasp_contact_margin_m
    )
    assert profile.grasp_contact_margin_m <= (
        profile.grasp_min_width_m - profile.gripper_closed_m
    )
    assert profile.grasp_width_tolerance_m < (
        profile.gripper_open_m - profile.gripper_closed_m
    )


def test_live_ros2_place_stage_comes_from_the_tf_place_frame():
    settings = load_settings(REAL)
    profile = settings.profile
    assert settings.frames.place == "place_target_01"
    assert settings.place_enabled
    # The place pose is no longer a profile field; only its clearances stay calibrated.
    assert not hasattr(profile, "place_position_arm_m")
    assert not hasattr(profile, "place_orientation_arm_xyzw")
    # 2026-11-17 平台链验证提交（6401700）把现场 clearance 改为 0.02（profile 注释预留恢复 0.04，
    # 但 HEAD 实际值 0.02）；测试与仓库当前现场标定值同步，不改业务配置。
    assert profile.place_approach_clearance_m == pytest.approx(0.02)
    assert profile.place_retreat_clearance_m == pytest.approx(0.02)
    assert profile.calibration_revision.endswith("obs-high-wristroll-m150-v11")


@pytest.mark.parametrize("config", [CONFIG, LIVE])
def test_profiles_without_a_place_frame_keep_the_legacy_behavior(config):
    settings = load_settings(config)
    profile = settings.profile
    assert settings.frames.place is None
    assert not settings.place_enabled
    assert profile.place_approach_clearance_m == pytest.approx(0.04)
    assert profile.place_retreat_clearance_m == pytest.approx(0.04)
    assert profile.observation_clearance_m == pytest.approx(0.06)
    assert profile.observation_offset_root_m == pytest.approx((0.0, 0.0, 0.0))


@pytest.mark.parametrize(
    "place", ["world", "base_link", "objects_cube_01", "", "place target 01"]
)
def test_frames_place_must_be_a_distinct_nonblank_frame(place):
    values = load_settings(REAL).model_dump()
    values["frames"]["place"] = place
    with pytest.raises(ValueError):
        PickSettings.model_validate(values)


def test_frames_place_enables_the_tf_place_stage():
    values = load_settings(CONFIG).model_dump()
    values["frames"]["place"] = "place_target_01"
    settings = PickSettings.model_validate(values)
    assert settings.place_enabled
    assert settings.frames.place == "place_target_01"


def test_frames_place_defaults_to_disabled():
    settings = load_settings(CONFIG)
    assert settings.frames.place is None
    assert not settings.place_enabled


@pytest.mark.parametrize(
    "update,match",
    [
        ({"place_position_arm_m": (0.3, 0.0, 0.3)}, "place_position_arm_m"),
        ({"place_orientation_arm_xyzw": TOP_GRASP_ROLL180_XYZW}, "place_orientation_arm_xyzw"),
    ],
)
def test_removed_profile_place_pose_fields_are_rejected(update, match):
    values = load_settings(CONFIG).model_dump()
    values["profile"].update(update)
    with pytest.raises(ValidationError, match=match):
        PickSettings.model_validate(values)


@pytest.mark.parametrize("clearance", [0, -0.01, 0.21])
def test_rejects_invalid_observation_clearance(clearance):
    values = load_settings(CONFIG).model_dump()
    values["profile"]["observation_clearance_m"] = clearance
    with pytest.raises(ValidationError):
        PickSettings.model_validate(values)


@pytest.mark.parametrize("offset", [(0.3, 0.0, 0.0), (0.15, 0.15, 0.0), (0.0, 0.0, -0.21)])
def test_rejects_invalid_observation_offset(offset):
    values = load_settings(CONFIG).model_dump()
    values["profile"]["observation_offset_root_m"] = offset
    with pytest.raises(ValidationError, match="observation_offset_root_m"):
        PickSettings.model_validate(values)


def test_accepts_observation_offset_inside_the_limit():
    values = load_settings(CONFIG).model_dump()
    values["profile"]["observation_offset_root_m"] = (0.1, 0.1, 0.0)
    settings = PickSettings.model_validate(values)
    assert settings.profile.observation_offset_root_m == pytest.approx((0.1, 0.1, 0.0))


@pytest.mark.parametrize(
    "update",
    [
        {"home_tolerance_rad": 0.0},
        {"home_tolerance_rad": 0.31},
        {"home_timeout_s": 0.0},
    ],
)
def test_rejects_invalid_home_limits(update):
    values = load_settings(CONFIG).model_dump()
    values["limits"].update(update)
    with pytest.raises(ValidationError):
        PickSettings.model_validate(values)


def test_ros2_defaults_match_the_frozen_contract():
    spec = Ros2Settings()
    assert spec.prefix == "/arm"
    assert spec.base_frame == "base_link"
    assert spec.tcp_frame == "gripper_frame_link"
    assert spec.joint_states_topic == "/arm/joint_states"
    assert spec.joint_command_topic == "/arm/joint_command"
    assert spec.state_topic == "/arm/state"
    assert spec.plan_request_service == "/arm/plan_request"
    assert spec.switch_state_service == "/arm/switch_state"
    assert spec.acquire_authority_service == "/arm/authority/acquire"
    assert spec.release_authority_service == "/arm/authority/release"
    assert spec.joint_names == DEFAULT_JOINT_NAMES
    assert spec.gripper_joint_index == 5
    assert spec.joint_names[spec.gripper_joint_index] == "gripper"


def test_gripper_width_angle_map_matches_d009():
    gripper = GripperSettings()
    assert gripper.width_per_rad == pytest.approx(DEFAULT_GRIPPER_WIDTH_PER_RAD)
    assert gripper.width_from_angle(DEFAULT_GRIPPER_MAX_ANGLE_RAD) == pytest.approx(0.1)
    assert gripper.width_from_angle(0.0) == pytest.approx(0.0)
    assert gripper.angle_from_width(0.1) == pytest.approx(DEFAULT_GRIPPER_MAX_ANGLE_RAD)
    # Commands outside the map clamp instead of extrapolating.
    assert gripper.width_from_angle(10.0) == pytest.approx(0.1)
    assert gripper.width_from_angle(-1.0) == pytest.approx(0.0)
    assert gripper.angle_from_width(1.0) == pytest.approx(DEFAULT_GRIPPER_MAX_ANGLE_RAD)
    assert gripper.angle_from_width(-0.1) == pytest.approx(0.0)
    assert gripper.angle_from_width(0.08) == pytest.approx(0.08 / DEFAULT_GRIPPER_WIDTH_PER_RAD)


@pytest.mark.parametrize(
    "update",
    [
        {"prefix": "arm"},
        {"prefix": "/arm/"},
        {"prefix": "/arm//left"},
        {"prefix": "/a rm"},
        {"tcp_frame": "base_link"},
        {"tcp_frame": ""},
        {"base_frame": ""},
        {"joint_names": ("a", "b", "c", "d", "e")},
        {"joint_names": ("a", "b", "c", "d", "e", "e")},
        {"home_joint_positions": (0.0, 0.0, 0.0, 0.0, 0.0)},
    ],
)
def test_ros2_settings_reject_unsafe_values(update):
    with pytest.raises(ValidationError):
        Ros2Settings.model_validate({**Ros2Settings().model_dump(), **update})


def test_ros2_settings_accept_overrides():
    spec = Ros2Settings.model_validate(
        {
            "prefix": "/left_arm",
            "base_frame": "world",
            "tcp_frame": "tool0",
            "joint_names": DEFAULT_JOINT_NAMES,
            "home_joint_positions": (0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
        }
    )
    assert spec.joint_states_topic == "/left_arm/joint_states"
    assert spec.joint_command_topic == "/left_arm/joint_command"
    assert spec.plan_request_service == "/left_arm/plan_request"


def test_gripper_settings_reject_inverted_angle_span():
    with pytest.raises(ValidationError, match="minimum angle"):
        GripperSettings.model_validate(
            {"min_angle_rad": 1.0, "max_angle_rad": 0.5, "width_per_rad": 0.05}
        )


def test_pick_rejects_closed_width_above_grasp_interval():
    values = load_settings(CONFIG).model_dump()
    values["profile"]["gripper_closed_m"] = 0.04
    with pytest.raises(ValidationError, match="closure target"):
        PickSettings.model_validate(values)


@pytest.mark.parametrize(
    "update,match",
    [
        # Field bounds (gt=0, le=0.02).
        ({"grasp_contact_margin_m": 0.0}, "grasp_contact_margin_m"),
        ({"grasp_contact_margin_m": 0.021}, "grasp_contact_margin_m"),
        ({"grasp_width_tolerance_m": 0.0}, "grasp_width_tolerance_m"),
        ({"grasp_width_tolerance_m": 0.021}, "grasp_width_tolerance_m"),
        # margin would exceed the calibrated closed -> min clearance.
        ({"grasp_contact_margin_m": 0.02}, "grasp contact margins"),
        # Contact window collapses: closed + tolerance margin >= open - tolerance margin.
        (
            {
                "gripper_closed_m": 0.030,
                "grasp_min_width_m": 0.032,
                "grasp_max_width_m": 0.034,
                "cube_side_m": 0.033,
                "gripper_open_m": 0.035,
                "grasp_contact_margin_m": 0.003,
            },
            "grasp contact margins",
        ),
        # Holding tolerance must stay below the full closed -> open travel.
        (
            {
                "gripper_closed_m": 0.030,
                "grasp_min_width_m": 0.032,
                "grasp_max_width_m": 0.034,
                "cube_side_m": 0.033,
                "gripper_open_m": 0.036,
                "grasp_width_tolerance_m": 0.010,
            },
            "grasp contact margins",
        ),
    ],
)
def test_pick_rejects_invalid_grasp_contact_margins(update, match):
    values = load_settings(CONFIG).model_dump()
    values["profile"].update(update)
    with pytest.raises(ValidationError, match=match):
        PickSettings.model_validate(values)


def test_pick_rejects_workspace_bounds_reversed():
    values = load_settings(CONFIG).model_dump()
    values["profile"]["workspace_min_m"] = (1.0, 0.0, 0.0)
    with pytest.raises(ValidationError, match="workspace bounds"):
        PickSettings.model_validate(values)


def test_pick_rejects_non_unit_reference_orientation():
    values = load_settings(CONFIG).model_dump()
    values["profile"]["reference_tcp_orientation_root_xyzw"] = (0.0, 2.0, 0.0, 0.5)
    with pytest.raises(ValidationError, match="unit xyzw quaternion"):
        PickSettings.model_validate(values)


def test_pick_rejects_unknown_extra_keys():
    values = load_settings(CONFIG).model_dump()
    values["unexpected"] = True
    with pytest.raises(ValidationError):
        PickSettings.model_validate(values)


def test_unknown_task_is_rejected(tmp_path):
    config = tmp_path / "drawer.yaml"
    config.write_text("task: open\n")
    with pytest.raises(ValueError, match="config task must be pick"):
        task_type(config)
    with pytest.raises(ValueError, match="config task must be pick"):
        load_settings(config)


def test_missing_profile_is_rejected(tmp_path):
    config = tmp_path / "broken.yaml"
    config.write_text(
        "task: pick\nframes: {root: a, arm_base: b, object: c}\nprofile: {}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValidationError):
        load_settings(config)


def test_hold_duration_environment_override(monkeypatch):
    monkeypatch.setenv("PICK_HOLD_DURATION_S", "0.75")
    settings = load_settings(CONFIG)
    assert settings.profile.hold_duration_s == pytest.approx(0.75)


def test_hold_duration_override_requires_inline_profile(monkeypatch, tmp_path):
    monkeypatch.setenv("PICK_HOLD_DURATION_S", "1")
    config = tmp_path / "pick.yaml"
    config.write_text("task: pick\nframes: {root: a, arm_base: b, object: c}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="Pick profile must be a YAML mapping"):
        load_settings(config)


def test_profile_files_is_rejected(tmp_path):
    config = tmp_path / "pick.yaml"
    config.write_text(
        "task: pick\nframes: {root: a, arm_base: b, object: c}\nprofile_files: [a.yaml, b.yaml]\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="profile_files"):
        load_settings(config)


def test_frames_require_three_distinct_names():
    values = load_settings(CONFIG).model_dump()
    values["frames"]["arm_base"] = values["frames"]["root"]
    with pytest.raises(ValidationError, match="distinct"):
        PickSettings.model_validate(values)


def test_frames_allow_so101_arm_base():
    values = load_settings(CONFIG).model_dump()
    values["frames"]["arm_base"] = "base_link"
    values["ros2"]["base_frame"] = "base_link"
    settings = PickSettings.model_validate(values)
    assert settings.frames.arm_base == "base_link"
    assert settings.ros2.base_frame == "base_link"


def test_ros2_base_frame_must_match_command_frame():
    values = load_settings(CONFIG).model_dump()
    values["ros2"]["base_frame"] = "base_link"
    with pytest.raises(ValidationError, match="ros2.base_frame must equal frames.arm_base"):
        PickSettings.model_validate(values)


def test_config_limit_is_finite():
    # Keep the copied defaults sane: no NaN/Inf sneaks into the servo timing.
    settings = load_settings(CONFIG)
    assert math.isfinite(settings.limits.servo_max_gap_s)
    assert settings.limits.servo_max_gap_s > 1 / settings.limits.servo_hz


def test_live_ros2_vision_high_observe_config():
    """S6b（task-29）：视觉链路高观察位看全局；抓取参考回 q0；rest 0。"""
    settings = load_settings(VISION)
    assert settings.vision.enabled is True
    assert settings.vision.observe_position_arm_m == pytest.approx(
        HIGH_OBSERVE_POSITION_ARM_M)
    assert settings.vision.observe_orientation_arm_xyzw == pytest.approx(
        HIGH_OBSERVE_ORIENTATION_ARM_XYZW)
    assert settings.ros2.home_joint_positions == pytest.approx(WRIST180_REST_JOINTS)
    assert settings.limits.servo_max_gap_s == pytest.approx(0.45)
    # 真机单码 PnP 角度噪声（spread 0.0204）→ 稳定门 0.03 rad；平移/reproj 门不变
    assert settings.vision.observe_stability_rad == pytest.approx(0.03)
    assert settings.vision.observe_stability_m == pytest.approx(0.008)
    assert settings.vision.observe_max_reprojection_px == pytest.approx(1.5)
    assert settings.profile.reference_tcp_orientation_root_xyzw == pytest.approx(
        TOP_GRASP_ROLL180_XYZW
    )
    # 校准 revision 已 bump（高观察位 + 抓取 q0；place TF 由 Lead 同步旋转）；
    # rest wrist_roll=0（π 超真机限位 [-2.74385, 2.84121]）。
    assert 'obs-high-wristroll-m150' in settings.profile.calibration_revision
    assert settings.ros2.home_joint_positions[4] == pytest.approx(0.0)
    # mock/vision 配置与 tools/ 不改（只验证真机视觉模板本身）。
    mock_vision = ROOT / 'configs/pick_place/mock_ros2_vision.yaml'
    if mock_vision.is_file():
        mock_settings = load_settings(mock_vision)
        assert mock_settings.vision.observe_orientation_arm_xyzw != pytest.approx(
            HIGH_OBSERVE_ORIENTATION_ARM_XYZW
        )
