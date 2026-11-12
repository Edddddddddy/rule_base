"""Purpose: guard the vision configuration contract and its validation guards.

Split out of test_pick_vision_servo.py (S6i) to keep every test module < 800 lines.
Usage: source ros2_ws/setup_ros_env.sh && python3 -m pytest tests/unit/test_pick_vision_config.py
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml
from pick_place_support import offline_settings
from pydantic import ValidationError

from rule_base.config.load import load_settings
from rule_base.config.pick_place import PickSettings

ROOT = Path(__file__).resolve().parents[2]
VISION_CONFIG = ROOT / "configs/pick_place/live_ros2_vision.yaml"
REAL_PROFILE = ROOT / "configs/pick_place/profiles/real_pick.yaml"
TARGET_FRAME = "grasp_target_01"
PLACE_FRAME = "place_target_01"


# -- configuration -------------------------------------------------------


def test_vision_defaults_match_the_frozen_contract():
    settings = offline_settings()
    vision = settings.vision
    assert settings.frames.grasp_target is None
    assert vision.enabled is False
    assert vision.refresh_service == "/vision/refresh_object_pose"
    assert vision.refresh_timeout_s == pytest.approx(3.0)
    assert vision.max_corrections == 2
    assert vision.position_tolerance_m == pytest.approx(0.006)
    assert vision.orientation_tolerance_rad == pytest.approx(0.05)
    assert vision.correction_settle_s == pytest.approx(0.4)
    assert vision.observe_joint_positions is None
    assert vision.observe_joint_tolerance_rad == pytest.approx(0.02)
    assert vision.observe_settle_s == pytest.approx(0.8)
    assert vision.observe_sample_interval_s == pytest.approx(0.2)
    assert vision.observe_warmup_samples == 2
    # S6j：抓取目标固定偏移默认零（仅 real_pick.yaml 模板启用 +Y 固定补偿）。
    assert settings.profile.grasp_target_offset_arm_m == pytest.approx((0.0, 0.0, 0.0))
    assert not settings.vision_enabled


def test_vision_config_template_enables_the_target_frame():
    settings = load_settings(VISION_CONFIG)
    assert isinstance(settings, PickSettings)
    assert settings.frames.grasp_target == TARGET_FRAME
    assert settings.frames.place == PLACE_FRAME
    assert settings.vision_enabled
    assert settings.vision.refresh_service == "/vision/refresh_object_pose"
    assert settings.vision.max_corrections == 2
    # S6j（现场用户决定）：只在观察位锁定一次抓取位置，关闭 pregrasp refresh/校正；
    # 横向偏差由 profile.grasp_target_offset_arm_m 固定补偿。
    assert settings.vision.lock_target_after_observation is True
    # S6k：pregrasp 近距离视觉不可信（现场重检差 101.5 mm），只保留观察位一次锁定；
    # 抓取位仅用夹爪闭合宽度验证（字段/代码保留，默认值不变）。
    assert settings.vision.grasp_verify_enabled is False
    # S6j：横向固定补偿以用户现场 WIP 的 real_pick.yaml 为准（当前 [0, +0.020, 0] =
    # base_link +Y 2 cm，机器人左侧）；直接读 profile 文件比较，避免测试值再次漂移。
    real_profile = yaml.safe_load(REAL_PROFILE.read_text(encoding="utf-8"))
    assert settings.profile.grasp_target_offset_arm_m == pytest.approx(
        tuple(real_profile["grasp_target_offset_arm_m"])
    )
    assert settings.profile.grasp_target_offset_arm_m == pytest.approx((0.0, 0.020, 0.0))
    # S6g/S6l：观察位先停稳（1.0 s）、5 次间隔（0.2 s）采样确认 TF 稳定后再锁定；
    # 观察关节到位容差 0.02 rad（比 home 0.08 更严）。
    assert settings.vision.observe_samples == 5
    assert settings.vision.observe_joint_tolerance_rad == pytest.approx(0.02)
    assert settings.vision.observe_settle_s == pytest.approx(1.0)
    assert settings.vision.observe_sample_interval_s == pytest.approx(0.2)
    # S6l：先丢 4 帧旧 coast/平滑目标（run 131649 表明旧平滑 >0.4 s），不计入稳定性门。
    assert settings.vision.observe_warmup_samples == 4
    # task-33：用户手动观察关节（5 臂关节；rest→关节空间→TF 快照→抓取）
    # S6h：wrist_flex 用实测可达上限 ~1.6536 rad（原 1.738 不可达 → 伺服饱和、
    # 误差恒 0.0844 rad → 0.02 rad 观察容差永远达不到、vision_observe_joints 超时）。
    assert settings.vision.observe_joint_positions == pytest.approx(
        (-0.046, 0.0491, -0.3175, 1.6536, -1.4281))
    assert settings.vision.observe_joint_positions[3] <= 1.6536


def test_vision_enabled_requires_a_grasp_target_frame():
    values = offline_settings().model_dump()
    values["vision"]["enabled"] = True
    with pytest.raises(ValidationError, match="grasp_target"):
        PickSettings.model_validate(values)


@pytest.mark.parametrize("duplicate", ["root", "arm_base", "object", "place"])
def test_grasp_target_must_be_a_distinct_frame(duplicate):
    values = offline_settings().model_dump()
    values["frames"]["place"] = PLACE_FRAME
    values["frames"]["grasp_target"] = values["frames"][duplicate]
    with pytest.raises(ValueError):
        PickSettings.model_validate(values)


@pytest.mark.parametrize("grasp_target", ["", "grasp target 01"])
def test_grasp_target_must_be_nonblank(grasp_target):
    values = offline_settings().model_dump()
    values["frames"]["place"] = PLACE_FRAME
    values["frames"]["grasp_target"] = grasp_target
    with pytest.raises(ValueError):
        PickSettings.model_validate(values)


def _vision_values(**updates: object) -> dict[str, object]:
    values = offline_settings().model_dump()
    values["frames"]["grasp_target"] = TARGET_FRAME
    values["vision"].update({"enabled": True, "observe_first": True})
    values["vision"].update(updates)
    return values


def test_observe_joint_positions_are_optional_and_validated():
    settings = PickSettings.model_validate(_vision_values())
    assert settings.vision.observe_joint_positions is None
    settings = PickSettings.model_validate(
        _vision_values(observe_joint_positions=(-0.046, 0.0491, -0.3175, 1.738, -1.4281)))
    assert settings.vision.observe_joint_positions is not None


@pytest.mark.parametrize("joints", [
    (0.0, 0.0, 0.0, 0.0),                 # wrong length (4)
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),       # wrong length (6)
    (0.0, 0.0, 0.0, 0.0, 3.2),            # wrist_roll outside [-2.74385, 2.84121]
    (0.0, 2.5, 0.0, 0.0, 0.0),            # shoulder_lift outside upper limit
    (0.0, float("nan"), 0.0, 0.0, 0.0),   # non-finite
])
def test_rejects_invalid_observe_joint_positions(joints):
    with pytest.raises(ValidationError):
        PickSettings.model_validate(_vision_values(observe_joint_positions=joints))


def test_observe_joint_positions_require_observe_first_enabled():
    values = _vision_values(observe_joint_positions=(0.0, 0.0, 0.0, 0.0, 0.0))
    values["vision"]["observe_first"] = False
    with pytest.raises(ValidationError, match="observe_joint_positions"):
        PickSettings.model_validate(values)
    values = _vision_values(observe_joint_positions=(0.0, 0.0, 0.0, 0.0, 0.0))
    values["vision"]["enabled"] = False
    values["vision"]["observe_first"] = False
    with pytest.raises(ValidationError, match="observe_joint_positions"):
        PickSettings.model_validate(values)


@pytest.mark.parametrize(
    "update",
    [
        {"max_corrections": 0},
        {"max_corrections": 4},
        {"max_corrections": True},
        {"refresh_timeout_s": 0},
        {"position_tolerance_m": 0},
        {"orientation_tolerance_rad": 0},
        {"correction_settle_s": -0.1},
        {"observe_joint_tolerance_rad": 0},
        {"observe_joint_tolerance_rad": 0.081},
        {"observe_settle_s": -0.1},
        {"observe_settle_s": 5.1},
        {"observe_sample_interval_s": -0.1},
        {"observe_sample_interval_s": 1.01},
        {"observe_warmup_samples": -1},
        {"observe_warmup_samples": 6},
        {"observe_warmup_samples": True},
        {"refresh_service": "vision/refresh_object_pose"},
        {"refresh_service": "/vision/refresh_object_pose/"},
        {"refresh_service": "/vision//refresh"},
        {"refresh_service": "/vision/re fresh"},
    ],
)
def test_rejects_invalid_vision_settings(update):
    values = offline_settings().model_dump()
    values["frames"]["grasp_target"] = TARGET_FRAME
    values["vision"].update(update)
    with pytest.raises(ValidationError):
        PickSettings.model_validate(values)


def test_observe_settle_and_joint_tolerance_boundaries_are_accepted():
    values = offline_settings().model_dump()
    values["frames"]["grasp_target"] = TARGET_FRAME
    values["vision"].update({
        "observe_joint_tolerance_rad": 0.08,
        "observe_settle_s": 5.0,
        "observe_sample_interval_s": 1.0,
    })
    settings = PickSettings.model_validate(values)
    assert settings.vision.observe_joint_tolerance_rad == pytest.approx(0.08)
    assert settings.vision.observe_settle_s == pytest.approx(5.0)
    assert settings.vision.observe_sample_interval_s == pytest.approx(1.0)
    values["vision"].update({
        "observe_joint_tolerance_rad": 0.001,
        "observe_settle_s": 0.0,
        "observe_sample_interval_s": 0.0,
    })
    settings = PickSettings.model_validate(values)
    assert settings.vision.observe_joint_tolerance_rad == pytest.approx(0.001)
    assert settings.vision.observe_settle_s == 0.0
    assert settings.vision.observe_sample_interval_s == 0.0


@pytest.mark.parametrize("offset", [
    (0.06, 0.0, 0.0),
    (0.0, -0.051, 0.0),
    (0.0, 0.0, float("nan")),
])
def test_rejects_invalid_grasp_target_offset(offset):
    values = offline_settings().model_dump()
    values["profile"]["grasp_target_offset_arm_m"] = offset
    with pytest.raises(ValidationError, match="grasp_target_offset_arm_m"):
        PickSettings.model_validate(values)


def test_grasp_target_offset_bounds_are_accepted():
    values = offline_settings().model_dump()
    values["profile"]["grasp_target_offset_arm_m"] = (0.05, -0.05, 0.0)
    settings = PickSettings.model_validate(values)
    assert settings.profile.grasp_target_offset_arm_m == pytest.approx((0.05, -0.05, 0.0))


