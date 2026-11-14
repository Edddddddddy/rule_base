"""Deterministic synthetic Pick environment and Arm without DDS or hidden hardware."""

from __future__ import annotations

import math
import time
from dataclasses import replace
from typing import Any

from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import (
    ContractError,
    Feedback,
    PickScene,
    Pose,
    Quaternion,
)
from rule_base.infrastructure.transform.math import compose, from_pose, inverse, to_pose
from rule_base.infrastructure.transform.observations import PickObservationCache


def tf_entry(
    parent: str,
    child: str,
    xyz: tuple[float, float, float],
    *,
    timestamp_ns: int,
    orientation_xyzw: Quaternion = (0, 0, 0, 1),
) -> dict[str, Any]:
    return {
        "header": {
            "frame_id": parent,
            "stamp": {"sec": timestamp_ns // 10**9, "nanosec": timestamp_ns % 10**9},
        },
        "child_frame_id": child,
        "transform": {
            "translation": dict(zip("xyz", xyz, strict=True)),
            "rotation": dict(zip("xyzw", orientation_xyzw, strict=True)),
        },
    }


class FakeTransport:
    def __init__(self) -> None:
        self.closed = False

    def open(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True

    def check_health(self) -> None:
        pass


PICK_MOCK_OBJECT_POSE = Pose(
    (0.5207944072076852, -0.026879787753819245, 0.379877403462897),
    (0.07129472561780667, -0.05073860905648599, -0.0027215354184881835, 0.9961602526175499),
)
PICK_MOCK_ROOT_TO_ARM = Pose((-0.10325, 0, 0), (0.0, 2**-0.5, 0.0, 2**-0.5))
# The vision target TCP stands back from the contact point along the arm approach
# axis (profile.grasp_contact_offset_tcp_m = 4 cm; root approach -Z is arm -X here).
PICK_MOCK_TARGET_OFFSET_ARM = (-0.04, 0.0, 0.0)
# Place target in the mock command frame: object + 12 cm along mock +Y, identity
# orientation. The mock world->arm base is Ry(90 deg), so an identity arm-frame
# orientation keeps the TCP approach axis (+X) pointing down in the root frame
# (the top-grasp gate used by planning); do not copy the real profile quaternion.
PICK_MOCK_PLACE_POSE = Pose(
    (0.5207944072076852, 0.09312021224618075, 0.379877403462897),
    (0.0, 0.0, 0.0, 1.0),
)


class FakePickEnvironment:
    def __init__(
        self,
        settings: PickSettings,
        *,
        target_pose: Pose | None = None,
        publish_target: bool = True,
    ) -> None:
        self.settings = settings
        self.cache = PickObservationCache(settings)
        # Vision target TCP in the arm command frame; tests may move it per refresh.
        self.target_pose = target_pose if target_pose is not None else self._default_target()
        self.publish_target = publish_target
        self.publish_on_refresh = True
        self.refresh_responses: list[str] = []
        self.target_updates: list[Pose] = []
        self.refresh_calls = 0
        # Scripted service latency; > vision.refresh_timeout_s behaves like a timeout.
        self.refresh_delay_s = 0.0
        f = settings.frames
        root_arm = tf_entry(f.root, f.arm_base, PICK_MOCK_ROOT_TO_ARM.position, timestamp_ns=0)
        root_arm["transform"]["rotation"] = dict(
            zip("xyzw", PICK_MOCK_ROOT_TO_ARM.orientation, strict=True)
        )
        self.cache.update("tf_static", {"transforms": [root_arm]})
        if settings.env_frames.enabled:
            # Env mode: the place target is a tf_static frame (published once), while
            # the object marker is a dynamic /tf frame re-published below.
            self._publish_env_place_static()
        self.publish_object()

    def _publish_env_place_static(self) -> None:
        """Env mode: publish the tf_static place frame (parent arm_base) once."""
        f = self.settings.frames
        place_frame = self.settings.place_frame
        if place_frame is None:  # defensive: env config validation requires it
            return
        arm_place = tf_entry(f.arm_base, place_frame, PICK_MOCK_PLACE_POSE.position, timestamp_ns=0)
        arm_place["transform"]["rotation"] = dict(
            zip("xyzw", PICK_MOCK_PLACE_POSE.orientation, strict=True)
        )
        self.cache.update("tf_static", {"transforms": [arm_place]})

    def _default_target(self) -> Pose:
        """Top-grasp TCP 4 cm above the object centre (back along the approach axis)."""
        profile = self.settings.profile
        reference = from_pose(Pose((0.0, 0.0, 0.0), profile.reference_tcp_orientation_root_xyzw))
        arm_reference = compose(inverse(from_pose(PICK_MOCK_ROOT_TO_ARM)), reference)
        orientation = to_pose(arm_reference).orientation
        position = (
            PICK_MOCK_OBJECT_POSE.position[0] + PICK_MOCK_TARGET_OFFSET_ARM[0],
            PICK_MOCK_OBJECT_POSE.position[1] + PICK_MOCK_TARGET_OFFSET_ARM[1],
            PICK_MOCK_OBJECT_POSE.position[2] + PICK_MOCK_TARGET_OFFSET_ARM[2],
        )
        return Pose(position, orientation)

    def publish_object(self) -> None:
        f = self.settings.frames
        object_frame = self.settings.object_frame
        place_frame = self.settings.place_frame
        target_frame = self.settings.grasp_target_frame
        stamp = time.time_ns()
        arm_object = tf_entry(
            f.arm_base, object_frame, PICK_MOCK_OBJECT_POSE.position, timestamp_ns=stamp
        )
        arm_object["transform"]["rotation"] = dict(
            zip("xyzw", PICK_MOCK_OBJECT_POSE.orientation, strict=True)
        )
        transforms = [arm_object]
        if self.settings.env_frames.enabled:
            # The env place frame is static (published once in __init__); only the
            # dynamic object marker is refreshed every cycle.
            self.cache.update("tf", {"transforms": transforms})
            return
        if place_frame is not None:
            arm_place = tf_entry(
                f.arm_base, place_frame, PICK_MOCK_PLACE_POSE.position, timestamp_ns=stamp
            )
            arm_place["transform"]["rotation"] = dict(
                zip("xyzw", PICK_MOCK_PLACE_POSE.orientation, strict=True)
            )
            transforms.append(arm_place)
        if target_frame is not None and self.publish_target:
            object_target = to_pose(
                compose(inverse(from_pose(PICK_MOCK_OBJECT_POSE)), from_pose(self.target_pose))
            )
            arm_target = tf_entry(
                object_frame,
                target_frame,
                object_target.position,
                timestamp_ns=stamp,
                orientation_xyzw=object_target.orientation,
            )
            transforms.append(arm_target)
        self.cache.update("tf", {"transforms": transforms})

    def request_new_observation(self) -> None:
        self.cache.request_new_observation()
        self.publish_object()

    def refresh_object_pose(self) -> str:
        """Scripted stand-in for the std_srvs/Trigger vision refresh service."""
        self.cache.request_new_vision_sample()
        self.refresh_calls += 1
        if self.refresh_delay_s:
            time.sleep(self.refresh_delay_s)
            if self.refresh_delay_s > self.settings.vision.refresh_timeout_s:
                raise ContractError(
                    "vision refresh service timed out: "
                    f"service={self.settings.vision.refresh_service}; "
                    f"timeout_s={self.settings.vision.refresh_timeout_s:.3f}"
                )
        response = self.refresh_responses.pop(0) if self.refresh_responses else "ok"
        if response != "ok":
            raise ContractError(f"vision refresh failed: {response}")
        if self.target_updates:
            self.target_pose = self.target_updates.pop(0)
        if self.publish_on_refresh:
            self.publish_object()
        return response

    def sample_age_s(self) -> float | None:
        return self.cache.sample_age_s()

    def robot_ready(self) -> bool:
        return self.cache.robot_ready()

    def snapshot(self) -> PickScene:
        return self.cache.snapshot()

    def check_health(self) -> None:
        self.cache.check_health()


class FakePickArm:
    def __init__(
        self,
        settings: PickSettings,
        *,
        object_width_m: float | None = None,
        object_present: bool = True,
    ) -> None:
        self.settings = settings
        p = settings.profile
        self.object_width_m = (
            object_width_m
            if object_width_m is not None
            else (p.grasp_min_width_m + p.grasp_max_width_m) / 2
        )
        self.object_present = object_present
        self.current = Feedback(
            Pose((0.35, 0, 0.4), PICK_MOCK_OBJECT_POSE.orientation),
            (0.0,) * 6,
            p.gripper_open_m,
            0.0,
            0.0,
            (0.0,) * 6,
        )
        self.owned = False
        self.connected = False
        self.servo_ready = False
        self.calls: list[str] = []
        self.targets: list[Pose] = []
        self.joint_commands: list[tuple[float, ...]] = []

    def connect(self) -> None:
        self.calls.append("connect")
        self.connected = True

    def acquire(self) -> None:
        self.calls.append("acquire")
        self.owned = True

    def feedback(self) -> Feedback:
        if not self.connected:
            raise ContractError("fake Pick Arm not connected")
        return self.current

    def start_servo(self, hold_pose: Pose) -> None:
        if not self.owned:
            raise ContractError("fake Pick Arm has no control")
        self.calls.append("start_servo")
        self.servo_ready = True
        self.move(hold_pose)

    def recover(self, timeout_s: float, stable_time_s: float) -> None:
        self.calls.append("recover")
        if not self.connected:
            self.connect()
        self.stop()
        self.release()
        self.acquire()

    def move(self, target: Pose) -> None:
        if not self.owned or not self.servo_ready:
            raise ContractError("fake Pick Servo not initialized")
        self.calls.append("servo")
        self.targets.append(target)
        self.current = replace(self.current, pose=target)

    def move_joints(self, positions: tuple[float, ...]) -> None:
        if not self.owned:
            raise ContractError("fake Pick joint command has no control")
        expected = len(self.settings.ros2.joint_names)
        if len(positions) != expected or not all(math.isfinite(value) for value in positions):
            raise ContractError(f"expected {expected} finite Arm joint positions")
        self.calls.append("move_joints")
        self.joint_commands.append(tuple(positions))
        self.current = replace(self.current, joint_positions=tuple(positions))

    def gripper(self, width_m: float, effort: float) -> None:
        if not self.owned:
            raise ContractError("fake Pick gripper has no control")
        self.calls.append("gripper")
        contacted = self.object_present and width_m < self.object_width_m
        self.current = replace(
            self.current,
            gripper_width_m=self.object_width_m if contacted else width_m,
            gripper_effort=effort if contacted else 0.0,
        )

    def stop(self) -> None:
        self.calls.append("stop")
        self.servo_ready = False

    def release(self) -> None:
        self.calls.append("release")
        self.owned = False

    def close(self) -> None:
        self.calls.append("close")
        self.connected = False
