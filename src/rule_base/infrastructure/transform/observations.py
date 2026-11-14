"""Fresh Pick scene snapshots from the shared bounded TF buffer."""

from __future__ import annotations

import time
from threading import RLock
from typing import Any

from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import ContractError, PickScene, Pose
from rule_base.infrastructure.transform.buffer import TransformBuffer, field, payload, stamp_ns
from rule_base.infrastructure.transform.math import to_pose


class PickObservationCache:
    """Thread-safe Pick scene and TCP pose views over one TransformBuffer.

    Both the offline fakes and the ROS 2 transport feed the same cache, so the
    freshness gates and frame lookups stay identical in dry-run and live modes.
    """

    def __init__(self, settings: PickSettings) -> None:
        self.settings = settings
        self.tf = TransformBuffer(
            max_age_s=settings.limits.tf_age_s,
            max_skew_s=settings.limits.tf_skew_s,
            future_tolerance_s=settings.limits.tf_future_tolerance_s,
        )
        self._lock = RLock()
        self._error: str | None = None
        self._object_not_before_ns = 0
        self._object_stamp_ns = 0
        self._static_object_seen = False
        self._require_dynamic_object = False
        self._target_not_before_ns = 0
        self._target_stamp_ns = 0
        self._static_target_seen = False
        self._require_dynamic_target = False
        self._sample_reason = "Pick retry"

    def request_new_observation(self) -> None:
        """A retry must see a scene sample measured after the previous attempt.

        The shared TF buffer also carries the arm feedback chain (base -> TCP), so
        it must not be cleared wholesale; freshness is enforced through the object
        (and, when configured, vision target) sample timestamp instead. A statically
        published frame has no newer sample to wait for and is accepted as-is.
        """
        self._request_new_samples("Pick retry")

    def request_new_vision_sample(self) -> None:
        """A vision refresh must be followed by one newer object/target TF sample.

        The same freshness mechanism as retries: the vision node republishes the
        object and target frames from the forced detection cycle (contract §5.2/§6.2),
        so rule_base must not silently accept the pre-refresh sample.
        """
        self._request_new_samples("Pick vision refresh")

    def _request_new_samples(self, reason: str) -> None:
        now = time.time_ns()
        with self._lock:
            self._sample_reason = reason
            self._object_not_before_ns = now
            self._object_stamp_ns = 0
            self._require_dynamic_object = not self._static_object_seen
            if self.settings.grasp_target_frame is not None:
                self._target_not_before_ns = now
                self._target_stamp_ns = 0
                self._require_dynamic_target = not self._static_target_seen

    def fail(self, message: str) -> None:
        with self._lock:
            self._error = message

    def check_health(self) -> None:
        with self._lock:
            if self._error:
                raise ContractError(self._error)

    def update(self, key: str, message: Any) -> None:
        if key not in {"tf", "tf_static"}:
            raise ContractError(f"unknown observation channel {key}")
        with self._lock:
            self.tf.update(payload(message), static=key == "tf_static")
            object_frame = self.settings.object_frame
            target_frame = self.settings.grasp_target_frame
            for item in field(payload(message), "transforms"):
                child = field(item, "child_frame_id")
                if object_frame is not None and child == object_frame:
                    if key == "tf":
                        self._object_stamp_ns = max(
                            self._object_stamp_ns, stamp_ns(field(item, "header"))
                        )
                    else:
                        self._static_object_seen = True
                elif target_frame is not None and child == target_frame:
                    if key == "tf":
                        self._target_stamp_ns = max(
                            self._target_stamp_ns, stamp_ns(field(item, "header"))
                        )
                    else:
                        self._static_target_seen = True

    def robot_ready(self) -> bool:
        """Warm-up gate for the deferred-snapshot modes (vision observe_first / env).

        这两种模式都是「先到观察位再取目标」，折叠 rest 时目标 TF 可能还不存在
        （视觉节点未检测 / env 相机看不到物体），因此 warmup 只查机器人根→臂 TF，
        不要求 object/place/grasp_target TF。返回 True；不可达/过期抛 ContractError。
        """
        with self._lock:
            self.check_health()
            frames = self.settings.frames
            self.tf.lookup_many([(frames.root, frames.arm_base)])
            return True

    def snapshot(self) -> PickScene:
        with self._lock:
            self.check_health()
            frames = self.settings.frames
            object_frame = self.settings.object_frame
            place_frame = self.settings.place_frame
            target_frame = self.settings.grasp_target_frame
            if object_frame is None:  # defensive: config validation requires one source
                raise ContractError("Pick object TF frame is not configured")
            if self._require_dynamic_object and self._object_stamp_ns < self._object_not_before_ns:
                raise ContractError(
                    f"{self._sample_reason} requires new object TF after return to initial pose"
                )
            if (
                target_frame is not None
                and self._require_dynamic_target
                and self._target_stamp_ns < self._target_not_before_ns
            ):
                raise ContractError(
                    f"{self._sample_reason} requires new grasp_target TF sample: "
                    f"{frames.arm_base} <- {target_frame}"
                )
            pairs = [(frames.root, frames.arm_base), (frames.arm_base, object_frame)]
            optional: list[str] = []
            if place_frame is not None:
                pairs.append((frames.arm_base, place_frame))
                optional.append("place")
            if target_frame is not None:
                pairs.append((frames.arm_base, target_frame))
                optional.append("target")
            dynamic_pair = (frames.arm_base, object_frame) if self._require_dynamic_object else None
            try:
                matrices = self.tf.lookup_many(pairs, dynamic_pair=dynamic_pair)
            except ContractError as exc:
                message = str(exc)
                missing = [
                    name
                    for name in (object_frame, place_frame, target_frame)
                    if name is not None and name in message
                ]
                if "TF path unavailable" in message and missing:
                    frame_name = missing[0]
                    kind = (
                        "grasp_target"
                        if target_frame is not None and frame_name == target_frame
                        else ("place" if frame_name == place_frame else "object")
                    )
                    raise ContractError(
                        f"Pick {kind} TF is missing: {frames.arm_base} <- {frame_name}"
                    ) from exc
                if dynamic_pair is None or "no dynamic sample" not in message:
                    raise
                # A statically published frame has no newer sample to wait for.
                self._require_dynamic_object = False
                self._require_dynamic_target = False
                matrices = self.tf.lookup_many(pairs)
            root_arm, arm_object, *rest = matrices
            found = dict(zip(optional, rest, strict=True))
            return PickScene(root_arm, arm_object, found.get("place"), found.get("target"))

    def sample_age_s(self) -> float | None:
        """Age of the freshest dynamic object/target sample used by the last scene."""
        with self._lock:
            stamps = [self._object_stamp_ns]
            if self.settings.grasp_target_frame is not None:
                stamps.append(self._target_stamp_ns)
            freshest = max(stamps)
            if not freshest:
                return None
            return max(0.0, (time.time_ns() - freshest) / 10**9)

    def pose(self, target_frame: str, source_frame: str) -> Pose:
        """Return the source-frame pose expressed in the target frame."""
        with self._lock:
            self.check_health()
            return to_pose(self.tf.lookup_many([(target_frame, source_frame)])[0])
