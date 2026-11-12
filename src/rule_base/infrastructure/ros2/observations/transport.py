"""ROS 2 lifecycle transport: one node, TF observations and feedback channels.

The transport owns the ROS 2 node and spins it on a dedicated thread because the
station `Arm` calls block inside the round worker thread. Both the environment
warmup and the command adapter read the same caches, which keeps feedback
freshness checks identical for every consumer.
"""

from __future__ import annotations

import math
import os
import threading
import time
from threading import Condition, RLock
from typing import Any

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
except ImportError as exc:  # pragma: no cover - depends on the sourced ROS 2 environment
    raise RuntimeError(
        "ROS 2 Python bindings are unavailable; source ros2_ws/setup_ros_env.sh first"
    ) from exc

from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import (
    ArmFeedbackError,
    ContractError,
    Feedback,
    PickScene,
    Pose,
)
from rule_base.infrastructure.transform.observations import PickObservationCache
from rule_base.observability.events import event

# Valid FSM states reported on `<prefix>/state` (D-006); negative values latch a fault.
_STATE_MIN, _STATE_MAX = -3, 5


class Ros2Transport:
    """Lifecycle transport and environment snapshot source for the Pick station."""

    def __init__(self, settings: PickSettings) -> None:
        self.settings = settings
        self.observations = PickObservationCache(settings)
        self._lock = RLock()
        self._updates = Condition(self._lock)
        self._joints: tuple[float, ...] | None = None
        self._joint_velocities: tuple[float, ...] = ()
        self._joint_effort: tuple[float, ...] = ()
        self._joint_at = 0.0
        self._effort_unavailable_reported = False
        self._state: int | None = None
        self._state_at = 0.0
        self._tcp_at = 0.0
        self._tcp_dynamic_seen = False
        self._error: str | None = None
        self._node: Any = None
        self._executor: Any = None
        self._spin_thread: threading.Thread | None = None
        self._stop_spin = threading.Event()
        self._subscriptions: list[Any] = []
        self._refresh_client: Any = None
        self._owns_context = False

    # -- lifecycle ---------------------------------------------------------

    @property
    def node(self) -> Any:
        if self._node is None:
            raise ContractError("ROS 2 transport is not open")
        return self._node

    def open(self) -> None:
        if self._node is not None:
            raise ContractError("ROS 2 transport is already open")
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True
        self._node = rclpy.create_node(f"rule_base_{self.settings.task}_{os.getpid()}")
        try:
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(
                target=self._spin, name="rule-base-ros2", daemon=True
            )
            self._stop_spin.clear()
            self._spin_thread.start()
            self._subscribe()
            event(
                "ros2_transport_open",
                node=self._node.get_name(),
                joint_states=self.settings.ros2.joint_states_topic,
                base_frame=self.settings.ros2.base_frame,
                tcp_frame=self.settings.ros2.tcp_frame,
            )
            self._wait_for_joint_states()
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        errors: list[str] = []
        self._stop_spin.set()
        thread = self._spin_thread
        self._spin_thread = None
        node, self._node = self._node, None
        executor, self._executor = self._executor, None
        if thread is not None:
            thread.join(timeout=2)
            if thread.is_alive():
                errors.append("ROS 2 spin thread did not stop")
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception as exc:  # pragma: no cover - best-effort teardown
                errors.append(str(exc))
        if node is not None:
            try:
                node.destroy_node()
            except Exception as exc:  # pragma: no cover
                errors.append(str(exc))
        self._subscriptions.clear()
        self._refresh_client = None
        if self._owns_context:
            self._owns_context = False
            try:
                rclpy.shutdown()
            except Exception as exc:  # pragma: no cover
                errors.append(str(exc))
        if errors:
            raise ContractError("ROS 2 transport cleanup failed: " + "; ".join(errors))

    def check_health(self) -> None:
        with self._lock:
            self._check_error()
        if self._spin_thread is None or not self._spin_thread.is_alive():
            raise ContractError("ROS 2 spin thread stopped")
        if not rclpy.ok():
            raise ContractError("ROS 2 context is shut down")
        self.observations.check_health()

    # -- environment snapshot ---------------------------------------------

    def robot_ready(self) -> bool:
        return self.observations.robot_ready()

    def snapshot(self) -> PickScene:
        return self.observations.snapshot()

    def request_new_observation(self) -> None:
        self.observations.request_new_observation()

    def sample_age_s(self) -> float | None:
        return self.observations.sample_age_s()

    def refresh_object_pose(self) -> str:
        """Force one vision detection cycle through the contract's Trigger service.

        Runs on the round worker thread: the node is spun by the dedicated thread,
        so only the service call itself is waited on here. The cache is told first
        that a newer object/target TF sample is required, so a pre-refresh sample
        can never be mistaken for the refreshed detection.
        """
        vision = self.settings.vision
        if not vision.enabled:
            raise ContractError("vision refresh requested while vision is disabled")
        self.check_health()
        self.observations.request_new_vision_sample()
        client = self._vision_client()
        timeout = vision.refresh_timeout_s
        if not client.wait_for_service(timeout_sec=timeout):
            raise ContractError(
                f"vision refresh service is unavailable: {vision.refresh_service}"
            )
        started = time.monotonic()
        future = client.call_async(Trigger.Request())
        while not future.done():
            if self._stop_spin.is_set() or self._spin_thread is None:
                raise ContractError("vision refresh interrupted by transport shutdown")
            if time.monotonic() - started > timeout:
                future.cancel()
                raise ContractError(
                    "vision refresh service timed out: "
                    f"service={vision.refresh_service}; timeout_s={timeout:.3f}"
                )
            time.sleep(0.01)
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - transport faults are fatal
            raise ContractError(f"vision refresh service call failed: {exc}") from exc
        if response is None:  # pragma: no cover - defensive
            raise ContractError("vision refresh service returned no response")
        if not response.success:
            raise ContractError(f"vision refresh rejected: {response.message}")
        return str(response.message)

    def _vision_client(self) -> Any:
        with self._lock:
            if self._refresh_client is None:
                self._refresh_client = self.node.create_client(
                    Trigger, self.settings.vision.refresh_service
                )
                event(
                    "ros2_vision_client",
                    service=self.settings.vision.refresh_service,
                )
            return self._refresh_client

    # -- cached feedback ---------------------------------------------------

    def joint_snapshot(self) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        with self._lock:
            self._check_error()
            if self._joints is None:
                raise ArmFeedbackError("missing Arm joint_states feedback")
            age = time.monotonic() - self._joint_at
            if age > self.settings.arm.feedback_timeout_s:
                raise ArmFeedbackError(
                    f"Arm joint_states feedback is stale; age_s={age:.6f}; "
                    f"max_age_s={self.settings.arm.feedback_timeout_s}"
                )
            return self._joints, self._joint_velocities, self._joint_effort

    def state_snapshot(self) -> tuple[int, float]:
        with self._lock:
            self._check_error()
            if self._state is None:
                raise ArmFeedbackError("missing Arm state feedback")
            age = time.monotonic() - self._state_at
            if age > self.settings.arm.state_timeout_s:
                raise ArmFeedbackError(
                    f"Arm state feedback is stale; age_s={age:.6f}; "
                    f"max_age_s={self.settings.arm.state_timeout_s}"
                )
            return self._state, age

    def pose(self, target_frame: str, source_frame: str) -> Pose:
        """Return source_frame in target_frame, gated on fresh dynamic TCP TF."""
        with self._lock:
            self._check_error()
            if (
                self._tcp_dynamic_seen
                and time.monotonic() - self._tcp_at > self.settings.arm.feedback_timeout_s
            ):
                raise ArmFeedbackError(
                    f"Arm pose feedback is stale; frame={self.settings.ros2.base_frame}"
                    f"->{self.settings.ros2.tcp_frame}"
                )
        return self.observations.pose(target_frame, source_frame)

    def feedback(self) -> Feedback:
        """Assemble the domain Feedback from the transport caches."""
        spec = self.settings.ros2
        positions, velocities, effort = self.joint_snapshot()
        state, _ = self.state_snapshot()
        if state < 0:
            raise ContractError(f"Arm reports unhealthy state {state}")
        pose = self.pose(spec.base_frame, spec.tcp_frame)
        gripper = spec.gripper
        index = spec.gripper_joint_index
        width = gripper.width_from_angle(positions[index])
        width_velocity = gripper.width_per_rad * velocities[index] if velocities else 0.0
        gripper_effort = effort[index] if effort else 0.0
        return Feedback(pose, positions, width, width_velocity, gripper_effort, velocities)

    # -- internals ---------------------------------------------------------

    def _spin(self) -> None:
        while not self._stop_spin.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except Exception as exc:  # pragma: no cover - executor faults are fatal
                if not self._stop_spin.is_set():
                    self._fail(f"ROS 2 executor failed: {exc}")
                return

    def _subscribe(self) -> None:
        node = self.node
        spec = self.settings.ros2
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Sensor feedback accepts reliable or best-effort publishers.
        feedback_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._subscriptions = [
            node.create_subscription(
                TFMessage, "/tf", lambda msg: self._on_tf(msg, static=False), tf_qos
            ),
            node.create_subscription(
                TFMessage, "/tf_static", lambda msg: self._on_tf(msg, static=True), static_qos
            ),
            node.create_subscription(
                JointState, spec.joint_states_topic, self._on_joint_states, feedback_qos
            ),
            node.create_subscription(Int32, spec.state_topic, self._on_state, feedback_qos),
        ]

    def _on_tf(self, message: Any, *, static: bool) -> None:
        try:
            self.observations.update("tf_static" if static else "tf", message)
        except Exception as exc:
            self._fail(f"invalid TF input: {exc}")
            return
        tcp = self.settings.ros2.tcp_frame
        contains_tcp = any(
            transform.header.frame_id == tcp or transform.child_frame_id == tcp
            for transform in message.transforms
        )
        if not contains_tcp:
            return
        with self._updates:
            if not static:
                self._tcp_dynamic_seen = True
            self._tcp_at = time.monotonic()
            self._updates.notify_all()

    def _on_joint_states(self, message: Any) -> None:
        try:
            spec = self.settings.ros2
            names = tuple(message.name)
            if names != spec.joint_names:
                raise ContractError(
                    f"expected ordered joints {list(spec.joint_names)}, got {list(names)}"
                )
            positions = tuple(float(v) for v in message.position)
            velocities = tuple(float(v) for v in message.velocity)
            effort = tuple(float(v) for v in message.effort)
            if len(positions) != len(spec.joint_names):
                raise ContractError("incomplete Arm joint_states positions")
            if len(velocities) not in (0, len(spec.joint_names)):
                raise ContractError("incomplete Arm joint_states velocities")
            if len(effort) not in (0, len(spec.joint_names)):
                raise ContractError("incomplete Arm joint_states effort")
            if not all(math.isfinite(v) for v in positions + velocities):
                raise ContractError("nonfinite Arm joint_states feedback")
            if effort and not all(math.isfinite(v) for v in effort):
                # Some hardware (mock ros2_control, or real arms without an
                # effort state interface) publishes NaN effort. Effort is only
                # diagnostic in the Pick station, so treat it as unavailable
                # instead of rejecting otherwise valid position/velocity
                # feedback. Position/velocity stay strict.
                if not self._effort_unavailable_reported:
                    self._effort_unavailable_reported = True
                    event(
                        "arm_feedback_effort_unavailable",
                        reason="non-finite effort in joint_states; treated as unavailable",
                    )
                effort = ()
        except Exception as exc:
            self._fail(f"invalid Arm joint_states: {exc}")
            return
        with self._updates:
            self._joints = positions
            # An absent velocity array stays absent: it must never mean stopped.
            self._joint_velocities = velocities
            self._joint_effort = effort
            self._joint_at = time.monotonic()
            self._updates.notify_all()

    def _on_state(self, message: Any) -> None:
        state = int(message.data)
        if not _STATE_MIN <= state <= _STATE_MAX:
            self._fail(f"unknown Arm state {state}")
            return
        with self._updates:
            self._state = state
            self._state_at = time.monotonic()
            self._updates.notify_all()

    def _wait_for_joint_states(self) -> None:
        deadline = time.monotonic() + self.settings.limits.warmup_timeout_s
        next_report = 0.0
        with self._updates:
            while True:
                self._check_error()
                if self._spin_thread is None or not self._spin_thread.is_alive():
                    raise ContractError("ROS 2 spin thread stopped during warmup")
                now = time.monotonic()
                if (
                    self._joints is not None
                    and now - self._joint_at <= self.settings.arm.feedback_timeout_s
                ):
                    return
                remaining = deadline - now
                if remaining <= 0:
                    raise ContractError(
                        "ROS 2 warmup timed out waiting for "
                        f"{self.settings.ros2.joint_states_topic}"
                    )
                if now >= next_report:
                    event(
                        "ros2_transport_waiting",
                        pending=[self.settings.ros2.joint_states_topic],
                        remaining_s=remaining,
                    )
                    next_report = now + 1.0
                self._updates.wait(min(remaining, 0.05))

    def _fail(self, message: str) -> None:
        with self._updates:
            if self._error is None:
                self._error = message
            self._updates.notify_all()

    def _check_error(self) -> None:
        if self._error is not None:
            raise ContractError(self._error)
