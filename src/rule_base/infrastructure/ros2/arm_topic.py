"""ROS 2 implementation of the domain Arm/PickArm ports for the SO-101 station.

Feedback subscriptions and TF buffering live in `Ros2Transport` (the single
owner of the node) so the environment warmup and the command adapter cannot
diverge. This module turns that feedback into the domain `Feedback` contract and
maps the port calls onto `arm_msgs` services plus `<prefix>/joint_command`.
"""

from __future__ import annotations

import math
import time
from threading import Event

try:
    from arm_msgs.srv import (
        AcquireAuthority,
        PlanRequest,
        ReleaseAuthority,
        SwitchState,
    )
    from sensor_msgs.msg import JointState
except ImportError as exc:  # pragma: no cover - depends on the sourced ROS 2 environment
    raise RuntimeError(
        "arm_msgs/ROS 2 bindings are unavailable; build and source ros2_ws first"
    ) from exc

from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import (
    ArmFeedbackError,
    ArmSessionError,
    ContractError,
    Feedback,
    InactiveLeaseError,
    Interrupted,
    Pose,
)
from rule_base.infrastructure.ros2.observations.transport import Ros2Transport
from rule_base.infrastructure.transform.math import from_pose
from rule_base.observability.events import event

# Frozen P7 FSM identifiers (D-006): Servo mode 3 and IDLE 0.
SERVO_STATE_ID = 3
IDLE_STATE_ID = 0
JOINT_COMMAND_QOS_DEPTH = 10


class Ros2ArmTopic:
    """Arm/PickArm port backed by arm_msgs services and ROS 2 topics."""

    def __init__(
        self,
        settings: PickSettings,
        transport: Ros2Transport,
        stop: Event | None = None,
    ) -> None:
        self.settings, self.transport = settings, transport
        self.stop_requested = stop
        self._clients: dict[str, object] = {}
        self._publisher: object | None = None
        self._lease_id: int | None = None
        self._owned = False
        self._uncertain = False
        self._servo_requested = False
        self._servo_confirmed = False
        self._servo_acknowledged_at: float | None = None
        self._idle_acknowledged_at: float | None = None
        self._connected = False

    # -- connection and lifecycle ------------------------------------------

    def connect(self) -> None:
        if self._connected:
            return
        node = self.transport.node
        spec = self.settings.ros2
        arm = self.settings.arm
        clients: list[tuple[str, object, str]] = [
            ("plan", node.create_client(PlanRequest, spec.plan_request_service), "plan_request")
        ]
        if arm.acquire_control:
            clients.append(
                (
                    "switch",
                    node.create_client(SwitchState, spec.switch_state_service),
                    "switch_state",
                )
            )
            if arm.manage_lease:
                clients.extend(
                    [
                        (
                            "acquire",
                            node.create_client(AcquireAuthority, spec.acquire_authority_service),
                            "authority/acquire",
                        ),
                        (
                            "release",
                            node.create_client(ReleaseAuthority, spec.release_authority_service),
                            "authority/release",
                        ),
                    ]
                )
        for key, client, label in clients:
            self._clients[key] = client
            event("arm_topic_connect", operation="client", endpoint=spec.prefix + "/" + label)
        self._publisher = node.create_publisher(
            JointState, spec.joint_command_topic, JOINT_COMMAND_QOS_DEPTH
        )
        self._connected = True
        try:
            self._wait_ready()
            self._wait_feedback()
        except BaseException:
            self.close()
            raise
        event("arm_topic_connected", node=spec.prefix)

    def _wait_ready(self) -> None:
        deadline = time.monotonic() + self.settings.limits.warmup_timeout_s
        next_report = 0.0
        while True:
            self._check_stop()
            self.transport.check_health()
            now = time.monotonic()
            pending = []
            for key, client in self._clients.items():
                if not client.service_is_ready():  # type: ignore[attr-defined]
                    pending.append(f"{key}: no server")
            publisher = self._publisher
            if publisher is None or publisher.get_subscription_count() == 0:  # type: ignore[attr-defined]
                pending.append("joint_command: no subscriber")
            if not pending:
                return
            remaining = deadline - now
            if remaining <= 0:
                raise ContractError("Arm ROS 2 discovery warmup timed out: " + "; ".join(pending))
            if now >= next_report:
                event("arm_topic_waiting", pending=pending, remaining_s=remaining)
                next_report = now + 1.0
            time.sleep(min(remaining, 0.05))

    def _wait_feedback(self) -> None:
        deadline = time.monotonic() + self.settings.limits.warmup_timeout_s
        while True:
            self._check_stop()
            try:
                self.feedback()
                return
            except ContractError as exc:
                if time.monotonic() >= deadline:
                    raise ContractError(f"Arm ROS 2 feedback warmup timed out: {exc}") from None
            time.sleep(0.05)

    def close(self) -> None:
        errors: list[str] = []
        node = self.transport.node if self._connected else None
        if node is not None:
            for client in self._clients.values():
                try:
                    node.destroy_client(client)
                except Exception as exc:  # pragma: no cover - best-effort teardown
                    errors.append(str(exc))
            if self._publisher is not None:
                try:
                    node.destroy_publisher(self._publisher)
                except Exception as exc:  # pragma: no cover
                    errors.append(str(exc))
        self._clients.clear()
        self._publisher = None
        self._owned = False
        self._connected = False
        if errors:
            raise ContractError("Arm ROS 2 cleanup failed: " + "; ".join(errors))

    # -- authority and session ---------------------------------------------

    def acquire(self) -> None:
        self._check_connected()
        arm = self.settings.arm
        if not arm.acquire_control:
            return
        spec = self.settings.ros2
        if arm.manage_lease:
            request = AcquireAuthority.Request()
            request.target_topic = spec.joint_command_topic
            self._uncertain = True
            response = self._call("acquire", request)
            if response is None:
                raise ContractError("Arm authority acquisition timed out")
            if not response.success:
                self._uncertain = False
                raise ContractError(f"Arm authority denied: {response.message}")
            lease_id = int(response.lease_id)
            if lease_id <= 0:
                # The server may already hold authority for us; only an explicit
                # successful release can clear that uncertainty.
                raise ContractError("Arm acquisition returned an invalid lease ID")
            self._lease_id = lease_id
            self._uncertain = False
        self._switch(SERVO_STATE_ID)
        self._servo_acknowledged_at = time.monotonic()
        self._servo_requested = True
        self._servo_confirmed = False
        self._owned = True
        # The state topic may still deliver a pre-switch IDLE sample; only a
        # sample received after the acknowledged switch may confirm Servo mode.
        self._wait_servo_state()

    def _wait_servo_state(self) -> None:
        deadline = time.monotonic() + self.settings.arm.state_timeout_s
        acknowledged = self._servo_acknowledged_at or 0.0
        while True:
            self._check_stop()
            now = time.monotonic()
            try:
                state, age = self.transport.state_snapshot()
            except ArmFeedbackError:
                state, age = -1, 0.0
            sample_at = now - age
            if state < 0 and sample_at > acknowledged:
                raise ContractError(f"Arm reports unhealthy state {state}")
            if sample_at > acknowledged and state == SERVO_STATE_ID:
                self._servo_confirmed = True
                return
            if now >= deadline:
                raise ContractError("Arm Servo state confirmation timed out")
            time.sleep(0.01)

    def _switch(self, state_id: int) -> None:
        request = SwitchState.Request()
        request.state_id = state_id
        response = self._call("switch", request)
        if response is None:
            raise ContractError(f"Arm switch_state({state_id}) timed out")
        if not response.success:
            raise ContractError(f"Arm switch_state({state_id}) denied: {response.message}")

    def start_servo(self, hold_pose: Pose) -> None:
        self._check_connected()
        self._require_authority()
        from_pose(hold_pose)
        if not self.settings.arm.acquire_control:
            return
        if not self._servo_requested:
            self._switch(SERVO_STATE_ID)
            self._servo_acknowledged_at = time.monotonic()
            self._servo_requested = True
            self._servo_confirmed = False
            self._owned = True
            self._wait_servo_state()
        event("arm_topic_servo", node=self.settings.ros2.prefix)

    def stop(self) -> None:
        self._idle_acknowledged_at = None
        self._owned = False
        self._servo_confirmed = False
        self._servo_acknowledged_at = None
        if self._uncertain:
            raise ContractError("Arm acquisition outcome unknown; cleanup unconfirmed")
        if not self.settings.arm.acquire_control:
            self._servo_requested = False
            return
        if not self._servo_requested:
            return
        request = SwitchState.Request()
        request.state_id = IDLE_STATE_ID
        response = self._call("switch", request, cleanup=True)
        if response is None:
            raise ContractError("Arm IDLE switch timed out")
        if not response.success:
            raise ContractError(f"Arm IDLE switch denied: {response.message}")
        self._idle_acknowledged_at = time.monotonic()

    def confirm_idle(self) -> None:
        if not self._servo_requested:
            return
        if self._idle_acknowledged_at is None:
            raise ContractError("Arm IDLE request has no successful acknowledgement")
        timeout_s = self.settings.arm.state_timeout_s
        deadline = time.monotonic() + timeout_s
        while True:
            self._check_stop()
            now = time.monotonic()
            try:
                state, age = self.transport.state_snapshot()
            except ArmFeedbackError:
                state, age = -1, 0.0
            if (
                state == IDLE_STATE_ID
                and now - age >= (self._idle_acknowledged_at or 0.0)
                and age <= timeout_s
            ):
                return
            if now >= deadline:
                raise ContractError("Arm IDLE state confirmation timed out")
            time.sleep(0.01)

    def release(self) -> None:
        arm = self.settings.arm
        if not arm.manage_lease:
            return
        if self._uncertain:
            raise ContractError("Arm acquisition outcome unknown; no lease to release")
        if not arm.acquire_control or self._lease_id is None:
            return
        request = ReleaseAuthority.Request()
        request.lease_id = self._lease_id
        response = self._call("release", request, cleanup=True)
        if response is None:
            raise ContractError("Arm authority release timed out")
        if not response.success:
            lease_was_active = self._lease_id is not None
            self._lease_id = None
            self._owned = False
            if lease_was_active and _is_inactive_lease(response.message):
                raise InactiveLeaseError(f"Arm authority release denied: {response.message}")
            raise ContractError(f"Arm authority release denied: {response.message}")
        self._lease_id = None
        self._owned = False

    # -- feedback ----------------------------------------------------------

    def feedback(self) -> Feedback:
        self._check_connected()
        return self.transport.feedback()

    # -- motion commands ---------------------------------------------------

    def move(self, target: Pose) -> None:
        self._check_connected()
        self._require_authority()
        self._plan(target)

    def _plan(self, target: Pose) -> None:
        from_pose(target)
        spec = self.settings.ros2
        request = PlanRequest.Request()
        request.target_pose.header.frame_id = spec.base_frame
        request.target_pose.header.stamp = self.transport.node.get_clock().now().to_msg()
        position = request.target_pose.pose.position
        position.x, position.y, position.z = target.position
        orientation = request.target_pose.pose.orientation
        (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ) = target.orientation
        started = time.monotonic()
        response = self._call("plan", request)
        elapsed = time.monotonic() - started
        if response is None:
            raise ArmSessionError("Arm plan_request timed out")
        if not response.success:
            raise ContractError(f"Arm plan_request denied: {response.message}")
        event(
            "arm_topic_plan",
            node=spec.prefix,
            position_m=target.position,
            elapsed_s=elapsed,
        )

    def move_joints(self, positions: tuple[float, ...]) -> None:
        spec = self.settings.ros2
        if len(positions) != len(spec.joint_names):
            raise ContractError(f"expected {len(spec.joint_names)} finite Arm joint positions")
        if not all(math.isfinite(value) for value in positions):
            raise ContractError(f"expected {len(spec.joint_names)} finite Arm joint positions")
        self._check_connected()
        self._require_authority()
        self._publish_joint_command(positions)

    def gripper(self, width_m: float, effort: float) -> None:
        if not math.isfinite(width_m) or width_m < 0:
            raise ContractError("invalid gripper width")
        self._check_connected()
        self._require_authority()
        spec = self.settings.ros2
        positions, _, _ = self.transport.joint_snapshot()
        command = list(positions)
        command[spec.gripper_joint_index] = spec.gripper.angle_from_width(width_m)
        self._publish_joint_command(tuple(command))
        event(
            "arm_topic_gripper",
            node=spec.prefix,
            width_m=width_m,
            angle_rad=command[spec.gripper_joint_index],
            effort=effort,
        )

    def _publish_joint_command(self, positions: tuple[float, ...]) -> None:
        spec = self.settings.ros2
        publisher = self._publisher
        if publisher is None:
            raise ContractError("Arm joint command publisher is not open")
        if publisher.get_subscription_count() == 0:  # type: ignore[attr-defined]
            raise ContractError("Arm joint_command has no subscriber")
        message = JointState()
        message.header.stamp = self.transport.node.get_clock().now().to_msg()
        message.name = list(spec.joint_names)
        message.position = list(positions)
        publisher.publish(message)  # type: ignore[attr-defined]

    # -- recovery ----------------------------------------------------------

    def recover(self, timeout_s: float, stable_time_s: float) -> None:
        self._check_error()
        if self._uncertain:
            raise ContractError("Arm authority outcome unknown; automatic reacquisition blocked")
        if not self._connected:
            self.close()
            self.connect()
        self.stop()
        self.release()
        # The old session is closed. A later acquire denial must not make final
        # cleanup switch another controller's newly acquired session to IDLE.
        self._servo_requested = False
        self._servo_confirmed = False
        self._servo_acknowledged_at = None
        self._wait_recovery_feedback(timeout_s, stable_time_s, require_servo=False)
        self.acquire()
        if not self.settings.arm.acquire_control:
            self._wait_recovery_feedback(timeout_s, 0.0, require_servo=True)
        if self.settings.ros2.recover_to_home:
            self.move_joints(self.settings.ros2.home_joint_positions)
        event("arm_topic_session_recovered", node=self.settings.ros2.prefix)

    def _wait_recovery_feedback(
        self, timeout_s: float, stable_time_s: float, *, require_servo: bool
    ) -> None:
        deadline = time.monotonic() + timeout_s
        stable_since: float | None = None
        while True:
            self._check_recovery()
            now = time.monotonic()
            try:
                self.feedback()
                state, _ = self.transport.state_snapshot()
            except ArmFeedbackError:
                stable_since = None
            else:
                if require_servo and state != SERVO_STATE_ID:
                    stable_since = None
                elif state < 0:
                    raise ContractError(f"Arm recovery blocked by unhealthy state {state}")
                elif stable_since is None:
                    stable_since = now
                if stable_since is not None and now - stable_since >= stable_time_s:
                    return
            if now >= deadline:
                raise ArmFeedbackError(
                    f"Arm recovery feedback timed out; required_stable_s={stable_time_s}"
                )
            time.sleep(min(0.01, max(0.0, deadline - now)))

    # -- guards ------------------------------------------------------------

    def _check_connected(self) -> None:
        if not self._connected:
            raise ContractError("Arm ROS 2 adapter is not connected")
        self._check_error()

    def _check_error(self) -> None:
        self.transport.check_health()

    def _require_authority(self) -> None:
        state, age = self.transport.state_snapshot()
        if state < 0:
            raise ContractError(f"Arm reports unhealthy state {state}")
        if self.settings.arm.acquire_control and not self._owned:
            raise ArmSessionError("motion requires this round's ROS 2 authority")
        acknowledged = self._servo_acknowledged_at or 0.0
        if (
            self._owned
            and self._servo_confirmed
            and state == IDLE_STATE_ID
            and time.monotonic() - age > acknowledged
        ):
            # Position and Servo modes are both valid under one lease; only a
            # post-switch IDLE sample means the server dropped the session.
            raise ArmSessionError(
                f"Pick Servo session lost: actual state={state}, expected={SERVO_STATE_ID}"
            )

    def _check_stop(self) -> None:
        if self.stop_requested is not None and self.stop_requested.is_set():
            raise Interrupted("stop requested during Arm ROS 2 operation")

    def _check_recovery(self) -> None:
        self._check_stop()

    def _call(self, key: str, request: object, *, cleanup: bool = False) -> object | None:
        client = self._clients.get(key)
        if client is None:
            raise ContractError(f"Arm ROS 2 client is not open: {key}")
        timeout_ms = (
            self.settings.arm.cleanup_rpc_timeout_ms
            if cleanup and self.settings.arm.cleanup_rpc_timeout_ms is not None
            else self.settings.arm.rpc_timeout_ms
        )
        try:
            return client.call(request, timeout_sec=timeout_ms / 1000)  # type: ignore[attr-defined]
        except Exception as exc:
            raise ArmSessionError(f"Arm {key} service call failed: {exc}") from exc


def _is_inactive_lease(message: str) -> bool:
    lowered = message.lower()
    return "lease" in lowered and (
        "does not match" in lowered or "no active" in lowered or "not active" in lowered
    )
