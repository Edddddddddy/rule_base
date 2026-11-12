#!/usr/bin/env python3
"""Minimal SO-101 arm server double for the rule_base ROS 2 loopback test.

Implements the frozen arm-server contract (D-004..D-009):
  * services  <prefix>/plan_request, <prefix>/switch_state,
              <prefix>/authority/{acquire,release}
  * topics    <prefix>/joint_states (6 joints), <prefix>/state (Int32),
              <prefix>/joint_command (JointState commands, D-005)
  * TF        static world->base_link, world->objects_cube_01 and
              world->place_target_01 (D-093 place pose from TF),
              dynamic base_link->gripper_frame_link (perfect tracking)
  * gripper   D-009 linear m<->rad map plus simple contact simulation

This double exists only for tests/integration/test_ros2_loopback.py. WP4 runs
against the real `so101_arm_app` package instead.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time

import rclpy
from arm_msgs.srv import (
    AcquireAuthority,
    PlanRequest,
    ReleaseAuthority,
    SwitchState,
)
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from tf2_msgs.msg import TFMessage

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
IDLE, SERVO, POSITION = 0, 3, 4
ALLOWED_TRANSITIONS = {
    (IDLE, SERVO),
    (SERVO, IDLE),
    (IDLE, POSITION),
    (POSITION, IDLE),
    (SERVO, POSITION),
    (POSITION, SERVO),
}
LEASE_TIMEOUT_S = 5.0
# D-093/D-094: the frozen loopback config reads the place pose from this static TF.
PLACE_FRAME = "place_target_01"
PLACE_POSITION = (0.6, 0.1, 0.25)
# Top-grasp pose in the world frame (TCP +X down, closing axis horizontal).
PLACE_ORIENTATION = (0.0, 2**-0.5, 0.0, 2**-0.5)


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _slerp(q0, q1, fraction: float):
    dot = sum(a * b for a, b in zip(q0, q1, strict=True))
    if dot < 0:
        q1 = tuple(-value for value in q1)
        dot = -dot
    if dot > 0.9995:
        result = tuple(a + (b - a) * fraction for a, b in zip(q0, q1, strict=True))
        norm = math.sqrt(sum(value * value for value in result))
        return tuple(value / norm for value in result)
    theta = math.acos(_clamp(dot, -1.0, 1.0))
    sin_theta = math.sin(theta)
    w0 = math.sin((1 - fraction) * theta) / sin_theta
    w1 = math.sin(fraction * theta) / sin_theta
    return tuple(w0 * a + w1 * b for a, b in zip(q0, q1, strict=True))


def _stamp(seconds: float):
    from builtin_interfaces.msg import Time

    seconds = max(0.0, seconds)
    return Time(sec=int(seconds), nanosec=int((seconds - int(seconds)) * 1e9))


def _transform(parent: str, child: str, xyz, xyzw, stamp) -> TransformStamped:
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = float(xyz[0])
    message.transform.translation.y = float(xyz[1])
    message.transform.translation.z = float(xyz[2])
    message.transform.rotation.x = float(xyzw[0])
    message.transform.rotation.y = float(xyzw[1])
    message.transform.rotation.z = float(xyzw[2])
    message.transform.rotation.w = float(xyzw[3])
    return message


class MockArm(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ros2_mock_arm")
        self.prefix = args.prefix
        self.base_frame = args.base_frame
        self.tcp_frame = args.tcp_frame
        self.object_frame = args.object_frame
        self.motion_duration = args.motion_duration
        self.width_per_rad = args.width_per_rad
        self.max_width = args.max_width
        self.contact_width = args.contact_width
        self.state = IDLE
        self.lease_id = 0
        self.lease_deadline = 0.0
        self.gripper_width = 0.0
        # Arm joint positions follow streamed /arm/joint_command targets (the mock
        # double has no FK): this is what makes the per-round home converge.
        self.arm_joints = (0.0,) * len(JOINT_NAMES[:-1])
        initial = (
            (args.initial_x, args.initial_y, args.initial_z),
            (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)),
        )
        self.current = initial
        self.trajectory: tuple[tuple, tuple, tuple, tuple, float, float] | None = None

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.joint_publisher = self.create_publisher(
            JointState, f"{self.prefix}/joint_states", reliable
        )
        self.state_publisher = self.create_publisher(Int32, f"{self.prefix}/state", reliable)
        self.tf_publisher = self.create_publisher(
            TFMessage, "/tf", QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.tf_static_publisher = self.create_publisher(
            TFMessage,
            "/tf_static",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            JointState, f"{self.prefix}/joint_command", self._on_command, reliable
        )
        self.create_service(AcquireAuthority, f"{self.prefix}/authority/acquire", self._on_acquire)
        self.create_service(ReleaseAuthority, f"{self.prefix}/authority/release", self._on_release)
        self.create_service(SwitchState, f"{self.prefix}/switch_state", self._on_switch)
        self.create_service(PlanRequest, f"{self.prefix}/plan_request", self._on_plan)
        self._publish_static_tf()
        self.timer = self.create_timer(0.01, self._tick)

    # -- publishers --------------------------------------------------------

    def _publish_static_tf(self) -> None:
        stamp = _stamp(time.time())
        static = TFMessage(
            transforms=[
                _transform("world", self.base_frame, (0.0, 0.0, 0.0), (0, 0, 0, 1), stamp),
                _transform(
                    "world",
                    self.object_frame,
                    (0.5, 0.0, 0.2),
                    (0, 0, 0, 1),
                    stamp,
                ),
                _transform(
                    "world",
                    PLACE_FRAME,
                    PLACE_POSITION,
                    PLACE_ORIENTATION,
                    stamp,
                ),
            ]
        )
        self.tf_static_publisher.publish(static)

    def _tick(self) -> None:
        now = time.monotonic()
        if self.trajectory is not None:
            start_position, start_orientation, end_position, end_orientation, started, duration = (
                self.trajectory
            )
            fraction = 0.0 if duration <= 0 else _clamp((now - started) / duration, 0.0, 1.0)
            position = tuple(
                start + (end - start) * fraction
                for start, end in zip(start_position, end_position, strict=True)
            )
            orientation = _slerp(start_orientation, end_orientation, fraction)
            self.current = (position, orientation)
            if fraction >= 1.0:
                self.trajectory = None
        if self.lease_id and now > self.lease_deadline:
            self.lease_id = 0
        self._publish_joint_states()
        self.state_publisher.publish(Int32(data=self.state))
        self.tf_publisher.publish(
            TFMessage(
                transforms=[
                    _transform(
                        self.base_frame,
                        self.tcp_frame,
                        self.current[0],
                        self.current[1],
                        _stamp(time.time()),
                    )
                ]
            )
        )

    def _publish_joint_states(self) -> None:
        angle = self.gripper_width / self.width_per_rad
        self.joint_publisher.publish(
            JointState(
                header=__import__("std_msgs.msg", fromlist=["Header"]).Header(
                    stamp=_stamp(time.time())
                ),
                name=list(JOINT_NAMES),
                position=[*self.arm_joints, angle],
                velocity=[0.0] * 6,
                effort=[0.0] * 6,
            )
        )

    # -- subscriptions -----------------------------------------------------

    def _on_command(self, message: JointState) -> None:
        if tuple(message.name) != JOINT_NAMES or len(message.position) != 6:
            self.get_logger().warning("ignoring malformed joint_command")
            return
        if self.lease_id:
            self.lease_deadline = time.monotonic() + LEASE_TIMEOUT_S
        self.arm_joints = tuple(float(value) for value in message.position[:-1])
        commanded = _clamp(float(message.position[-1]) * self.width_per_rad, 0.0, self.max_width)
        # Simple contact simulation: a closing command reports the cube contact width.
        self.gripper_width = (
            self.contact_width if commanded < self.contact_width + 0.01 else commanded
        )

    # -- services ----------------------------------------------------------

    def _on_acquire(self, request, response):
        if request.target_topic != f"{self.prefix}/joint_command":
            response.success, response.message = False, "target_topic is not whitelisted"
            return response
        if self.lease_id:
            response.success, response.message = False, "authority already held"
            return response
        self.lease_id += 1
        self.lease_deadline = time.monotonic() + LEASE_TIMEOUT_S
        response.success, response.lease_id, response.message = True, self.lease_id, ""
        return response

    def _on_release(self, request, response):
        if request.lease_id != self.lease_id or not self.lease_id:
            response.success, response.message = (
                False,
                "lease id does not match the active lease",
            )
            return response
        self.lease_id = 0
        response.success, response.message = True, ""
        return response

    def _on_switch(self, request, response):
        if request.state_id not in (IDLE, SERVO, POSITION):
            response.success, response.message = False, f"unknown state {request.state_id}"
            return response
        if (self.state, request.state_id) not in ALLOWED_TRANSITIONS:
            response.success, response.message = (
                False,
                f"illegal transition {self.state}->{request.state_id}",
            )
            return response
        if request.state_id in (SERVO, POSITION) and not self.lease_id:
            response.success, response.message = False, "no authority for motion mode"
            return response
        self.state = request.state_id
        response.success, response.message = True, ""
        return response

    def _on_plan(self, request, response):
        if not self.lease_id:
            response.success, response.message = False, "no authority"
            return response
        frame = request.target_pose.header.frame_id
        if not frame or frame.startswith("/"):
            response.success, response.message = False, "invalid target frame"
            return response
        pose = request.target_pose.pose
        target = (
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        norm = math.sqrt(sum(value * value for value in target[1]))
        if not all(math.isfinite(value) for value in target[0] + target[1]) or norm <= 0:
            response.success, response.message = False, "invalid target pose"
            return response
        target = (target[0], tuple(value / norm for value in target[1]))
        position, orientation = self.current
        self.trajectory = (
            position,
            orientation,
            target[0],
            target[1],
            time.monotonic(),
            self.motion_duration,
        )
        self.lease_deadline = time.monotonic() + LEASE_TIMEOUT_S
        response.success, response.message = True, ""
        return response


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", default="/arm")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--tcp-frame", default="gripper_frame_link")
    parser.add_argument("--object-frame", default="objects_cube_01")
    parser.add_argument("--motion-duration", type=float, default=0.2)
    parser.add_argument("--width-per-rad", type=float, default=0.1 / 1.74533)
    parser.add_argument("--max-width", type=float, default=0.1)
    parser.add_argument("--contact-width", type=float, default=0.0445)
    parser.add_argument("--initial-x", type=float, default=0.4)
    parser.add_argument("--initial-y", type=float, default=0.0)
    parser.add_argument("--initial-z", type=float, default=0.4)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = MockArm(args)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    print(json.dumps({"event": "mock_ready", "prefix": args.prefix}), flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
