"""Helpers for the S1 rule_base instance E2E test (tests/integration/test_instance_entry.py).

Kept out of the test module only for the repository 800-line guideline: process/log
helpers, rclpy spin/service helpers, round-drive helpers and the shared-domain
(ROS_DOMAIN_ID=229) preflight live here.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import pytest

ROOT = Path(__file__).resolve().parents[2]  # rule_base/
WORKSPACE = ROOT.parent  # yayyrobotics/
MOCK_ARM = ROOT / "tests/integration/ros2_mock_arm.py"
WRAPPER = ROOT / "tests/integration/instance_entry_mock.py"
#: mock fixtures 单点 owner（P5 迁移后）= <ws>/factory_manager/local/integration/mocks/；
#: 其他仓库只经 MOCK_FIXTURES_DIR 只读消费，不复制副本（contracts R3）。
MOCK_FIXTURES_DIR = Path(
    os.environ.get(
        "MOCK_FIXTURES_DIR",
        str(WORKSPACE / "factory_manager" / "local" / "integration" / "mocks"),
    )
)
FAKE_MANAGER = MOCK_FIXTURES_DIR / "fake_inference_manager.py"
INSTANCE_CONFIG = ROOT / "configs/pick_place/mock_ros2_instance.yaml"

DOMAIN_ID = 229
MARKER_ENV = "RULE_BASE_S1_TEST_ID"
EXPECTED_JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
STOPPED_TOPIC = "/inference/stopped"
RESET_TOPIC = "/env/reset_status"
READY_TOPIC = "/inference_server/ready"
JOINT_COMMAND_TOPIC = "/arm/joint_command"
JOINT_STATES_TOPIC = "/arm/joint_states"
ROUND_TIMEOUT_S = 180.0
LEASE_TIMEOUT_S = 5.0  # ros2_mock_arm.py LEASE_TIMEOUT_S

pytestmark = pytest.mark.integration

MOCK_FIXTURES_FIX = (
    "mock fixtures 缺失（{}）：单点 owner 在 "
    "<ws>/factory_manager/local/integration/mocks/；修复: "
    "export MOCK_FIXTURES_DIR=/home/ubuntu/yayyrobotics/factory_manager/local/integration/mocks"
    "（或 git -C /home/ubuntu/yayyrobotics/factory_manager checkout -- local/integration/mocks）"
)


def require_mock_fixtures() -> Path:
    """校验 mock fixtures（FM 单点 owner）完整；缺失时 skip 并给修复命令。"""
    required = ("fake_inference_manager.py", "fake_instance.py", "common.py")
    missing = [name for name in required if not (MOCK_FIXTURES_DIR / name).is_file()]
    if missing:
        pytest.skip(MOCK_FIXTURES_FIX.format(MOCK_FIXTURES_DIR) + f"；缺失: {missing}")
    return MOCK_FIXTURES_DIR


# ---------------------------------------------------------------------------
# helpers: subprocesses / logs / /proc
# ---------------------------------------------------------------------------
@dataclass
class Collector:
    """Message collector for a single-threaded rclpy spin harness."""

    messages: list[Any] = field(default_factory=list)
    timestamps: list[float] = field(default_factory=list)

    def callback(self, message: Any) -> None:
        self.messages.append(message)
        self.timestamps.append(time.monotonic())

    def last(self) -> Any | None:
        return self.messages[-1] if self.messages else None

    def values(self) -> list[bool]:
        return [bool(message.data) for message in self.messages]

    def count_value(self, value: bool) -> int:
        return sum(1 for message in self.messages if bool(message.data) is value)


class ProcessHandle:
    """Child process with its own session (killing the group reaps grandchildren)."""

    def __init__(
        self, name: str, proc: subprocess.Popen, log_path: Path, log_file: Any
    ) -> None:
        self.name = name
        self.proc = proc
        self.log_path = log_path
        self._log_file = log_file

    def alive(self) -> bool:
        return self.proc.poll() is None

    def wait(self, timeout: float = 10.0) -> int | None:
        try:
            return self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def signal_group(self, sig: int) -> None:
        try:
            os.killpg(os.getpgid(self.proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            try:
                self.proc.send_signal(sig)
            except ProcessLookupError:
                pass

    def terminate(self, timeout: float = 10.0) -> int | None:
        if self.proc.poll() is None:
            self.signal_group(signal.SIGTERM)
            code = self.wait(timeout)
            if code is None:
                self.signal_group(signal.SIGKILL)
                code = self.wait(5.0)
        self.close_log()
        return self.proc.poll()

    def close_log(self) -> None:
        try:
            self._log_file.close()
        except OSError:
            pass

    def log_text(self) -> str:
        try:
            return self.log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""

    def tail(self, lines: int = 120) -> str:
        return "\n".join(self.log_text().splitlines()[-lines:])


def spawn(name: str, argv: list[str], log_dir: Path, env: dict[str, str] | None = None) -> ProcessHandle:
    log_path = log_dir / f"{name}.log"
    log_file = log_path.open("wb")
    proc = subprocess.Popen(
        argv,
        env=env if env is not None else os.environ.copy(),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    print(f"[it] started {name}: pid={proc.pid} log={log_path}")
    return ProcessHandle(name, proc, log_path, log_file)


def wait_log(handle: ProcessHandle, needle: str, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if needle in handle.log_text():
            return
        if not handle.alive():
            break
        time.sleep(0.05)
    pytest.fail(f"{description}: {needle!r} not found in {handle.name} log\n{handle.tail()}")


def marker_pids(marker: str) -> list[int]:
    """All pids whose environment contains this test's unique marker."""
    needle = f"{MARKER_ENV}={marker}".encode()
    pids: list[int] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            environ = (entry / "environ").read_bytes()
        except OSError:
            continue
        if needle in environ.split(b"\0"):
            pids.append(int(entry.name))
    return pids


def pid_desc(pid: int) -> str:
    try:
        cmdline = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(
            errors="replace"
        )
        return f"{pid}: {cmdline.strip()}"
    except OSError:
        return f"{pid}: <exited>"


# ---------------------------------------------------------------------------
# helpers: rclpy spin / services / publishers
# ---------------------------------------------------------------------------
def spin(node: Any, seconds: float) -> None:
    import rclpy

    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def spin_until(
    node: Any,
    description: str,
    predicate: Callable[[], Any],
    timeout: float,
    *,
    detail: Callable[[], str] | None = None,
) -> Any:
    import rclpy

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        value = predicate()
        if value:
            return value
    extra = f"\n{detail()}" if detail is not None else ""
    raise AssertionError(f"timed out after {timeout:g}s waiting for: {description}{extra}")


def wait_services(node: Any, clients: list[Any], names: list[str], timeout: float) -> None:
    import rclpy

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if all(client.service_is_ready() for client in clients):
            return
        rclpy.spin_once(node, timeout_sec=0.05)
    ready = [name for name, client in zip(names, clients, strict=True) if client.service_is_ready()]
    raise AssertionError(f"services not discovered within {timeout:g}s: {set(names) - set(ready)}")


def call_service(node: Any, client: Any, request: Any, description: str, timeout: float = 10.0) -> Any:
    future = client.call_async(request)
    spin_until(node, f"{description} response", future.done, timeout)
    response = future.result()
    assert response is not None, f"{description}: empty service response"
    return response


def publish_for(node: Any, publisher: Any, value: bool, duration: float, hz: float = 5.0) -> None:
    """Publish a Bool level for ``duration`` seconds (mimics env_app's 2 Hz level)."""
    import rclpy
    from std_msgs.msg import Bool

    message = Bool(data=value)
    period = 1.0 / hz
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=max(0.0, min(period, deadline - time.monotonic())))


def publish_until(
    node: Any,
    publisher: Any,
    value: bool,
    predicate: Callable[[], Any],
    description: str,
    timeout: float,
    hz: float = 5.0,
) -> Any:
    import rclpy
    from std_msgs.msg import Bool

    message = Bool(data=value)
    period = 1.0 / hz
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=max(0.0, min(period / 2.0, deadline - time.monotonic())))
        if predicate():
            return True
    raise AssertionError(f"timed out after {timeout:g}s waiting for: {description}")


def hold_payload(collector: Collector) -> tuple[list[str], list[float]] | None:
    latest = collector.last()
    if latest is None or not latest.name or len(latest.name) != len(latest.position):
        return None
    return list(latest.name), [float(value) for value in latest.position]


def prime_lease(node: Any, publisher: Any, joint_states: Collector) -> bool:
    """FM-style hold command used only until the instance has taken the lease over."""
    from sensor_msgs.msg import JointState

    payload = hold_payload(joint_states)
    if payload is None:
        import rclpy

        rclpy.spin_once(node, timeout_sec=0.05)
        return False
    names, positions = payload
    message = JointState()
    message.header.stamp = node.get_clock().now().to_msg()
    message.name = names
    message.position = positions
    publisher.publish(message)
    return True


# ---------------------------------------------------------------------------
# shared-domain preflight (S1 domain 229 is also used by station_support SUP-P1)
# ---------------------------------------------------------------------------
def _foreign_tf_offenders(messages: list[Any]) -> list[str]:
    """TF frames that another publisher parented differently than the mock arm.

    The mock arm publishes world -> {base_link, objects_cube_01, place_target_01};
    station_support (contracts §7, parent_frame=base_link) publishes
    base_link -> objects_cube_01 / place_target_01, which makes the rule_base TF
    buffer reject the graph ("conflicting TF parent/source").
    """
    offenders: list[str] = []
    for message in messages:
        for transform in message.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            if child in ("objects_cube_01", "place_target_01") and parent != "world":
                offenders.append(f"{parent} -> {child}")
    return offenders


def _foreign_node_names(node: Any) -> list[str]:
    """Other S1 E2E nodes visible on domain 229 (ours are not created yet)."""
    markers = (
        "station_support",
        "arm_stack",
        "so101_arm_app",
        "mock_inference_manager",
        "rule_base_",
    )
    foreign: list[str] = []
    for name, namespace in node.get_node_names_and_namespaces():
        if namespace not in ("/", ""):
            continue
        if name == "ros2_mock_arm":
            continue  # this test's own mock arm
        if any(marker in name for marker in markers):
            foreign.append(name)
    mock_arms = sum(1 for name, namespace in node.get_node_names_and_namespaces() if name == "ros2_mock_arm")
    if mock_arms > 1:
        foreign.append(f"{mock_arms}x ros2_mock_arm (another arm double running)")
    return foreign


def wait_for_quiet_domain(node: Any, *, timeout_s: float = 90.0) -> None:
    """Block until domain 229 has no competing S1 session, else fail with a hint.

    contracts §8 freezes 229 for both S1 tasks (rule_base instance + station_support),
    so their E2E runs must be serial. Concurrent station_support TF publishing makes
    the rule_base child fail on a TF parent conflict - do not debug the instance for
    that; wait for the other session or ask the Lead to serialize.
    """
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    def qos(durability: Any, depth: int) -> Any:
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=durability,
        )

    deadline = time.monotonic() + timeout_s
    attempt = 0
    while True:
        attempt += 1
        tf = Collector()
        tf_static = Collector()
        subscriptions = [
            node.create_subscription(TFMessage, "/tf", tf.callback, qos(DurabilityPolicy.VOLATILE, 200)),
            node.create_subscription(
                TFMessage, "/tf_static", tf_static.callback, qos(DurabilityPolicy.TRANSIENT_LOCAL, 100)
            ),
        ]
        try:
            spin(node, 1.2)
            reasons: list[str] = []
            offenders = _foreign_tf_offenders(tf_static.messages + tf.messages)
            if offenders:
                reasons.append("foreign TF parents: " + ", ".join(sorted(set(offenders))))
            joint_publishers = node.count_publishers(JOINT_STATES_TOPIC)
            if joint_publishers > 1:
                reasons.append(
                    f"{joint_publishers} publishers on {JOINT_STATES_TOPIC} "
                    "(expected only this test's mock arm)"
                )
            state_publishers = node.count_publishers("/arm/state")
            if state_publishers > 1:
                reasons.append(f"{state_publishers} publishers on /arm/state")
            if node.count_publishers(STOPPED_TOPIC) > 0:
                reasons.append(f"foreign publisher on {STOPPED_TOPIC} (another instance running)")
            if node.count_publishers(READY_TOPIC) > 0:
                reasons.append(
                    f"foreign publisher on {READY_TOPIC} (another fake inference manager running)"
                )
            foreign_nodes = _foreign_node_names(node)
            if foreign_nodes:
                reasons.append("foreign ROS nodes: " + ", ".join(foreign_nodes))
        finally:
            for subscription in subscriptions:
                try:
                    node.destroy_subscription(subscription)
                except Exception:  # pragma: no cover - best-effort teardown
                    pass
        if not reasons:
            if attempt > 1:
                print(f"[it] domain 229 became quiet after {attempt} probes")
            return
        if time.monotonic() >= deadline:
            pytest.fail(
                f"ROS domain {DOMAIN_ID} is not quiet after {timeout_s:g}s: {'; '.join(reasons)}.\n"
                "S1 tasks share the frozen domain 229 (contracts §8): the rule_base instance E2E "
                "and the station_support SUP-P1 ROS 2 integration test must run serially. "
                "Stop the other S1 session (or ask the Lead to serialize) and re-run this test."
            )
        print(f"[it] domain 229 busy (probe {attempt}): {'; '.join(reasons)}; waiting 2 s ...")
        time.sleep(2.0)


# ---------------------------------------------------------------------------
# round helpers
# ---------------------------------------------------------------------------
def false_since(collector: Collector, start: int) -> bool:
    return any(bool(message.data) is False for message in collector.messages[start:])


def round_finished(collector: Collector, start: int) -> bool:
    window = collector.messages[start:]
    if not window:
        return False
    return any(bool(message.data) is False for message in window) and bool(window[-1].data) is True


def round_transitions(collector: Collector) -> int:
    """Count True->False edges = number of started rounds (stopped is a 2 Hz level)."""
    transitions = 0
    previous: bool | None = None
    for message in collector.messages:
        value = bool(message.data)
        if value is False and previous is not False:
            transitions += 1
        previous = value
    return transitions


def assert_hold_stream(
    joint_command: Collector,
    joint_states: Collector,
    start: int,
    round_number: int,
) -> float:
    """Assert the idle hold stream is 2-5 Hz and carries measured joint positions."""
    window = joint_command.messages[start:]
    timestamps = joint_command.timestamps[start:]
    assert len(window) >= 2, (
        f"round {round_number}: entry hold stream did not resume after the round "
        f"(messages={len(window)}, total={len(joint_command.messages)}, "
        f"last_command_at={[round(ts, 2) for ts in timestamps[-5:]]}); "
        "is the keepalive publish gated off or did the instance exit?"
    )
    if len(timestamps) >= 2 and timestamps[-1] > timestamps[0]:
        rate = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
    else:  # pragma: no cover - defensive
        rate = 0.0
    assert 1.5 <= rate <= 6.0, (
        f"round {round_number}: hold stream rate {rate:.2f} Hz outside the 2-5 Hz band "
        f"(messages={len(window)})"
    )
    recent_samples = joint_states.messages[-300:]
    assert recent_samples, f"round {round_number}: no /arm/joint_states samples collected"
    for message in window:
        assert list(message.name) == list(EXPECTED_JOINTS), (
            f"round {round_number}: hold names {list(message.name)} != frozen six-joint order"
        )
        values = [float(value) for value in message.position]
        matched = any(
            len(sample.position) == len(values)
            and all(abs(a - float(b)) < 1e-6 for a, b in zip(values, sample.position, strict=True))
            for sample in recent_samples
        )
        assert matched, (
            f"round {round_number}: hold position {values} does not match any measured "
            "/arm/joint_states sample (keepalive must target measured positions)"
        )
    return rate


def run_round(
    node: Any,
    *,
    reset_pub: Any,
    stopped: Collector,
    joint_command: Collector,
    joint_states: Collector,
    switch_client: Any,
    SwitchState: Any,
    round_number: int,
) -> dict[str, float]:
    """Drive one False->True reset edge, wait for the round, then prove lease keepalive."""
    publish_for(node, reset_pub, False, duration=0.8, hz=5.0)
    start = len(stopped.messages)
    publish_until(
        node,
        reset_pub,
        True,
        lambda: false_since(stopped, start),
        description=f"round {round_number}: stopped=False after the reset False->True edge",
        timeout=10.0,
    )
    round_started_at = time.monotonic()
    publish_until(
        node,
        reset_pub,
        True,
        lambda: round_finished(stopped, start),
        description=f"round {round_number}: stopped=True after the round finished",
        timeout=ROUND_TIMEOUT_S,
    )
    duration = time.monotonic() - round_started_at
    assert duration < ROUND_TIMEOUT_S, f"round {round_number} took {duration:.1f}s"

    # The child exited before stopped=True (entry publishes True after reaping it).
    hold_start = len(joint_command.messages)
    spin(node, 1.0)
    rate = assert_hold_stream(joint_command, joint_states, hold_start, round_number)

    # Wait beyond the 5 s arm lease timeout with no other publisher: a switch to IDLE
    # only succeeds if the entry keepalive refreshed the lease meanwhile.
    # contracts R14（manage_lease:false）：实例轮末保持 SERVO(3)、由 FM 统一 switch_idle，
    # 所以这里只切 IDLE(0)；切回 SERVO(3) 留给下一轮 entry 子进程启动时自己 0->3 完成
    # （若在此切回 3，下一轮 entry 的 switch(3) 会变成 3->3 非法）。
    idle_deadline = round_started_at + duration + LEASE_TIMEOUT_S + 0.6
    remaining = idle_deadline - time.monotonic()
    if remaining > 0:
        spin(node, remaining)
    for state_id in (0,):
        response = call_service(
            node,
            switch_client,
            SwitchState.Request(state_id=state_id),
            f"round {round_number}: switch_state({state_id}) (lease proof)",
        )
        assert response.success, (
            f"round {round_number}: /arm/switch_state({state_id}) denied after the round: "
            f"{response.message} -> entry keepalive did not hold the lease"
        )
    return {"round": float(round_number), "duration_s": duration, "hold_hz": rate}


# ---------------------------------------------------------------------------
# log evidence
# ---------------------------------------------------------------------------
def parse_events(log_text: str) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for line in log_text.splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(payload, dict) and "event" in payload:
            events.append(payload)
    return events


def assert_manager_log_evidence(log_text: str, inference_id: int) -> dict[str, Any]:
    events = parse_events(log_text)
    names = [event["event"] for event in events]
    assert "instance_ready" in names, f"entry never reached instance_ready\n{log_text[-4000:]}"
    started = [event for event in events if event["event"] == "round_started"]
    finished = [event for event in events if event["event"] == "round_finished"]
    failed = [event for event in events if event["event"] == "instance_failed"]
    assert [event["round"] for event in started] == [1, 2, 3], started
    assert [event["round"] for event in finished] == [1, 2, 3], finished
    assert not failed, f"instance reported failure: {failed}"
    topics = next(event for event in events if event["event"] == "instance_topics")
    assert topics["stopped"] == STOPPED_TOPIC
    assert topics["reset_status"] == RESET_TOPIC
    assert topics["joint_states"] == JOINT_STATES_TOPIC
    assert topics["joint_command"] == JOINT_COMMAND_TOPIC
    ready = next(event for event in events if event["event"] == "instance_ready")
    assert ready["inference_id"] == inference_id
    return {
        "instance_ready": True,
        "rounds": [event["round"] for event in started],
        "failure_events": failed,
    }


