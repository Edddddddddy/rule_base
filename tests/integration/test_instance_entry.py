"""Purpose: end-to-end proof of the real rule_base 026 instance entry (S1 DoD, mock only).

Scenario (contracts §2 / §9, 03 plan §3):

1. start ``tests/integration/ros2_mock_arm.py`` (arm double with authority/lease);
2. the test harness plays factory_manager: acquires the arm lease, then starts the FM-owned
   mock ``fake_inference_manager.py --instance-script tests/integration/instance_entry_mock.py``
   (single owner: ``<ws>/factory_manager/local/integration/mocks/``);
3. launch with a positive timeout / ``pick_place`` -> ready contains the id -> the real
   entry publishes ``/inference/stopped=True`` (2 Hz, transient_local);
4. three rounds: harness drives ``/env/reset_status`` False (>=0.6 s level) -> True and
   asserts ``stopped`` goes False (round start) then True (round end, child exited);
5. after every round the harness waits >5 s (arm lease timeout) and then calls
   ``/arm/switch_state(0)``: it must succeed, proving the entry keepalive kept the lease
   alive while idle (the child's own traffic stopped when it exited). contracts R14:
   with ``manage_lease:false`` the instance keeps SERVO(3) at round end and the next
   round's entry re-enters SERVO from IDLE, so the harness only switches IDLE here;
6. the idle ``/arm/joint_command`` hold stream is ~2-5 Hz and carries the latest measured
   ``/arm/joint_states`` positions (gripper held);
7. after round 3 a sustained True level for 2 s must not start a round (edge semantics);
8. ``/inference_server/stop`` -> ready removes the id -> entry reaped with exit code 0;
   SIGTERM the fake manager -> exit 0 -> no process with this test's env marker remains.

Caveat: domain 229 is shared with the station_support SUP-P1 task (contracts §8), so the
test waits for a quiet domain first; S1 E2E runs must be serial.

Usage:
    cd /home/ubuntu/yayyrobotics
    export ROS_DOMAIN_ID=229
    source ros2_ws/setup_ros_env.sh
    export PYTHONPATH=/home/ubuntu/yayyrobotics/rule_base/src:$PYTHONPATH
    source /home/ubuntu/yayyrobotics/inference_manager/outputs/ros2-interfaces/install/setup.bash
                                                      # inference_manager_msgs（IM 唯一 owner；先 bash scripts/build_ros2_interfaces.sh）
    cd rule_base
    python3 -m pytest tests/integration/test_instance_entry.py -q -s
"""


from __future__ import annotations

import json
import os
import signal
import sys
import tempfile
import time
import uuid
from pathlib import Path
from typing import Any

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from instance_e2e_support import (  # noqa: E402
    DOMAIN_ID,
    FAKE_MANAGER,
    INSTANCE_CONFIG,
    JOINT_COMMAND_TOPIC,
    JOINT_STATES_TOPIC,
    MARKER_ENV,
    MOCK_ARM,
    READY_TOPIC,
    RESET_TOPIC,
    ROOT,
    ROUND_TIMEOUT_S,
    STOPPED_TOPIC,
    WRAPPER,
    Collector,
    ProcessHandle,
    assert_manager_log_evidence,
    call_service,
    hold_payload,
    marker_pids,
    parse_events,
    pid_desc,
    prime_lease,
    publish_for,
    require_mock_fixtures,
    round_transitions,
    run_round,
    spawn,
    spin,
    spin_until,
    wait_for_quiet_domain,
    wait_log,
    wait_services,
)

pytestmark = pytest.mark.integration


# ---------------------------------------------------------------------------
# environment gates
# ---------------------------------------------------------------------------
def _require_ros2() -> None:
    try:
        import arm_msgs.srv  # noqa: F401
        import inference_manager_msgs.srv  # noqa: F401
        import rclpy  # noqa: F401
        from sensor_msgs.msg import JointState  # noqa: F401
    except ImportError as exc:  # pragma: no cover - depends on the shell environment
        pytest.skip(
            "ROS 2 / mock overlay not sourced: "
            "export ROS_DOMAIN_ID=229 && source ros2_ws/setup_ros_env.sh && "
            "export PYTHONPATH=/home/ubuntu/yayyrobotics/rule_base/src:$PYTHONPATH && "
            "source /home/ubuntu/yayyrobotics/inference_manager/outputs/ros2-interfaces/install/setup.bash"
            f"（接口缺失时先 bash scripts/build_ros2_interfaces.sh；{exc}）"
        )


def _require_domain_229() -> None:
    raw = os.environ.get("ROS_DOMAIN_ID", "")
    if not raw:
        pytest.skip(
            "ROS_DOMAIN_ID is not set; S1 integration runs on the registered domain 229: "
            "export ROS_DOMAIN_ID=229 before sourcing ros2_ws/setup_ros_env.sh"
        )
    try:
        domain = int(raw)
    except ValueError:
        pytest.fail(f"ROS_DOMAIN_ID={raw!r} is not an integer; S1 tests require 229")
    if domain == 0:
        pytest.fail(
            "ROS_DOMAIN_ID=0 is the real-robot domain; mock/instance tests must use 229 "
            "(contracts §8)"
        )
    if domain != DOMAIN_ID:
        pytest.fail(
            f"ROS_DOMAIN_ID={domain} is not the S1 registered domain {DOMAIN_ID} "
            "(contracts §8); export ROS_DOMAIN_ID=229 before sourcing the ROS env"
        )


# ---------------------------------------------------------------------------
# the test
# ---------------------------------------------------------------------------
def test_instance_entry_three_rounds_keepalive_and_sigterm(tmp_path):
    _require_ros2()
    _require_domain_229()
    assert MOCK_ARM.is_file(), MOCK_ARM
    assert WRAPPER.is_file(), WRAPPER
    require_mock_fixtures()
    assert INSTANCE_CONFIG.is_file(), INSTANCE_CONFIG

    import rclpy
    from arm_msgs.srv import AcquireAuthority, ReleaseAuthority, SwitchState
    from inference_manager_msgs.srv import Launch, Stop
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Int32MultiArray

    marker = f"rb-s1-{uuid.uuid4().hex}"
    # Keep the E2E logs outside pytest's shared tmp base (parallel sessions rotate it):
    # override with RB_INSTANCE_E2E_LOG_DIR when debugging locally.
    raw_log_dir = os.environ.get("RB_INSTANCE_E2E_LOG_DIR", "")
    log_dir = Path(raw_log_dir) if raw_log_dir else Path(tempfile.mkdtemp(prefix="rb-instance-e2e-"))
    log_dir.mkdir(parents=True, exist_ok=True)
    print(f"[it] domain={os.environ.get('ROS_DOMAIN_ID')} logs={log_dir}")
    arm: ProcessHandle | None = None
    manager: ProcessHandle | None = None
    node = None
    rclpy.init()
    evidence: dict[str, Any] = {}
    started_at = time.monotonic()

    def _dump_logs() -> None:
        for handle in (arm, manager):
            if handle is not None:
                print(f"\n===== {handle.name} log (tail) =====\n{handle.tail(160)}")

    try:
        try:
            node = Node(f"rb_instance_it_{os.getpid()}_{uuid.uuid4().hex[:8]}")

            def qos(durability: Any, depth: int = 1) -> Any:
                return QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=depth,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=durability,
                )

            stopped = Collector()
            ready = Collector()
            joint_command = Collector()
            joint_states = Collector()
            node.create_subscription(Bool, STOPPED_TOPIC, stopped.callback, qos(DurabilityPolicy.TRANSIENT_LOCAL))
            node.create_subscription(
                Int32MultiArray, READY_TOPIC, ready.callback, qos(DurabilityPolicy.TRANSIENT_LOCAL)
            )
            node.create_subscription(
                JointState, JOINT_COMMAND_TOPIC, joint_command.callback, qos(DurabilityPolicy.VOLATILE, 200)
            )
            node.create_subscription(
                JointState, JOINT_STATES_TOPIC, joint_states.callback, qos(DurabilityPolicy.VOLATILE, 200)
            )
            reset_pub = node.create_publisher(Bool, RESET_TOPIC, qos(DurabilityPolicy.VOLATILE))
            hold_pub = node.create_publisher(
                JointState, JOINT_COMMAND_TOPIC, qos(DurabilityPolicy.VOLATILE)
            )
            acquire_client = node.create_client(AcquireAuthority, "/arm/authority/acquire")
            release_client = node.create_client(ReleaseAuthority, "/arm/authority/release")
            switch_client = node.create_client(SwitchState, "/arm/switch_state")
            launch_client = node.create_client(Launch, "/inference_server/launch")
            stop_client = node.create_client(Stop, "/inference_server/stop")

            # 1) arm double first: the entry warmup needs <prefix>/joint_states.
            arm = spawn(
                "mock_arm",
                [sys.executable, str(MOCK_ARM), "--prefix", "/arm"],
                log_dir,
            )
            wait_log(arm, '"mock_ready"', 20.0, "mock arm")
            wait_services(
                node,
                [acquire_client, release_client, switch_client],
                ["acquire", "release", "switch_state"],
                20.0,
            )
            # S1 domain 229 is shared with station_support: refuse to run concurrently
            # (its base_link->objects_cube_01 TF conflicts with the mock arm's static TF).
            wait_for_quiet_domain(node)
            spin_until(
                node,
                "first /arm/joint_states sample",
                lambda: hold_payload(joint_states) is not None,
                15.0,
            )

            # 2) harness plays factory_manager: acquire the lease (contracts §3).
            lease_response = call_service(
                node,
                acquire_client,
                AcquireAuthority.Request(target_topic=JOINT_COMMAND_TOPIC),
                "acquire authority",
            )
            assert lease_response.success, f"acquire denied: {lease_response.message}"
            lease_id = int(lease_response.lease_id)
            assert lease_id > 0, lease_response
            print(f"[it] lease acquired: lease_id={lease_id}")
            for _ in range(2):
                assert prime_lease(node, hold_pub, joint_states)
                spin(node, 0.1)

            # 3) fake manager launches the real entry via the wrapper script.
            manager_env = os.environ.copy()
            manager_env[MARKER_ENV] = marker
            manager = spawn(
                "fake_manager",
                [
                    sys.executable,
                    str(FAKE_MANAGER),
                    "--instance-script",
                    str(WRAPPER),
                    "--stop-timeout",
                    "10",
                ],
                log_dir,
                env=manager_env,
            )
            wait_services(
                node,
                [launch_client, stop_client],
                ["launch", "stop"],
                20.0,
            )
            spin_until(
                node,
                f"first {READY_TOPIC} message",
                lambda: ready.last() is not None,
                10.0,
            )

            # Keep the lease alive as FM would until the instance is ready (entry keepalive
            # takes over from there; priming stops as soon as stopped=True is observed).
            launch_request = Launch.Request()
            launch_request.repo_url = str(ROOT)
            launch_request.commit_hash = ""
            launch_request.timeout = int(ROUND_TIMEOUT_S)
            launch_request.namespace = ""
            launch_request.station_type = "pick_place"
            launch_request.client_id = "s1-instance-it"
            launch_request.lease_id = lease_id
            for _ in range(2):
                assert prime_lease(node, hold_pub, joint_states)
                spin(node, 0.1)
            launch_response = call_service(node, launch_client, launch_request, "launch")
            assert launch_response.success, f"launch not accepted: {launch_response.message}"
            assert launch_response.error_code == 0, launch_response
            inference_id = int(launch_response.inference_id)
            assert inference_id >= 1, launch_response
            print(f"[it] launch accepted: inference_id={inference_id}")

            deadline = time.monotonic() + 30.0
            next_prime = 0.0
            while time.monotonic() < deadline:
                ready_ids = list(ready.last().data) if ready.last() is not None else []
                stopped_true = stopped.last() is not None and bool(stopped.last().data) is True
                if inference_id in ready_ids and stopped_true:
                    break
                if time.monotonic() >= next_prime:
                    prime_lease(node, hold_pub, joint_states)
                    next_prime = time.monotonic() + 0.25
                spin(node, 0.05)
            else:
                pytest.fail(
                    f"instance not ready within 30s: ready={ready.values()} "
                    f"stopped={stopped.values()}\n{manager.tail()}"
                )
            print(f"[it] ready contains {inference_id}; stopped=True observed")
            assert stopped.count_value(True) >= 1

            # 2 Hz stopped level while idle (contracts §1.1).
            idle_before = len(stopped.messages)
            spin(node, 1.0)
            idle_rate = len(stopped.messages) - idle_before
            assert 1 <= idle_rate <= 4, f"stopped idle rate {idle_rate}/s is not ~2 Hz"
            print(f"[it] stopped idle rate: ~{idle_rate}/s")

            # 4/5/6) three reset-edge rounds + keepalive/lease proof per round.
            round_evidence = []
            for round_number in (1, 2, 3):
                assert manager.alive(), f"manager died before round {round_number}\n{manager.tail()}"
                assert stopped.last() is not None and bool(stopped.last().data) is True, (
                    f"round {round_number}: instance not idle (stopped=True) at round start"
                )
                result = run_round(
                    node,
                    reset_pub=reset_pub,
                    stopped=stopped,
                    joint_command=joint_command,
                    joint_states=joint_states,
                    switch_client=switch_client,
                    SwitchState=SwitchState,
                    round_number=round_number,
                )
                round_evidence.append(result)
                print(
                    f"[it] round#{round_number} ok: stopped False->True in "
                    f"{result['duration_s']:.1f}s; idle hold ~{result['hold_hz']:.1f} Hz; "
                    "lease switch_state(0) succeeded after >5 s idle (R14: keep SERVO)"
                )
            evidence["rounds"] = round_evidence
            assert round_transitions(stopped) == 3, (
                f"expected exactly 3 started rounds, got {round_transitions(stopped)}"
            )

            # 7) sustained True level must not start a 4th round (contracts §2.3).
            false_before = stopped.count_value(False)
            publish_for(node, reset_pub, True, duration=2.0, hz=5.0)
            assert stopped.count_value(False) == false_before, (
                "a continuously latched True reset_status level started a new round "
                f"(False messages {false_before} -> {stopped.count_value(False)})"
            )
            assert round_transitions(stopped) == 3
            print("[it] sustained True level after round 3 did not trigger a 4th round")
            assert manager.alive(), f"manager died\n{manager.tail()}"
            assert arm is not None and arm.alive(), "mock arm died during the rounds"

            # 8) stop the instance: ready removes the id and the entry exits 0.
            stop_response = call_service(
                node,
                stop_client,
                Stop.Request(inference_id=inference_id),
                "stop instance",
            )
            assert stop_response.success, f"stop not accepted: {stop_response.message}"
            assert stop_response.error_code == 0, stop_response
            wait_log(
                manager,
                "cleaned up: reclaimed (exit_code=0)",
                25.0,
                "entry reaped with exit code 0 after /inference_server/stop",
            )
            spin_until(
                node,
                f"ready removes inference_id={inference_id}",
                lambda: inference_id not in (list(ready.last().data) if ready.last() else []),
                15.0,
            )
            evidence["stop"] = {"ready_removed": True, "entry_exit_code": 0}
            print("[it] stop accepted; ready id removed; entry reaped with exit code 0")

            # SIGTERM the fake manager -> exit 0, then no test-marked process may remain.
            manager.proc.send_signal(signal.SIGTERM)
            manager_code = manager.wait(15.0)
            assert manager_code == 0, f"fake manager SIGTERM exit code {manager_code}\n{manager.tail()}"
            manager.close_log()
            manager_events = parse_events(manager.log_text())
            evidence["instance_log"] = assert_manager_log_evidence(
                manager.log_text(), inference_id
            )
            assert not any(event["event"] == "instance_failed" for event in manager_events)

            deadline = time.monotonic() + 10.0
            remaining = marker_pids(marker)
            while remaining and time.monotonic() < deadline:
                time.sleep(0.1)
                remaining = marker_pids(marker)
            assert not remaining, (
                "processes with this test's instance marker still running: "
                + ", ".join(pid_desc(pid) for pid in remaining)
            )
            evidence["no_residual_processes"] = True
            print("[it] fake manager exit 0; no residual instance/entry processes")
            print(f"[it] evidence: {json.dumps(evidence, default=str)}")
        except BaseException:
            if manager is not None and (
                "conflicting TF parent" in manager.log_text()
                or "invalid TF input" in manager.log_text()
            ):
                print(
                    "\n[it] NOTE: the rule_base child failed on a TF parent conflict; this "
                    "is cross-session interference on the shared S1 domain 229 "
                    "(station_support publishes base_link->objects_cube_01) - re-run the S1 "
                    "E2E tests serially, the instance protocol itself is not implicated."
                )
            _dump_logs()
            raise
    finally:
        if manager is not None:
            manager.terminate(timeout=10.0)
        if arm is not None:
            arm.terminate(timeout=10.0)
        for pid in marker_pids(marker):
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:  # pragma: no cover - best-effort teardown
                pass
        try:
            rclpy.shutdown()
        except Exception:  # pragma: no cover
            pass
        print(f"[it] cleanup done in {time.monotonic() - started_at:.1f}s")
