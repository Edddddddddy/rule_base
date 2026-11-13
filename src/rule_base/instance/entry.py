"""026 instance entrypoint for the SO-101 rule_base program (contracts §1.2/§2).

The inference manager launches this module (``python3 -m rule_base.instance.entry
--config <yaml> [--dry-run]``) instead of the station CLI. It implements the frozen
instance protocol:

* read ``INFERENCE_NAMESPACE`` / ``DOMAIN_ID`` / ``STATION_TYPE`` / ``CLIENT_ID`` /
  ``LEASE_ID`` (contracts §1.2);
* ``rclpy.init`` -> warmup (first ``<prefix>/joint_states`` sample, 30 s default) ->
  publish ``<ns>/inference/stopped=True`` at 2 Hz (RELIABLE + TRANSIENT_LOCAL depth 1);
* subscribe ``<ns>/env/reset_status`` and arm/start rounds on a strict False->True
  edge (continuously latched True never starts another round; resets received while a
  round runs are dropped);
* one round = publish ``stopped=False`` -> run a fresh ``rule_base.interfaces.node``
  child (one round, live or ``--dry-run --transport ros2``) -> publish ``stopped=True``;
* publish the ``<prefix>/joint_command`` hold heartbeat (latest measured joint
  positions, 2-5 Hz) while idle and between rounds, strictly paused while a child runs
  so the child owns the arm traffic;
* SIGTERM/SIGINT -> terminate the child (SIGTERM -> 5 s grace -> SIGKILL), stop timers,
  ``rclpy.shutdown`` and exit 0; a failed round logs ``stage``/``error``/
  ``inference_id`` and exits 1; config/env/warmup failures exit 2.

The instance never calls the arm authority acquire/release services: the factory
manager owns the lease and ``arm.manage_lease: false`` is enforced by the instance
configuration. No ROS imports happen at module import time, so the failure paths and
unit tests work without a sourced ROS 2 environment.
"""

from __future__ import annotations

import argparse
import os
import signal
import time
from pathlib import Path
from typing import Any

from rule_base.instance.keepalive import DEFAULT_HOLD_HZ, HoldCommand, Keepalive
from rule_base.instance.protocol import (
    EXIT_CONFIG,
    EXIT_OK,
    EXIT_RUNTIME,
    InstanceConfigError,
    InstanceEnv,
    InstanceProtocol,
    event_line,
    exit_code_for_round,
    parse_instance_env,
)
from rule_base.instance.runner import (
    DEFAULT_ROUND_TIMEOUT_S,
    DEFAULT_STOP_GRACE_S,
    RoundRunner,
)

#: Warmup budget while waiting for the first ``<prefix>/joint_states`` sample.
DEFAULT_WARMUP_TIMEOUT_S = 30.0
#: contracts §1.1: stopped is a 2 Hz level topic.
STOPPED_PERIOD_S = 0.5
#: Leave >1 stopped publish period for the final ``stopped=True`` to be delivered
#: before a failed instance exits (contracts §2).
FAILURE_PUBLISH_DRAIN_S = 0.7
#: QoS depth for the measured joint feedback and hold command topics.
FEEDBACK_QOS_DEPTH = 10
SPIN_PERIOD_S = 0.05


def emit(name: str, **fields: Any) -> None:
    """Print one structured JSON log line to stdout (asserted by tests/manager logs)."""
    print(event_line(name, **fields), flush=True)


class _Shutdown:
    """SIGTERM/SIGINT latch shared by the main loop and the child runner."""

    def __init__(self) -> None:
        self.requested = False
        self.signum: int | None = None


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="rule_base.instance.entry",
        description="rule_base 026 instance entry (contracts §2)",
    )
    parser.add_argument("--config", type=Path, required=True, help="rule_base Pick YAML")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="child rounds run offline (--dry-run --transport ros2); tests only, never on hardware",
    )
    parser.add_argument(
        "--warmup-timeout",
        type=float,
        default=DEFAULT_WARMUP_TIMEOUT_S,
        metavar="S",
        help="seconds to wait for the first <prefix>/joint_states sample (default: %(default)s)",
    )
    parser.add_argument(
        "--round-timeout",
        type=float,
        default=DEFAULT_ROUND_TIMEOUT_S,
        metavar="S",
        help="single-round child timeout in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--hold-hz",
        type=float,
        default=DEFAULT_HOLD_HZ,
        metavar="HZ",
        help="hold heartbeat rate, must be within 2-5 Hz (default: %(default)s)",
    )
    parser.add_argument(
        "--stop-grace",
        type=float,
        default=DEFAULT_STOP_GRACE_S,
        metavar="S",
        help="SIGTERM->SIGKILL grace for the child at shutdown (default: %(default)s)",
    )
    args = parser.parse_args(argv)
    if args.warmup_timeout <= 0:
        parser.error("--warmup-timeout must be positive")
    if args.round_timeout <= 0:
        parser.error("--round-timeout must be positive")
    if not 2.0 <= args.hold_hz <= 5.0:
        parser.error("--hold-hz must be within 2-5 Hz (contracts §2 D4)")
    if args.stop_grace < 0:
        parser.error("--stop-grace must not be negative")
    return args


def _load_settings(config_path: Path) -> Any:
    from rule_base.config.load import load_settings

    return load_settings(config_path)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        settings = _load_settings(args.config)
    except Exception as exc:  # noqa: BLE001 - classify every config failure as exit 2
        emit("instance_failed", stage="config", error=f"{type(exc).__name__}: {exc}")
        return EXIT_CONFIG
    try:
        env = parse_instance_env(os.environ)
    except InstanceConfigError as exc:
        emit("instance_failed", stage="environment", error=str(exc))
        return EXIT_CONFIG
    for warning in env.warnings:
        emit("instance_warning", warning=warning)
    try:
        keepalive = Keepalive(settings.ros2.joint_names, hz=args.hold_hz)
    except ValueError as exc:
        emit("instance_failed", stage="config", error=str(exc))
        return EXIT_CONFIG
    # contracts §1.2: DOMAIN_ID is mandatory; when the process did not inherit
    # ROS_DOMAIN_ID, derive it from DOMAIN_ID before rclpy.init reads it.
    if not (os.environ.get("ROS_DOMAIN_ID") or "").strip():
        os.environ["ROS_DOMAIN_ID"] = str(env.domain_id)
    emit(
        "instance_starting",
        namespace=env.namespace,
        domain_id=env.domain_id,
        station_type=env.station_type,
        client_id=env.client_id,
        lease_id=env.lease_id,
        inference_id=env.inference_id,
        dry_run=bool(args.dry_run),
        config=str(args.config.expanduser().resolve()),
    )
    return _serve(args, env, settings, keepalive)


def _serve(args: argparse.Namespace, env: InstanceEnv, settings: Any, keepalive: Keepalive) -> int:
    """Initialize rclpy and run the protocol loop; all ROS imports stay in this path."""
    try:
        import rclpy
    except ImportError as exc:  # pragma: no cover - depends on the shell environment
        emit(
            "instance_failed",
            stage="ros2_init",
            error=f"rclpy is unavailable ({exc}); source ros2_ws/setup_ros_env.sh",
        )
        return EXIT_CONFIG
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool

    shutdown = _Shutdown()
    runner = RoundRunner(
        args.config,
        dry_run=bool(args.dry_run),
        timeout_s=args.round_timeout,
        stop_grace_s=args.stop_grace,
    )

    def _handle_signal(signum: int, _frame: Any) -> None:
        shutdown.requested = True
        shutdown.signum = signum
        runner.interrupt()

    previous_handlers: dict[int, Any] = {}
    for signum in (signal.SIGTERM, signal.SIGINT):
        try:
            previous_handlers[signum] = signal.signal(signum, _handle_signal)
        except (ValueError, OSError):  # pragma: no cover - non-main thread
            pass
    try:
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception as exc:  # noqa: BLE001 - ROS init failure = configuration error
        emit("instance_failed", stage="ros2_init", error=f"{type(exc).__name__}: {exc}")
        _restore_signal_handlers(previous_handlers)
        return EXIT_CONFIG

    node = None
    executor = None
    app: _InstanceApp | None = None
    try:
        node = Node(f"rule_base_instance_{os.getpid()}")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        app = _InstanceApp(
            node=node,
            executor=executor,
            settings=settings,
            env=env,
            keepalive=keepalive,
            runner=runner,
            shutdown=shutdown,
            warmup_timeout_s=args.warmup_timeout,
            rclpy_module=rclpy,
            external_shutdown=ExternalShutdownException,
            joint_state_type=JointState,
            bool_type=Bool,
        )
        return app.run()
    except Exception as exc:  # noqa: BLE001 - never leak a running child on crashes
        emit(
            "instance_failed",
            stage="runtime",
            error=f"{type(exc).__name__}: {exc}",
            inference_id=env.inference_id,
        )
        return EXIT_RUNTIME
    finally:
        runner.interrupt()
        if app is not None:
            app.close()
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception:  # pragma: no cover - best-effort teardown
                pass
            try:
                executor.shutdown()
            except Exception:  # pragma: no cover
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:  # pragma: no cover
                pass
        try:
            rclpy.shutdown()
        except Exception:  # pragma: no cover - already shut down
            pass
        _restore_signal_handlers(previous_handlers)


def _restore_signal_handlers(previous: dict[int, Any]) -> None:
    for signum, handler in previous.items():
        try:
            signal.signal(signum, handler)
        except (ValueError, OSError):  # pragma: no cover - non-main thread
            pass


class _InstanceApp:
    """rclpy wiring for one instance process (stopped/reset/hold + round loop)."""

    def __init__(
        self,
        *,
        node: Any,
        executor: Any,
        settings: Any,
        env: InstanceEnv,
        keepalive: Keepalive,
        runner: RoundRunner,
        shutdown: _Shutdown,
        warmup_timeout_s: float,
        rclpy_module: Any,
        external_shutdown: type[BaseException],
        joint_state_type: type,
        bool_type: type,
    ) -> None:
        self.node = node
        self.executor = executor
        self.settings = settings
        self.env = env
        self.keepalive = keepalive
        self.runner = runner
        self.shutdown = shutdown
        self.warmup_timeout_s = float(warmup_timeout_s)
        self.rclpy = rclpy_module
        self._external_shutdown = external_shutdown
        self._bool_type = bool_type
        self._joint_state_type = joint_state_type
        self.gate = InstanceProtocol()
        self._round_requested = False
        self._stopped_timer: Any = None
        self._hold_timer: Any = None
        self._closed = False

        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        stopped_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reset_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        feedback_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=FEEDBACK_QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        ros2 = settings.ros2
        self.stopped_pub = node.create_publisher(bool_type, env.stopped_topic, stopped_qos)
        self.hold_pub = node.create_publisher(
            joint_state_type, ros2.joint_command_topic, feedback_qos
        )
        node.create_subscription(bool_type, env.reset_status_topic, self._on_reset_status, reset_qos)
        node.create_subscription(
            joint_state_type, ros2.joint_states_topic, self._on_joint_states, feedback_qos
        )
        emit(
            "instance_topics",
            stopped=env.stopped_topic,
            reset_status=env.reset_status_topic,
            joint_states=ros2.joint_states_topic,
            joint_command=ros2.joint_command_topic,
            prefix=ros2.prefix,
        )

    # -- ROS callbacks -----------------------------------------------------
    def _on_reset_status(self, message: Any) -> None:
        if self.gate.on_reset_status(bool(message.data)):
            self._round_requested = True

    def _on_joint_states(self, message: Any) -> None:
        self.keepalive.on_joint_states(message.name, message.position)

    def _on_stopped_timer(self) -> None:
        if self.shutdown.requested:
            return
        self._publish_stopped(self.gate.stopped_value)

    def _on_hold_timer(self) -> None:
        if self.shutdown.requested:
            return
        command = self.keepalive.next_command()
        if command is None:
            return
        self._publish_hold(command)

    # -- helpers -----------------------------------------------------------
    def _publish_stopped(self, value: bool) -> None:
        self.stopped_pub.publish(self._bool_type(data=bool(value)))

    def _publish_hold(self, command: HoldCommand) -> None:
        message = self._joint_state_type()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = list(command.names)
        message.position = [float(value) for value in command.positions]
        self.hold_pub.publish(message)

    def _spin_once(self, timeout_s: float) -> None:
        if not self.rclpy.ok():
            return
        try:
            self.executor.spin_once(timeout_sec=timeout_s)
        except self._external_shutdown:  # pragma: no cover - external rclpy shutdown
            self.shutdown.requested = True

    def _drain(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            if self.shutdown.requested or not self.rclpy.ok():
                return
            self._spin_once(SPIN_PERIOD_S)

    # -- lifecycle ---------------------------------------------------------
    def run(self) -> int:
        try:
            warmup_code = self._warmup()
            if warmup_code is not None:
                return warmup_code
            ros2 = self.settings.ros2
            emit(
                "instance_ready",
                namespace=self.env.namespace,
                inference_id=self.env.inference_id,
                stopped_topic=self.env.stopped_topic,
                reset_status_topic=self.env.reset_status_topic,
                joint_states_topic=ros2.joint_states_topic,
                joint_command_topic=ros2.joint_command_topic,
                hold_hz=self.keepalive.hz,
            )
            self.gate.mark_ready()
            self._publish_stopped(True)
            self._stopped_timer = self.node.create_timer(
                STOPPED_PERIOD_S, self._on_stopped_timer
            )
            self._hold_timer = self.node.create_timer(
                self.keepalive.period_s, self._on_hold_timer
            )
            self.keepalive.start()
            while not self.shutdown.requested:
                if not self.rclpy.ok():  # pragma: no cover - external shutdown
                    break
                self._spin_once(SPIN_PERIOD_S)
                if self._round_requested and not self.shutdown.requested:
                    self._round_requested = False
                    code = self._run_round()
                    if code is not None:
                        return code
            emit(
                "instance_stopping",
                signal=self.shutdown.signum,
                rounds_completed=self.gate.rounds_completed,
                rounds_failed=self.gate.rounds_failed,
                discarded_resets=self.gate.discarded_resets,
            )
            return EXIT_OK
        finally:
            self.close()

    def _warmup(self) -> int | None:
        """Wait for the first joint_states sample (contracts §2 warmup/connection check)."""
        deadline = time.monotonic() + self.warmup_timeout_s
        while time.monotonic() < deadline:
            if self.shutdown.requested:
                emit("instance_stopping", signal=self.shutdown.signum, stage="warmup")
                return EXIT_OK
            if not self.rclpy.ok():  # pragma: no cover - external shutdown
                return EXIT_RUNTIME
            self._spin_once(SPIN_PERIOD_S)
            if self.keepalive.has_sample:
                return None
        emit(
            "instance_failed",
            stage="warmup",
            error=(
                f"no sample on {self.settings.ros2.joint_states_topic} within "
                f"{self.warmup_timeout_s:g}s"
            ),
            inference_id=self.env.inference_id,
        )
        return EXIT_CONFIG

    def _run_round(self) -> int | None:
        """Run one child round; return None to keep serving or the instance exit code."""
        round_number = self.gate.rounds_started
        self.keepalive.stop()
        self._publish_stopped(False)
        emit("round_started", round=round_number, inference_id=self.env.inference_id)
        try:
            outcome = self.runner.run(on_tick=lambda: self._spin_once(0.02))
        except Exception as exc:  # noqa: BLE001 - an unexpected runner bug still fails cleanly
            self.gate.on_round_finished(failed=True)
            emit(
                "instance_failed",
                stage="round",
                error=f"{type(exc).__name__}: {exc}",
                inference_id=self.env.inference_id,
                round=round_number,
            )
            self._publish_stopped(True)
            self._drain(FAILURE_PUBLISH_DRAIN_S)
            return EXIT_OK if self.shutdown.requested else EXIT_RUNTIME
        exit_code = exit_code_for_round(outcome.status)
        if exit_code is None:
            self.gate.on_round_finished()
            self._publish_stopped(True)
            self.keepalive.start()
            emit(
                "round_finished",
                round=round_number,
                status="completed",
                rounds_completed=self.gate.rounds_completed,
                returncode=outcome.returncode,
            )
            return None
        self.gate.on_round_finished(failed=exit_code != EXIT_OK)
        if exit_code == EXIT_OK:
            emit(
                "round_interrupted",
                round=round_number,
                returncode=outcome.returncode,
                signal=self.shutdown.signum,
            )
            return EXIT_OK
        emit(
            "instance_failed",
            stage=outcome.stage,
            error=outcome.error or outcome.status,
            inference_id=self.env.inference_id,
            exit_code=outcome.returncode,
            round=round_number,
        )
        self._publish_stopped(True)
        self._drain(FAILURE_PUBLISH_DRAIN_S)
        if self.shutdown.requested:
            return EXIT_OK
        return EXIT_RUNTIME

    def close(self) -> None:
        """Idempotent teardown: stop the hold heartbeat and the published timers."""
        if self._closed:
            return
        self._closed = True
        self.keepalive.stop()
        for timer in (self._stopped_timer, self._hold_timer):
            if timer is None:
                continue
            try:
                timer.cancel()
            except Exception:  # pragma: no cover - best-effort teardown
                pass


if __name__ == "__main__":
    raise SystemExit(main())
