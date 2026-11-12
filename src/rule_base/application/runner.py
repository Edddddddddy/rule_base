"""Process lifecycle runner and task entrypoint for the Pick station."""

from __future__ import annotations

import asyncio
import signal
import time
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Event
from uuid import uuid4

from rule_base.config.load import TASK_STATIONS, load_settings, task_type
from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import (
    ContractError,
    LifecycleTransport,
    RoundOutcome,
    SnapshotEnvironment,
)
from rule_base.observability.events import event
from rule_base.stations.pick_place.execution import PickRound


class InstanceRunner:
    """Run one or more Pick rounds on one transport/environment pair."""

    def __init__(
        self,
        transport: LifecycleTransport,
        environment: SnapshotEnvironment,
        cycle: Callable[[str], RoundOutcome],
        stop: Event,
        *,
        warmup_timeout_s: float = 10,
        status_period_s: float = 0.5,
        rounds: int = 1,
        before_round: Callable[[], None] | None = None,
        round_id: str | None = None,
        vision_observe_first: bool = False,
    ) -> None:
        if rounds < 1:
            raise ValueError("rounds must be positive")
        self.transport, self.environment, self.cycle, self.stop = (
            transport,
            environment,
            cycle,
            stop,
        )
        self.warmup_timeout_s, self.status_period_s = warmup_timeout_s, status_period_s
        self.rounds, self.before_round = rounds, before_round
        # L6 采集编排（S2-5）：首轮可用编排器注入的 round id，使 recorder metadata 与
        # round_result.round_id 同源；其余轮次/缺省仍内部 uuid4（向后兼容）。
        self.round_id = round_id
        # S6b：vision observe_first 的 warmup 只等机器链（root→arm_base），不等物体 TF
        # （折叠 rest 时视觉节点尚未检测/发布 object TF）。
        self.vision_observe_first = bool(vision_observe_first)

    async def run(self) -> int:
        worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="station-arm")
        future: asyncio.Future[RoundOutcome] | None = None
        terminal_safe = False
        opened = False
        try:
            self.transport.open()
            opened = True
            self._wait_warmup()
            for index in range(self.rounds):
                if self.stop.is_set():
                    return 130
                if self.before_round is not None:
                    self.before_round()
                if index == 0:
                    event("armed")
                else:
                    event("round_started", round=index + 1, rounds=self.rounds)
                round_id = self.round_id if (self.round_id and index == 0) else uuid4().hex
                future = asyncio.get_running_loop().run_in_executor(worker, self.cycle, round_id)
                next_status = time.monotonic() + self.status_period_s
                while not future.done():
                    self.transport.check_health()
                    if time.monotonic() >= next_status:
                        event("heartbeat", running=True)
                        next_status = time.monotonic() + self.status_period_s
                    await asyncio.sleep(0.01)
                result = future.result()
                future = None
                terminal_safe = result.cleanup_confirmed
                if not terminal_safe:
                    raise ContractError("round cleanup unconfirmed")
                if result.status == "failed":
                    return 1
                if result.status == "interrupted":
                    return 130
            return 0
        except Exception as exc:
            # Record the trigger before cooperative cleanup emits interruption logs.
            event("runtime_error", error=str(exc))
            raise
        finally:
            self.stop.set()
            if future is not None:
                try:
                    result = await asyncio.shield(future)
                    terminal_safe = result.cleanup_confirmed
                except Exception:
                    terminal_safe = False
            worker.shutdown(wait=True, cancel_futures=True)
            if opened:
                self.transport.close()
            event("shutdown", cleanup_confirmed=terminal_safe)

    def _wait_warmup(self) -> None:
        deadline = time.monotonic() + self.warmup_timeout_s
        while True:
            if self.stop.is_set():
                raise ContractError("stop requested during warmup")
            self.transport.check_health()
            try:
                if self.vision_observe_first:
                    robot_ready = getattr(self.environment, 'robot_ready', None)
                    if robot_ready is not None:
                        robot_ready()
                    else:  # 兼容旧自定义 environment：退回完整 snapshot
                        self.environment.snapshot()
                else:
                    self.environment.snapshot()
                return
            except (ContractError, ValueError) as exc:
                if time.monotonic() >= deadline:
                    raise ContractError(f"warmup timed out waiting for valid TF: {exc}") from None
            time.sleep(0.02)


def validate_live_grasp_source(settings: PickSettings) -> None:
    """D-142: live grasp coordinates must come from the real-time vision target.

    The legacy path derives the grasp from the object TF (``frames.object``, fed by
    the fixed-point scene publisher); the user requires live position data only, so
    ``--run`` refuses it. ``--dry-run``/fake rounds keep supporting the legacy
    template for tests and the static mock chain.
    """
    if settings.frames.grasp_target is None:
        raise ValueError(
            "live execution requires the vision grasp_target frame (D-142): static "
            "object-frame grasp coordinates are disabled; use "
            "configs/pick_place/live_ros2_vision.yaml"
        )


async def serve(
    config_path: Path,
    *,
    live: bool,
    once: bool = False,
    target_id: int | None = None,
    station: str | None = None,
    transport: str = "fake",
    rounds: int = 1,
    round_id: str | None = None,
) -> int:
    """Assemble the Pick station and own its process signals and transport lifecycle."""
    task = task_type(config_path)
    if target_id is not None:
        raise ValueError("--target is only supported for the retired Press station")
    if transport not in {"fake", "ros2"}:
        raise ValueError("transport must be fake or ros2")
    if live and transport != "ros2":
        raise ValueError("live execution requires the ros2 transport")
    settings = load_settings(config_path)
    if live:
        validate_live_grasp_source(settings)
    effective_transport = "ros2" if live else transport
    stop = Event()
    loop = asyncio.get_running_loop()
    registered: list[signal.Signals] = []
    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            loop.add_signal_handler(sig, stop.set)
            registered.append(sig)
        except (NotImplementedError, RuntimeError):
            pass
    runner: InstanceRunner
    try:
        if effective_transport == "ros2":
            from rule_base.infrastructure.ros2.arm_topic import Ros2ArmTopic
            from rule_base.infrastructure.ros2.observations.transport import Ros2Transport

            ros2_transport = Ros2Transport(settings)
            pick_cycle = PickRound(
                settings,
                ros2_transport,
                lambda: Ros2ArmTopic(settings, ros2_transport, stop),
                stop,
            )
            runner = InstanceRunner(
                ros2_transport,
                ros2_transport,
                pick_cycle.run,
                stop,
                warmup_timeout_s=settings.limits.warmup_timeout_s,
                rounds=rounds,
                round_id=round_id,
                vision_observe_first=bool(
                    settings.vision.enabled and settings.vision.observe_first),
            )
        else:
            from rule_base.infrastructure.fakes.runtime import (
                FakePickArm,
                FakePickEnvironment,
                FakeTransport,
            )

            pick_environment = FakePickEnvironment(settings)
            pick_cycle = PickRound(settings, pick_environment, lambda: FakePickArm(settings), stop)
            runner = InstanceRunner(
                FakeTransport(),
                pick_environment,
                pick_cycle.run,
                stop,
                rounds=rounds,
                before_round=pick_environment.publish_object,
                round_id=round_id,
                vision_observe_first=bool(
                    settings.vision.enabled and settings.vision.observe_first),
            )
        event(
            "startup",
            task="pick_place/pick",
            station=station if station is not None else TASK_STATIONS[task],
            live=live,
            transport=effective_transport,
            rounds=rounds,
            fixture_only=settings.fixture_only,
            field_validated=settings.profile.field_validated,
            hold_duration_s=settings.profile.hold_duration_s,
        )
        return await runner.run()
    finally:
        stop.set()
        for sig in registered:
            loop.remove_signal_handler(sig)


def settings_for(config_path: Path) -> PickSettings:
    """Load a validated Pick configuration (kept for callers that only need config)."""
    return load_settings(config_path)
