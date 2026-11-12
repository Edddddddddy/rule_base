"""Purpose: Verify one-shot and repeated runner lifecycle, cleanup and transport ownership.

Usage: source ros2_ws/setup_ros_env.sh && python -m pytest tests/unit/test_runner.py
"""

import asyncio
from pathlib import Path
from threading import Event

import pytest

from rule_base.application import runner as module
from rule_base.application.runner import InstanceRunner
from rule_base.config.load import load_settings
from rule_base.domain.models import ContractError, RoundResult
from rule_base.infrastructure.fakes.runtime import (
    PICK_MOCK_ROOT_TO_ARM,
    FakePickEnvironment,
    FakeTransport,
    tf_entry,
)
from rule_base.infrastructure.transform.observations import PickObservationCache


def result(*, cleanup=True, status="completed"):
    return RoundResult("1", status, "test", "test result", 0.02, 0.02, cleanup)


def test_runner_executes_one_round_and_closes_transport(settings):
    transport = FakeTransport()
    rounds = []

    def cycle(round_id):
        rounds.append(round_id)
        return result()

    runner = InstanceRunner(transport, FakePickEnvironment(settings), cycle, Event())
    assert asyncio.run(runner.run()) == 0
    assert len(rounds) == 1
    assert transport.closed


def test_runner_runs_configured_rounds(settings):
    transport = FakeTransport()
    rounds = []

    def cycle(round_id):
        rounds.append(round_id)
        return result()

    environment = FakePickEnvironment(settings)
    runner = InstanceRunner(
        transport,
        environment,
        cycle,
        Event(),
        rounds=3,
        before_round=environment.publish_object,
    )
    assert asyncio.run(runner.run()) == 0
    assert len(rounds) == 3
    assert transport.closed


def test_runner_stops_after_failed_round(settings):
    transport = FakeTransport()
    rounds = []

    def cycle(round_id):
        rounds.append(round_id)
        return result(status="failed")

    runner = InstanceRunner(transport, FakePickEnvironment(settings), cycle, Event(), rounds=3)
    assert asyncio.run(runner.run()) == 1
    assert len(rounds) == 1
    assert transport.closed


def test_unconfirmed_cleanup_raises_and_closes_transport(settings):
    transport = FakeTransport()
    runner = InstanceRunner(
        transport,
        FakePickEnvironment(settings),
        lambda _: result(cleanup=False),
        Event(),
    )
    with pytest.raises(ContractError, match="cleanup unconfirmed"):
        asyncio.run(runner.run())
    assert transport.closed


def test_runner_rejects_environment_without_valid_tf(settings):
    transport = FakeTransport()
    environment = FakePickEnvironment(settings)
    runner = InstanceRunner(
        transport,
        environment,
        lambda _: result(),
        Event(),
        warmup_timeout_s=0.01,
    )
    environment.snapshot = lambda: (_ for _ in ()).throw(ContractError("missing TF"))
    with pytest.raises(ContractError, match="warmup timed out waiting for valid TF: missing TF"):
        asyncio.run(runner.run())
    assert transport.closed


def test_heartbeat_continues_while_worker_runs(settings, monkeypatch):
    release = Event()
    heartbeats = []

    def record(name, **values):
        if name == "heartbeat":
            heartbeats.append(values)
            if len(heartbeats) >= 3:
                release.set()

    def cycle(_):
        assert release.wait(2), "heartbeat did not continue during execution"
        return result()

    monkeypatch.setattr(module, "event", record)
    transport = FakeTransport()
    runner = InstanceRunner(
        transport, FakePickEnvironment(settings), cycle, Event(), status_period_s=0.01
    )
    assert asyncio.run(runner.run()) == 0
    assert len(heartbeats) >= 3 and transport.closed


def test_shutdown_waits_for_worker_cleanup(settings):
    async def scenario():
        stop, started, cleaned = Event(), Event(), Event()
        transport = FakeTransport()

        def cycle(_):
            started.set()
            assert stop.wait(2)
            assert not transport.closed
            cleaned.set()
            return result(status="interrupted")

        runner = InstanceRunner(transport, FakePickEnvironment(settings), cycle, stop)
        task = asyncio.create_task(runner.run())

        async def stop_after_start():
            while not started.is_set():
                await asyncio.sleep(0.005)
            stop.set()
            return await task

        try:
            assert await asyncio.wait_for(stop_after_start(), timeout=2) == 130
        finally:
            stop.set()
            await task
        assert cleaned.is_set() and transport.closed

    asyncio.run(scenario())


def test_tf_fault_is_logged_before_worker_stop_and_survives_cleanup(settings, monkeypatch):
    stop, started, cleaned = Event(), Event(), Event()
    events = []
    transport = FakeTransport()

    def check_health():
        if started.is_set():
            raise ContractError("conflicting TF parent/source")

    def cycle(_):
        started.set()
        assert stop.wait(2)
        assert events[-1] == ("runtime_error", {"error": "conflicting TF parent/source"})
        assert not transport.closed
        cleaned.set()
        return result(cleanup=False, status="failed")

    transport.check_health = check_health
    monkeypatch.setattr(module, "event", lambda name, **values: events.append((name, values)))
    runner = InstanceRunner(transport, FakePickEnvironment(settings), cycle, stop)
    with pytest.raises(ContractError, match="conflicting TF parent/source"):
        asyncio.run(runner.run())
    assert cleaned.is_set() and transport.closed
    assert events[-1] == ("shutdown", {"cleanup_confirmed": False})


def test_runner_uses_orchestrator_round_id_for_single_round(settings):
    """S2-5：capture 经 --round-id 注入 round id 时，round_result.round_id 与之同源。"""
    transport = FakeTransport()
    seen: list[str] = []

    def cycle(round_id):
        seen.append(round_id)
        return result()

    runner = InstanceRunner(
        transport,
        FakePickEnvironment(settings),
        cycle,
        Event(),
        round_id="a" * 32,
    )
    assert asyncio.run(runner.run()) == 0
    assert seen == ["a" * 32]


def test_runner_round_id_only_applies_to_first_round(settings):
    """多轮（--rounds > 1）时注入 id 只用于首轮，其余轮次内部 uuid4（不重号）。"""
    transport = FakeTransport()
    seen: list[str] = []

    def cycle(round_id):
        seen.append(round_id)
        return result()

    environment = FakePickEnvironment(settings)
    runner = InstanceRunner(
        transport,
        environment,
        cycle,
        Event(),
        rounds=2,
        before_round=environment.publish_object,
        round_id="b" * 32,
    )
    assert asyncio.run(runner.run()) == 0
    assert len(seen) == 2
    assert seen[0] == "b" * 32
    assert len(seen[1]) == 32 and seen[1] != seen[0]


def test_runner_generates_uuid_round_id_by_default(settings):
    """缺省 --round-id 时保持旧行为：内部 uuid4().hex（32 hex）。"""
    transport = FakeTransport()
    seen: list[str] = []

    def cycle(round_id):
        seen.append(round_id)
        return result()

    runner = InstanceRunner(transport, FakePickEnvironment(settings), cycle, Event())
    assert asyncio.run(runner.run()) == 0
    assert len(seen) == 1 and len(seen[0]) == 32
    int(seen[0], 16)


# --- S6b（task-24）：vision observe_first warmup 只等机器链（root→arm_base） -----------
def _vision_settings(settings):
    return settings.model_copy(update={
        'vision': settings.vision.model_copy(update={'enabled': True,
                                                     'observe_first': True})})


def test_vision_observe_first_warmup_does_not_require_object_tf(settings):
    """vision+observe_first：rest 时无 object TF 也能过 warmup（只等 root→arm_base）。"""
    vision_settings = _vision_settings(settings)
    environment = FakePickEnvironment(vision_settings)
    environment.snapshot = lambda: (_ for _ in ()).throw(
        ContractError('TF path unavailable: base_link <- objects_cube_01'))
    assert environment.robot_ready() is True
    rounds = []
    runner = InstanceRunner(
        FakeTransport(), environment,
        lambda round_id: (rounds.append(round_id), result())[1],
        Event(), warmup_timeout_s=0.01, vision_observe_first=True)
    assert asyncio.run(runner.run()) == 0
    assert len(rounds) == 1


def test_non_vision_warmup_still_requires_full_scene(settings):
    """非 vision（或 observe_first=False）：warmup 仍要求完整 scene，缺 object TF 失败。"""
    environment = FakePickEnvironment(settings)
    environment.snapshot = lambda: (_ for _ in ()).throw(
        ContractError('TF path unavailable: base_link <- objects_cube_01'))
    runner = InstanceRunner(
        FakeTransport(), environment, lambda _: result(), Event(),
        warmup_timeout_s=0.01, vision_observe_first=False)
    with pytest.raises(ContractError, match='objects_cube_01'):
        asyncio.run(runner.run())


def test_observation_cache_robot_ready_ignores_object_tf(settings):
    """PickObservationCache.robot_ready 只查 root→arm_base；snapshot 仍要求 object TF。"""
    cache = PickObservationCache(settings)
    frames = settings.frames
    root_arm = tf_entry(frames.root, frames.arm_base,
                        PICK_MOCK_ROOT_TO_ARM.position, timestamp_ns=0)
    root_arm['transform']['rotation'] = dict(
        zip('xyzw', PICK_MOCK_ROOT_TO_ARM.orientation, strict=True))
    cache.update('tf_static', {'transforms': [root_arm]})
    assert cache.robot_ready() is True
    with pytest.raises(ContractError, match=frames.object):
        cache.snapshot()


def test_vision_observe_first_normal_scene_still_passes(settings):
    """正常 vision（观察后物体 TF 已发布）：robot_ready 分支不破坏正常路径。"""
    vision_settings = _vision_settings(settings)
    environment = FakePickEnvironment(vision_settings)
    rounds = []
    runner = InstanceRunner(
        FakeTransport(), environment,
        lambda round_id: (rounds.append(round_id), result())[1],
        Event(), warmup_timeout_s=0.01, vision_observe_first=True)
    assert asyncio.run(runner.run()) == 0
    assert len(rounds) == 1


# --- D-142：--run 只允许实时视觉 grasp_target（禁用 static 固定物体位姿入口） -----------


def test_live_grasp_source_requires_vision_grasp_target():
    """live_ros2.yaml（只有 frames.object）不得用于 --run；vision 配置通过。"""
    root = Path(__file__).resolve().parents[2]
    static_settings = load_settings(root / "configs/pick_place/live_ros2.yaml")
    with pytest.raises(ValueError, match="grasp_target"):
        module.validate_live_grasp_source(static_settings)
    vision_settings = load_settings(root / "configs/pick_place/live_ros2_vision.yaml")
    module.validate_live_grasp_source(vision_settings)


def test_serve_rejects_live_without_vision_target_before_touching_ros():
    root = Path(__file__).resolve().parents[2]
    with pytest.raises(ValueError, match="grasp_target"):
        asyncio.run(module.serve(
            root / "configs/pick_place/live_ros2.yaml", live=True, transport="ros2"))
