"""Purpose: Keep the ported repository free of native CORA and loadable offline.

Usage: source ros2_ws/setup_ros_env.sh && python -m pytest tests/smoke/test_repository_contract.py
"""

from pathlib import Path

import pytest
import tomllib
import yaml

from rule_base.config.load import load_settings

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"


def _python_sources() -> list[Path]:
    return [path for path in SRC.rglob("*.py") if "__pycache__" not in path.parts]


def test_no_source_file_references_cora():
    forbidden = ("import cora", "from cora", "cora.msg", "CORA_MANIFEST", "ensure_cora")
    offenders = []
    for path in _python_sources():
        text = path.read_text(encoding="utf-8")
        for needle in forbidden:
            if needle in text:
                offenders.append(f"{path.relative_to(ROOT)}: {needle}")
    assert not offenders, "native CORA references remain: " + "; ".join(offenders)


def test_no_cora_payload_directories_remain():
    for relative in ("deps", "packages", "configs/drawer", "configs/press"):
        assert not (ROOT / relative).exists(), f"{relative} must be removed by the port"
    # S1: submission.yaml runtime.kind=uv requires a committed uv.lock (03 plan), so the
    # old port-cleanup rule "uv.lock must not exist" no longer applies.
    assert (ROOT / "uv.lock").is_file(), "uv.lock must exist for runtime.kind=uv (submission.yaml)"


def test_pyproject_has_no_native_dependency():
    data = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))
    dependencies = " ".join(data["project"]["dependencies"]).lower()
    assert "cora" not in dependencies
    assert data["project"]["scripts"]["so101-rule-base"] == "rule_base.interfaces.node:main"


def test_every_declared_config_loads():
    configs = [
        ROOT / "configs/pick_place/offline.yaml",
        ROOT / "configs/pick_place/live.yaml",
        ROOT / "configs/pick_place/live_ros2.yaml",
        # S1 instance-mode configs (03/05 plans): all must stay loadable.
        ROOT / "configs/pick_place/live_ros2_vision_instance.yaml",
        ROOT / "configs/pick_place/live_ros2_teach.yaml",
        ROOT / "configs/pick_place/mock_ros2_instance.yaml",
        # Env position-subscription mode (env_app TF object + tf_static place).
        ROOT / "configs/pick_place/live_ros2_env.yaml",
    ]
    for config in configs:
        settings = load_settings(config)
        assert settings.task == "pick"
        assert settings.ros2.prefix == "/arm"


def test_instance_configs_disable_lease_management():
    """D5/contracts §2: instance configs keep acquire_control but never manage the lease."""
    for name in (
        "mock_ros2_instance.yaml",
        "live_ros2_vision_instance.yaml",
        "live_ros2_teach.yaml",
        "live_ros2_env.yaml",
    ):
        settings = load_settings(ROOT / "configs/pick_place" / name)
        assert settings.arm.acquire_control is True, name
        assert settings.arm.manage_lease is False, name


def test_router_targets_an_existing_pick_config():
    router = yaml.safe_load((ROOT / "configs/live_router.yaml").read_text(encoding="utf-8"))
    assert set(router) == {"by_station"}
    assert set(router["by_station"]) == {"pick_place"}
    target = ROOT / "configs" / router["by_station"]["pick_place"]
    assert target.is_file()
    assert load_settings(target).task == "pick"


@pytest.mark.parametrize("removed", ["stations/drawer", "stations/press", "infrastructure/cora"])
def test_removed_packages_stay_removed(removed):
    assert not (SRC / "rule_base" / removed).exists()


def test_ros2_adapter_sources_exist():
    for relative in (
        "infrastructure/ros2/arm_topic.py",
        "infrastructure/ros2/observations/transport.py",
        "infrastructure/transform/observations.py",
    ):
        assert (SRC / "rule_base" / relative).is_file()
