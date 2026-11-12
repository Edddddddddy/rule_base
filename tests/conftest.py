"""Purpose: Share the offline Pick configuration across unit tests.

Usage: source ros2_ws/setup_ros_env.sh && python -m pytest tests/unit tests/smoke
"""

from pathlib import Path

import pytest

from rule_base.config.load import load_settings
from rule_base.config.pick_place import PickSettings


@pytest.fixture
def settings() -> PickSettings:
    return load_settings(Path(__file__).resolve().parents[1] / "configs/pick_place/offline.yaml")
