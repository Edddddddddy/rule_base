"""Shared offline fixtures for the split Pick station unit test modules.

Purpose: keep the per-topic Pick test modules (core / feedback / place / home and
observation) focused while sharing the offline settings, scene helpers and event
recorder that used to live in the single test_pick_place.py file.

Usage: imported by tests/unit/test_pick_place*.py (same directory is on sys.path
under pytest's default import mode); not collected because the name is not test_*.
"""

import re
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from rule_base.config.load import load_settings
from rule_base.config.pick_place import PickSettings
from rule_base.domain.models import PickScene, Pose
from rule_base.infrastructure.transform.math import from_pose

CONFIG = Path(__file__).resolve().parents[2] / "configs/pick_place/offline.yaml"
PLACE_FRAME = "place_target_01"


def offline_settings() -> PickSettings:
    """Offline fake settings with a zero hold and a single grasp attempt."""
    config = load_settings(CONFIG)
    assert isinstance(config, PickSettings)
    return config.model_copy(
        update={
            "profile": config.profile.model_copy(update={"hold_duration_s": 0}),
            "limits": config.limits.model_copy(update={"max_grasp_attempts": 1}),
        }
    )


def with_place(settings: PickSettings) -> PickSettings:
    """Read the place pose from the fake environment's place_target_01 TF."""
    return settings.model_copy(
        update={"frames": settings.frames.model_copy(update={"place": PLACE_FRAME})}
    )


def with_home(settings: PickSettings, home: tuple[float, ...]) -> PickSettings:
    """Use an explicit folded-rest joint target (defaults to the six zero joints)."""
    return settings.model_copy(
        update={"ros2": settings.ros2.model_copy(update={"home_joint_positions": home})}
    )


def upright_environment(angle=0):
    half = np.deg2rad(angle) / 2
    scene = PickScene(
        from_pose(Pose((0, 0, 0), (0, 0, 0, 1))),
        from_pose(Pose((0.4, 0, 0.3), (0, 0, np.sin(half), np.cos(half)))),
    )
    return SimpleNamespace(
        snapshot=lambda: scene, request_new_observation=lambda: None, check_health=lambda: None
    )


def recorded(events: list[tuple[str, dict[str, object]]], name: str) -> list[dict[str, object]]:
    return [fields for event_name, fields in events if event_name == name]


def record_events(monkeypatch, module) -> list[tuple[str, dict[str, object]]]:
    """Patch a station module's JSON event sink and return the captured events."""
    events: list[tuple[str, dict[str, object]]] = []
    monkeypatch.setattr(module, "event", lambda name, **fields: events.append((name, fields)))
    return events


def stages(events: list[tuple[str, dict[str, object]]]) -> list[str]:
    return [str(fields["stage"]) for name, fields in events if name == "stage"]


def logged_line(text: str, prefix: str) -> str:
    """Return the console plan-log line starting with ``prefix`` (tests only)."""
    line = next((line for line in text.splitlines() if line.startswith(prefix)), None)
    assert line is not None, f"no console line starting with {prefix!r} in {text!r}"
    return line


def logged_values(line: str, label: str, expected) -> None:
    """Assert ``label=[a, b, c]`` in one console plan-log line matches ``expected``.

    The log prints 4 decimals, so comparisons use an absolute 1e-4 tolerance.
    """
    match = re.search(rf"{re.escape(label)}=\[([^\]]+)\]", line)
    assert match is not None, f"{label} is missing from {line!r}"
    values = [float(part) for part in match.group(1).split(",")]
    assert values == pytest.approx(expected, abs=1e-4)
