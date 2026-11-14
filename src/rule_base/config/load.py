"""Load once, validate before imports or network construction."""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path

import yaml

from rule_base.config.pick_place import PickSettings

STATION_TASKS = {"pick_place": "pick"}
TASK_STATIONS = {task: station for station, task in STATION_TASKS.items()}


def load_settings(path: Path) -> PickSettings:
    with path.open(encoding="utf-8") as stream:
        values = yaml.safe_load(stream)
    if not isinstance(values, dict):
        raise ValueError("config must be a YAML mapping")
    if values.get("task") != "pick":
        raise ValueError("config task must be pick")
    profiles = values.pop("profile_files", None)
    pick_profile = values.pop("profile_file", None)
    if profiles is not None:
        raise ValueError("profile_files is only valid for multi-panel station configs")
    if pick_profile is not None:
        if values.get("task") != "pick" or "profile" in values:
            raise ValueError("profile_file is only valid for pick without profile")
        with (path.parent / str(pick_profile)).open(encoding="utf-8") as stream:
            values["profile"] = yaml.safe_load(stream)
    hold_duration = os.environ.get("PICK_HOLD_DURATION_S")
    if hold_duration is not None:
        profile = values.get("profile")
        if not isinstance(profile, dict):
            raise ValueError("Pick profile must be a YAML mapping")
        profile["hold_duration_s"] = hold_duration
    env_target_id = os.environ.get("RULE_BASE_TARGET_OBJECT_ID")
    env_frames = values.get("env_frames")
    if env_target_id is not None and isinstance(env_frames, dict):
        # Upstream object selection (env position-subscription mode only; other
        # coordinate sources ignore the variable). An explicit empty value is a
        # config error rather than a silent fallback to the YAML id.
        selected = env_target_id.strip()
        if not selected:
            raise ValueError(
                "RULE_BASE_TARGET_OBJECT_ID must not be empty when set "
                "(unset it to keep the env_frames.target_object_id from the config)"
            )
        section = dict(env_frames)
        section["target_object_id"] = selected
        values["env_frames"] = section
    return PickSettings.model_validate(values)


def task_type(path: Path) -> str:
    with path.open(encoding="utf-8") as stream:
        values = yaml.safe_load(stream)
    if not isinstance(values, dict) or values.get("task") != "pick":
        raise ValueError("config task must be pick")
    return "pick"


def station_type(environ: Mapping[str, str] | None = None) -> str:
    # STATION_TYPE is injected by the inference manager and only required when
    # --config points at the by_station router; direct station configs ignore it.
    env = os.environ if environ is None else environ
    raw = env.get("STATION_TYPE", "").strip()
    if raw not in STATION_TASKS:
        raise ValueError(f"STATION_TYPE must be one of: {', '.join(STATION_TASKS)}")
    return raw


def resolve_config_path(path: Path, environ: Mapping[str, str] | None = None) -> tuple[Path, str]:
    """Resolve the per-station config before any settings load or network setup."""
    with path.open(encoding="utf-8") as stream:
        values = yaml.safe_load(stream)
    if isinstance(values, dict) and "by_station" in values:
        mapping = values["by_station"]
        if not isinstance(mapping, dict) or set(values) != {"by_station"}:
            raise ValueError("router config must contain only a by_station mapping")
        station = station_type(environ)
        target_name = mapping.get(station)
        if not isinstance(target_name, str) or not target_name:
            raise ValueError(f"no config path mapped for STATION_TYPE {station!r} in {path}")
        target = path.parent / target_name
        if not target.is_file():
            raise ValueError(f"router target missing: {target}")
        try:
            matching = task_type(target) == STATION_TASKS[station]
        except ValueError:
            matching = False
        if not matching:
            raise ValueError(f"router target {target} task does not match {station}")
        return target, station
    return path, TASK_STATIONS[task_type(path)]
