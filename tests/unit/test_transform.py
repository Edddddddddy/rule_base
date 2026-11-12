"""Purpose: Check TF direction, rigid math and timestamp gates.

Usage: source ros2_ws/setup_ros_env.sh && python -m pytest tests/unit/test_transform.py
"""

import math
from contextlib import contextmanager
from copy import deepcopy

import numpy as np
import pytest

from rule_base.domain.models import ContractError, Pose
from rule_base.infrastructure.fakes.runtime import tf_entry
from rule_base.infrastructure.transform.buffer import TransformBuffer
from rule_base.infrastructure.transform.math import (
    compose,
    from_pose,
    inverse,
    split_line,
    to_pose,
)

NOW = 10_000_000_000


def buffer():
    return TransformBuffer(max_age_s=0.3, max_skew_s=0.05)


@pytest.mark.parametrize("quaternion", [(0, 0, 0, 1), (1, 0, 0, 0), (0, 0.6, 0, 0.8)])
def test_pose_roundtrip_and_inverse(quaternion):
    m = from_pose(Pose((0.2, -0.1, 0.4), quaternion))
    assert np.allclose(from_pose(to_pose(m)), m)
    assert np.allclose(compose(m, inverse(m)), np.eye(4))


def test_split_line_uses_fixed_spacing_and_keeps_exact_endpoint():
    start, end = (0.0, 0.0, 0.0), (0.0, 0.0, 0.05)
    assert split_line(start, end, 0.02) == [
        (0.0, 0.0, 0.02),
        (0.0, 0.0, 0.04),
        end,
    ]
    assert split_line(start, start, 0.02) == [start]
    with pytest.raises(ValueError, match="positive"):
        split_line(start, end, 0)


def test_tf_chain_static_epoch_zero_and_inverse():
    b = buffer()
    b.update({"transforms": [tf_entry("root", "base", (1, 0, 0), timestamp_ns=0)]}, static=True)
    b.update({"transforms": [tf_entry("root", "handle", (2, 0, 0), timestamp_ns=NOW)]})
    forward, backward = b.lookup_many([("base", "handle"), ("handle", "base")], now_ns=NOW)
    assert to_pose(forward).position == (1, 0, 0)
    assert to_pose(backward).position == (-1, 0, 0)


def test_lookup_rechecks_age_of_cached_samples():
    b = buffer()
    b.update({"transforms": [tf_entry("root", "handle", (0, 0, 0), timestamp_ns=NOW)]})
    b.lookup_many([("root", "handle")], now_ns=NOW + 300_000_000)
    for stamp in (NOW - 1, NOW + 300_000_001):
        with pytest.raises(ContractError, match="stale or future"):
            b.lookup_many([("root", "handle")], now_ns=stamp)


def test_lookup_reads_wall_clock_after_acquiring_buffer_lock(monkeypatch):
    b = buffer()
    newer_stamp = NOW + 10_000_000
    b.update({"transforms": [tf_entry("root", "handle", (0, 0, 0), timestamp_ns=newer_stamp)]})
    original_lock = b._lock
    locked = False

    @contextmanager
    def delayed_lock():
        nonlocal locked
        with original_lock:
            locked = True
            yield

    monkeypatch.setattr(b, "_lock", delayed_lock())
    monkeypatch.setattr(
        "rule_base.infrastructure.transform.buffer.time.time_ns",
        lambda: newer_stamp if locked else NOW,
    )
    # A wall-clock reading taken before lock acquisition would reject this sample.
    b.lookup_many([("root", "handle")])


@pytest.mark.parametrize("offset_ns,reason", [(1, "future"), (-300_000_001, "stale")])
def test_timestamp_error_reports_edge_age_and_limits(offset_ns, reason):
    b = buffer()
    b.update({"transforms": [tf_entry("root", "handle", (0, 0, 0), timestamp_ns=NOW + offset_ns)]})
    with pytest.raises(ContractError, match="stale or future-dated") as exc:
        b.lookup_many([("root", "handle")], now_ns=NOW)
    message = str(exc.value)
    assert f"reason={reason}; edge=root -> handle" in message
    assert f"age_s={-offset_ns / 1e9:.6f}" in message
    assert "max_age_s=0.300000; future_tolerance_s=0.000000" in message


def test_tf_skew_and_out_of_order():
    b = buffer()
    b.update(
        {
            "transforms": [
                tf_entry("root", "base", (0, 0, 0), timestamp_ns=NOW),
                tf_entry("root", "handle", (1, 0, 0), timestamp_ns=NOW - 60_000_000),
            ]
        }
    )
    with pytest.raises(ContractError, match="skew"):
        b.lookup_many([("base", "handle")], now_ns=NOW)
    b.update({"transforms": [tf_entry("root", "handle", (2, 0, 0), timestamp_ns=NOW)]})
    b.update({"transforms": [tf_entry("root", "handle", (3, 0, 0), timestamp_ns=NOW - 1)]})
    assert to_pose(b.lookup_many([("base", "handle")], now_ns=NOW)[0]).position[0] == 2


def test_lookup_uses_only_connected_path_edges(monkeypatch):
    b = buffer()
    b.update({"transforms": [tf_entry("root", "base", (1, 0, 0), timestamp_ns=0)]}, static=True)
    b.update(
        {
            "transforms": [
                tf_entry("root", "target", (2, 0, 0), timestamp_ns=NOW),
                *(
                    tf_entry("root", f"unrelated_{i}", (0, 0, 0), timestamp_ns=NOW - 1_000_000_000)
                    for i in range(300)
                ),
            ]
        },
        static=False,
    )
    calls = 0
    original_inverse = inverse

    def count_inverse(value):
        nonlocal calls
        calls += 1
        return original_inverse(value)

    monkeypatch.setattr("rule_base.infrastructure.transform.buffer.inverse", count_inverse)
    transform = b.lookup_many([("base", "target")], now_ns=NOW)[0]
    assert to_pose(transform).position == (1, 0, 0)
    assert calls == 1


def test_lookup_rejects_disconnected_roots_and_static_dynamic_pair():
    b = buffer()
    b.update({"transforms": [tf_entry("root_a", "a", (0, 0, 0), timestamp_ns=0)]}, static=True)
    b.update({"transforms": [tf_entry("root_b", "b", (0, 0, 0), timestamp_ns=0)]}, static=True)
    with pytest.raises(ContractError, match="path unavailable"):
        b.lookup_many([("a", "b")], now_ns=NOW)
    with pytest.raises(ContractError, match="no dynamic sample"):
        b.lookup_many([("root_a", "a")], now_ns=NOW, dynamic_pair=("root_a", "a"))


def test_lookup_rotated_branches_preserve_transform_order():
    b = buffer()
    turn = (0, 0, math.sqrt(0.5), math.sqrt(0.5))
    base = tf_entry("root", "base", (1, 0, 0), timestamp_ns=0)
    base["transform"]["rotation"] = dict(zip(("x", "y", "z", "w"), turn, strict=True))
    target = tf_entry("root", "target", (0, 2, 0), timestamp_ns=NOW)
    b.update({"transforms": [base]}, static=True)
    b.update({"transforms": [target]})

    actual = b.lookup_many([("base", "target")], now_ns=NOW)[0]
    expected = compose(
        inverse(from_pose(Pose((1, 0, 0), turn))),
        from_pose(Pose((0, 2, 0), (0, 0, 0, 1))),
    )
    assert np.allclose(actual, expected)


@pytest.mark.parametrize("fault", ["parent", "duplicate", "cycle", "quaternion", "zero", "nan"])
def test_invalid_batches_do_not_replace_valid_cache(fault):
    b = buffer()
    good = tf_entry("root", "handle", (1, 0, 0), timestamp_ns=NOW)
    b.update({"transforms": [good]})
    bad = deepcopy(good)
    items = [bad]
    if fault == "parent":
        bad["header"]["frame_id"] = "other"
    if fault == "duplicate":
        items.append(deepcopy(bad))
    if fault == "cycle":
        items.append(tf_entry("handle", "root", (0, 0, 0), timestamp_ns=NOW))
    if fault == "quaternion":
        bad["transform"]["rotation"]["w"] = 0
    if fault == "zero":
        bad["header"]["stamp"] = {"sec": 0, "nanosec": 0}
    if fault == "nan":
        bad["transform"]["translation"]["x"] = float("nan")
    with pytest.raises(ContractError):
        b.update({"transforms": items})
    assert to_pose(b.lookup_many([("root", "handle")], now_ns=NOW)[0]).position[0] == 1


@pytest.mark.parametrize("field_first", [True, False])
def test_static_and_dynamic_source_conflict_preserves_first_source(field_first):
    b = buffer()
    child = "station/upper/base_frame"
    static_entry = tf_entry("world", child, (1, 0, 0), timestamp_ns=0)
    dynamic_entry = tf_entry("station/reference_frame", child, (0, 0, 1), timestamp_ns=NOW)
    first, second = (static_entry, dynamic_entry) if field_first else (dynamic_entry, static_entry)
    b.update({"transforms": [first]}, static=field_first)
    with pytest.raises(ContractError) as exc:
        b.update({"transforms": [second]}, static=not field_first)
    message = str(exc.value)
    assert "world (/tf_static)" in message
    assert "station/reference_frame (/tf)" in message
    assert "DOMAIN_ID" in message
    parent = first["header"]["frame_id"]
    actual = to_pose(b.lookup_many([(parent, child)], now_ns=NOW)[0])
    assert actual.position == ((1, 0, 0) if field_first else (0, 0, 1))
