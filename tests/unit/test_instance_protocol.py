"""Purpose: unit-test the pure 026 instance protocol (env parsing, edge gate, exit codes).

Usage: python3 -m pytest tests/unit/test_instance_protocol.py -q
"""

import json

import pytest

from rule_base.instance import protocol
from rule_base.instance.protocol import (
    CHILD_FAILED,
    CHILD_INTERRUPTED,
    CHILD_OK,
    CHILD_SPAWN_ERROR,
    CHILD_TIMEOUT,
    EXIT_CONFIG,
    EXIT_OK,
    EXIT_RUNTIME,
    InstanceConfigError,
    InstanceProtocol,
    event_line,
    exit_code_for_round,
    namespace_topic,
    parse_instance_env,
    validate_namespace,
)

BASE_ENV = {
    "INFERENCE_NAMESPACE": "",
    "DOMAIN_ID": "229",
    "STATION_TYPE": "pick_place",
    "CLIENT_ID": "client-1",
    "LEASE_ID": "7",
    "INFERENCE_ID": "3",
}


def test_exit_codes_are_frozen():
    assert (EXIT_OK, EXIT_RUNTIME, EXIT_CONFIG) == (0, 1, 2)


# --- namespace / topics ----------------------------------------------------
@pytest.mark.parametrize(
    ("namespace", "suffix", "expected"),
    [
        ("", "inference/stopped", "/inference/stopped"),
        ("", "env/reset_status", "/env/reset_status"),
        ("/station1", "inference/stopped", "/station1/inference/stopped"),
        ("/station1", "env/reset_status", "/station1/env/reset_status"),
        ("/a/b", "env/reset_status", "/a/b/env/reset_status"),
    ],
)
def test_namespace_topic_resolution(namespace, suffix, expected):
    assert namespace_topic(namespace, suffix) == expected


def test_namespace_topic_has_no_double_slash_for_trailing_suffix_slash():
    assert namespace_topic("/station1", "/inference/stopped") == "/station1/inference/stopped"
    assert namespace_topic("", "/inference/stopped") == "/inference/stopped"


@pytest.mark.parametrize("valid", ["", "/station1", "/a/b_c"])
def test_validate_namespace_accepts_empty_or_absolute(valid):
    validate_namespace(valid)  # must not raise


@pytest.mark.parametrize(
    "invalid",
    ["station1", "a/b", "/", "/a/", "/a//b", "/a b", "  "],
)
def test_validate_namespace_rejects_invalid(invalid):
    with pytest.raises(InstanceConfigError):
        validate_namespace(invalid)


# --- environment parsing ---------------------------------------------------
def test_parse_env_default_namespace():
    env = parse_instance_env(BASE_ENV)
    assert env.namespace == ""
    assert env.domain_id == 229
    assert env.station_type == "pick_place"
    assert env.client_id == "client-1"
    assert env.lease_id == "7"
    assert env.inference_id == 3
    assert env.warnings == ()
    assert env.stopped_topic == "/inference/stopped"
    assert env.reset_status_topic == "/env/reset_status"


def test_parse_env_namespaced():
    env = parse_instance_env({**BASE_ENV, "INFERENCE_NAMESPACE": "/station1"})
    assert env.namespace == "/station1"
    assert env.stopped_topic == "/station1/inference/stopped"
    assert env.reset_status_topic == "/station1/env/reset_status"


def test_parse_env_allows_missing_ros_domain_id():
    # The entry derives ROS_DOMAIN_ID from DOMAIN_ID before rclpy.init.
    assert "ROS_DOMAIN_ID" not in BASE_ENV
    assert parse_instance_env(BASE_ENV).domain_id == 229


def test_parse_env_rejects_ros_domain_id_mismatch():
    with pytest.raises(InstanceConfigError, match="does not match ROS_DOMAIN_ID"):
        parse_instance_env({**BASE_ENV, "ROS_DOMAIN_ID": "193"})


def test_parse_env_accepts_matching_ros_domain_id():
    assert parse_instance_env({**BASE_ENV, "ROS_DOMAIN_ID": "229"}).domain_id == 229


def test_parse_env_requires_domain_id():
    env = dict(BASE_ENV)
    env.pop("DOMAIN_ID")
    with pytest.raises(InstanceConfigError, match="DOMAIN_ID is required"):
        parse_instance_env(env)


def test_parse_env_rejects_non_integer_domain_id():
    with pytest.raises(InstanceConfigError, match="not an integer"):
        parse_instance_env({**BASE_ENV, "DOMAIN_ID": "abc"})


def test_parse_env_rejects_negative_domain_id():
    with pytest.raises(InstanceConfigError, match="must not be negative"):
        parse_instance_env({**BASE_ENV, "DOMAIN_ID": "-1"})


def test_parse_env_requires_station_type():
    env = dict(BASE_ENV)
    env.pop("STATION_TYPE")
    with pytest.raises(InstanceConfigError, match="STATION_TYPE is required"):
        parse_instance_env(env)


def test_parse_env_rejects_unsupported_station_type():
    with pytest.raises(InstanceConfigError, match="not supported"):
        parse_instance_env({**BASE_ENV, "STATION_TYPE": "drawer"})


def test_parse_env_rejects_relative_namespace():
    with pytest.raises(InstanceConfigError, match="INFERENCE_NAMESPACE"):
        parse_instance_env({**BASE_ENV, "INFERENCE_NAMESPACE": "station1"})


def test_parse_env_empty_client_and_lease_are_warnings_only():
    env = parse_instance_env(
        {**BASE_ENV, "CLIENT_ID": "", "LEASE_ID": "  ", "INFERENCE_ID": ""}
    )
    assert env.client_id == ""
    assert env.lease_id == ""
    assert env.inference_id is None
    joined = "\n".join(env.warnings)
    assert "CLIENT_ID" in joined
    assert "LEASE_ID" in joined


def test_parse_env_missing_namespace_is_warning_only():
    env = dict(BASE_ENV)
    env.pop("INFERENCE_NAMESPACE")
    parsed = parse_instance_env(env)
    assert parsed.namespace == ""
    assert any("INFERENCE_NAMESPACE" in warning for warning in parsed.warnings)


def test_parse_env_invalid_inference_id_is_warning_only():
    parsed = parse_instance_env({**BASE_ENV, "INFERENCE_ID": "not-a-number"})
    assert parsed.inference_id is None
    assert any("INFERENCE_ID" in warning for warning in parsed.warnings)


# --- False -> True edge gate ----------------------------------------------
def ready_gate() -> InstanceProtocol:
    gate = InstanceProtocol()
    gate.mark_ready()
    return gate


def test_warming_ignores_reset_levels():
    gate = InstanceProtocol()
    assert gate.on_reset_status(False) is False
    assert gate.on_reset_status(True) is False
    assert gate.state == protocol.STATE_WARMING
    assert gate.armed is False

    gate.mark_ready()
    assert gate.state == protocol.STATE_IDLE


def test_true_without_false_is_not_an_edge():
    gate = ready_gate()
    assert gate.on_reset_status(True) is False
    assert gate.state == protocol.STATE_IDLE
    assert gate.armed is False


def test_false_arms_and_true_starts_exactly_one_round():
    gate = ready_gate()
    assert gate.on_reset_status(False) is False
    assert gate.armed is True
    assert gate.on_reset_status(True) is True
    assert gate.state == protocol.STATE_RUNNING
    assert gate.armed is False
    assert gate.rounds_started == 1
    assert gate.stopped_value is False


def test_repeated_true_while_running_is_dropped_not_queued():
    gate = ready_gate()
    gate.on_reset_status(False)
    gate.on_reset_status(True)
    for _ in range(5):
        assert gate.on_reset_status(True) is False
        assert gate.on_reset_status(False) is False
    assert gate.discarded_resets == 10
    assert gate.rounds_started == 1
    assert gate.state == protocol.STATE_RUNNING


def test_continuous_true_after_round_does_not_retrigger():
    gate = ready_gate()
    gate.on_reset_status(False)
    assert gate.on_reset_status(True) is True
    gate.on_round_finished()
    assert gate.state == protocol.STATE_IDLE
    assert gate.armed is False
    assert gate.stopped_value is True
    for _ in range(10):
        assert gate.on_reset_status(True) is False
    assert gate.rounds_started == 1
    assert gate.rounds_completed == 1


def test_round_after_round_requires_fresh_false():
    gate = ready_gate()
    gate.on_reset_status(False)
    assert gate.on_reset_status(True) is True
    gate.on_round_finished()
    # A fresh False->True edge is a new round even while the level stays True in between.
    assert gate.on_reset_status(False) is False
    assert gate.on_reset_status(True) is True
    assert gate.rounds_started == 2
    gate.on_round_finished()
    assert gate.rounds_completed == 2


def test_failed_round_is_counted_and_disarms():
    gate = ready_gate()
    gate.on_reset_status(False)
    gate.on_reset_status(True)
    gate.on_round_finished(failed=True)
    assert gate.rounds_failed == 1
    assert gate.armed is False
    assert gate.on_reset_status(True) is False


def test_round_finished_without_running_round_is_a_bug():
    gate = ready_gate()
    with pytest.raises(RuntimeError, match="not running"):
        gate.on_round_finished()


# --- exit code classification ---------------------------------------------
@pytest.mark.parametrize(
    ("status", "expected"),
    [
        (CHILD_OK, None),
        (CHILD_INTERRUPTED, EXIT_OK),
        (CHILD_FAILED, EXIT_RUNTIME),
        (CHILD_TIMEOUT, EXIT_RUNTIME),
        (CHILD_SPAWN_ERROR, EXIT_RUNTIME),
    ],
)
def test_exit_code_for_round(status, expected):
    assert exit_code_for_round(status) == expected


# --- structured log line ---------------------------------------------------
def test_event_line_is_single_json_object():
    line = event_line("instance_failed", stage="round", inference_id=3)
    assert "\n" not in line
    payload = json.loads(line)
    assert payload == {"event": "instance_failed", "stage": "round", "inference_id": 3}
    assert line.startswith('{"event": "instance_failed"')
