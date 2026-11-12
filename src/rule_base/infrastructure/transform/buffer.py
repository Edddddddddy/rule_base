"""Bounded TF tree with static/dynamic separation and lookup-time age gates."""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from dataclasses import dataclass
from threading import RLock
from typing import Any

from rule_base.domain.models import ContractError, Matrix, Pose

from .math import IDENTITY, compose, from_pose, inverse


def field(value: Any, name: str) -> Any:
    try:
        return value[name] if isinstance(value, Mapping) else getattr(value, name)
    except (KeyError, AttributeError) as exc:
        raise ContractError(f"message missing {name}") from exc


def payload(message: Any) -> Any:
    # CORA receive envelope holds a typed message in .data; mappings are fixtures.
    return message if isinstance(message, Mapping) else getattr(message, "data", message)


def stamp_ns(header: Any) -> int:
    stamp = field(header, "stamp")
    sec, nano = field(stamp, "sec"), field(stamp, "nanosec")
    if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 10**9:
        raise ContractError("invalid TF timestamp")
    return int(sec * 10**9 + nano)


def number(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ContractError("expected a finite number")
    return float(value)


@dataclass(frozen=True)
class Edge:
    parent: str
    child: str
    transform: Matrix
    timestamp_ns: int | None


class TransformBuffer:
    def __init__(
        self,
        *,
        max_age_s: float,
        max_skew_s: float,
        capacity: int = 512,
        future_tolerance_s: float = 0,
    ) -> None:
        self.max_age_ns = int(max_age_s * 10**9)
        self.max_skew_ns = int(max_skew_s * 10**9)
        self.future_tolerance_ns = int(future_tolerance_s * 10**9)
        self.capacity = capacity
        self._edges: dict[str, Edge] = {}
        self._lock = RLock()
        self._dynamic_not_before_ns = 0

    def update(self, message: Any, *, static: bool = False) -> None:
        batch: dict[str, Edge] = {}
        for item in field(payload(message), "transforms"):
            header = field(item, "header")
            parent, child = field(header, "frame_id"), field(item, "child_frame_id")
            if (
                any(
                    not isinstance(n, str) or not n or any(c.isspace() for c in n)
                    for n in (parent, child)
                )
                or parent == child
            ):
                raise ContractError("invalid TF frame pair")
            t = field(item, "transform")
            p, q = field(t, "translation"), field(t, "rotation")
            transform = from_pose(
                Pose(
                    (number(field(p, "x")), number(field(p, "y")), number(field(p, "z"))),
                    (
                        number(field(q, "x")),
                        number(field(q, "y")),
                        number(field(q, "z")),
                        number(field(q, "w")),
                    ),
                )
            )
            timestamp = None if static else stamp_ns(header)
            if timestamp == 0:
                raise ContractError("dynamic TF stamp must be nonzero wall-clock time")
            if child in batch:
                raise ContractError(f"duplicate TF child {child}")
            batch[child] = Edge(parent, child, transform, timestamp)
        with self._lock:
            merged = dict(self._edges)
            for child, edge in batch.items():
                if (
                    edge.timestamp_ns is not None
                    and edge.timestamp_ns < self._dynamic_not_before_ns
                ):
                    continue
                previous = merged.get(child)
                if previous:
                    if previous.parent != edge.parent or (previous.timestamp_ns is None) != static:
                        previous_source = "/tf_static" if previous.timestamp_ns is None else "/tf"
                        source = "/tf_static" if static else "/tf"
                        raise ContractError(
                            f"conflicting TF parent/source for {child}: "
                            f"{previous.parent} ({previous_source}) vs {edge.parent} ({source}); "
                            "isolate mock and field publishers using different DOMAIN_ID values"
                        )
                    if static and previous.transform != edge.transform:
                        raise ContractError(
                            f"static calibration changed for {child}; restart required"
                        )
                    if (
                        not static
                        and previous.timestamp_ns is not None
                        and edge.timestamp_ns is not None
                    ):
                        if edge.timestamp_ns < previous.timestamp_ns:
                            continue  # old transport packet must not replace a newer sample
                        if (
                            edge.timestamp_ns == previous.timestamp_ns
                            and edge.transform != previous.transform
                        ):
                            raise ContractError(
                                f"conflicting TF sample at identical timestamp: {child}"
                            )
                merged[child] = edge
            if len(merged) > self.capacity:
                raise ContractError("TF capacity exceeded")
            for child in merged:
                seen: set[str] = set()
                current = child
                while current in merged:
                    if current in seen:
                        raise ContractError("TF cycle detected")
                    seen.add(current)
                    current = merged[current].parent
            self._edges = merged

    def parent_of(self, child: str) -> str | None:
        with self._lock:
            edge = self._edges.get(child)
            return None if edge is None else edge.parent

    def clear_dynamic(self) -> None:
        """Keep calibration; discard old dynamics, including delayed pre-reset samples."""
        with self._lock:
            self._dynamic_not_before_ns = time.time_ns()
            self._edges = {
                child: edge for child, edge in self._edges.items() if edge.timestamp_ns is None
            }

    def lookup_many(
        self,
        pairs: list[tuple[str, str]],
        *,
        now_ns: int | None = None,
        dynamic_pair: tuple[str, str] | None = None,
    ) -> list[Matrix]:
        with self._lock:
            # Read the clock after locking, so a concurrent update cannot insert
            # a sample newer than a clock value taken before lock acquisition.
            if now_ns is None:
                now_ns = time.time_ns()
            results = []
            all_stamps: list[int] = []
            dynamic_edges: dict[str, Edge] = {}
            for target, source in pairs:
                ancestors = {target}
                current = target
                while current in self._edges:
                    current = self._edges[current].parent
                    ancestors.add(current)

                source_edges: list[Edge] = []
                current = source
                while current not in ancestors and current in self._edges:
                    edge = self._edges[current]
                    source_edges.append(edge)
                    current = edge.parent
                if current not in ancestors:
                    raise ContractError(f"TF path unavailable: {target} <- {source}")
                common = current

                transform = IDENTITY
                stamps: list[int] = []
                current = target
                while current != common:
                    edge = self._edges[current]
                    transform = compose(transform, inverse(edge.transform))
                    if edge.timestamp_ns is not None:
                        stamps.append(edge.timestamp_ns)
                        dynamic_edges[edge.child] = edge
                    current = edge.parent
                for edge in reversed(source_edges):
                    transform = compose(transform, edge.transform)
                    if edge.timestamp_ns is not None:
                        stamps.append(edge.timestamp_ns)
                        dynamic_edges[edge.child] = edge
                if (target, source) == dynamic_pair and not stamps:
                    raise ContractError(f"TF path has no dynamic sample: {target} <- {source}")
                results.append(transform)
                all_stamps.extend(stamps)
            for edge in dynamic_edges.values():
                assert edge.timestamp_ns is not None
                age_ns = now_ns - edge.timestamp_ns
                if age_ns < -self.future_tolerance_ns or age_ns > self.max_age_ns:
                    reason = "future" if age_ns < 0 else "stale"
                    raise ContractError(
                        "TF snapshot is stale or future-dated: "
                        f"reason={reason}; edge={edge.parent} -> {edge.child}; "
                        f"age_s={age_ns / 1e9:.6f}; "
                        f"max_age_s={self.max_age_ns / 1e9:.6f}; "
                        f"future_tolerance_s={self.future_tolerance_ns / 1e9:.6f}"
                    )
            if all_stamps and max(all_stamps) - min(all_stamps) > self.max_skew_ns:
                raise ContractError("TF snapshot exceeds inter-edge skew")
            return results
