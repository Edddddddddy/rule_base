"""Rigid transform math; matrices consistently mean T_target_source."""

from __future__ import annotations

import math
from typing import cast

import numpy as np

from rule_base.domain.models import ContractError, Matrix, Pose, Quaternion, Vector3


def matrix(value: object) -> Matrix:
    a = np.asarray(value, dtype=float)
    if a.shape != (4, 4) or not np.isfinite(a).all():
        raise ContractError("transform must be a finite 4x4 matrix")
    r = a[:3, :3]
    if not np.allclose(a[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
        raise ContractError("invalid homogeneous row")
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0) or not math.isclose(
        float(np.linalg.det(r)), 1, abs_tol=1e-6
    ):
        raise ContractError("rotation must be orthonormal and right handed")
    return tuple(tuple(float(x) for x in row) for row in a)


IDENTITY = matrix(np.eye(4))


def from_pose(pose: Pose) -> Matrix:
    q = np.asarray(pose.orientation, dtype=float)
    if (
        q.shape != (4,)
        or not np.isfinite(q).all()
        or not math.isclose(float(np.linalg.norm(q)), 1, abs_tol=1e-3)
    ):
        raise ContractError("orientation must be a unit xyzw quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    a = np.eye(4)
    a[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    a[:3, 3] = pose.position
    return matrix(a)


def to_pose(value: Matrix) -> Pose:
    a = np.asarray(matrix(value))
    r = a[:3, :3]
    # Symmetric eigenproblem avoids instability near a 180-degree rotation.
    k = (
        np.array(
            [
                [
                    r[0, 0] - r[1, 1] - r[2, 2],
                    r[1, 0] + r[0, 1],
                    r[2, 0] + r[0, 2],
                    r[2, 1] - r[1, 2],
                ],
                [
                    r[1, 0] + r[0, 1],
                    r[1, 1] - r[0, 0] - r[2, 2],
                    r[2, 1] + r[1, 2],
                    r[0, 2] - r[2, 0],
                ],
                [
                    r[2, 0] + r[0, 2],
                    r[2, 1] + r[1, 2],
                    r[2, 2] - r[0, 0] - r[1, 1],
                    r[1, 0] - r[0, 1],
                ],
                [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1], r.trace()],
            ]
        )
        / 3
    )
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    if q[3] < 0:
        q = -q
    return Pose(cast(Vector3, tuple(a[:3, 3])), cast(Quaternion, tuple(q)))


def compose(*values: Matrix) -> Matrix:
    result = np.eye(4)
    for value in values:
        result = result @ np.asarray(value)
    return matrix(result)


def inverse(value: Matrix) -> Matrix:
    a = np.asarray(matrix(value))
    result = np.eye(4)
    result[:3, :3] = a[:3, :3].T
    result[:3, 3] = -a[:3, :3].T @ a[:3, 3]
    return matrix(result)


def rotate(value: Matrix, vector: Vector3) -> Vector3:
    return cast(Vector3, tuple(np.asarray(value)[:3, :3] @ vector))


def offset(pose: Pose, vector: Vector3, scale: float = 1) -> Pose:
    return Pose(
        cast(Vector3, tuple(a + scale * b for a, b in zip(pose.position, vector, strict=True))),
        pose.orientation,
    )


def split_line(start: Vector3, end: Vector3, step_m: float) -> list[Vector3]:
    """Sample a line in one frame at fixed spacing (m), excluding start, including end.

    The last segment may be shorter. Coincident endpoints return only end.
    This generates positions, not a timed trajectory or orientation interpolation.
    """
    if not math.isfinite(step_m) or step_m <= 0:
        raise ValueError("step_m must be finite and positive")
    if len(start) != 3 or len(end) != 3 or not all(map(math.isfinite, (*start, *end))):
        raise ValueError("line endpoints must be finite 3D positions")
    distance = math.dist(start, end)
    points = []
    for i in range(1, math.ceil(distance / step_m)):
        if math.isclose(i * step_m, distance, rel_tol=1e-12):
            break
        fraction = i * step_m / distance
        points.append(
            cast(Vector3, tuple(a + (b - a) * fraction for a, b in zip(start, end, strict=True)))
        )
    return [*points, end]


def errors(actual: Pose, target: Pose) -> tuple[float, float]:
    distance = math.dist(actual.position, target.position)
    a, b = np.asarray(actual.orientation), np.asarray(target.orientation)
    dot = abs(float(a @ b)) / float(np.linalg.norm(a) * np.linalg.norm(b))
    return distance, 2 * math.acos(min(1, max(0, dot)))
