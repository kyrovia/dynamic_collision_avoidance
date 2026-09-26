"""Link capsules: obstacle repulsion and non-adjacent self-collision.

The push follows CHOMP's collision potential. Its slope grows from 0 at the
influence distance to 1 at contact, and stays at 1 inside a collision.
Adjacent links are ignored, matching MoveIt's default allowed-collision pairs.
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from robot_avoidance_apf.apf import SphereObstacle
from robot_avoidance_apf.kinematics import BodySegment, JointAxis

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class CapsuleDraw:
    """Cylinder that matches one link capsule. RViz cylinders are along Z."""

    name: str
    start: Vec3
    end: Vec3
    center: Vec3
    orientation: tuple[float, float, float, float]
    length: float
    radius: float


@dataclass(frozen=True)
class AvoidanceCommand:
    """Joint velocity from both repulsive fields, and the tightest gaps."""

    velocity: tuple[float, ...]
    hold: bool
    obstacle_clearance: float
    self_clearance: float


def capsule_draws(
    segments: Sequence[BodySegment], radii: Sequence[float]
) -> tuple[CapsuleDraw, ...]:
    """One drawable capsule per link, in the same frame as the segment ends."""
    if len(radii) != len(segments):
        raise ValueError(f"expected {len(segments)} radii, got {len(radii)}")
    drawn: list[CapsuleDraw] = []
    for segment, radius in zip(segments, radii):
        start = _array(segment.start)
        end = _array(segment.end)
        offset = end - start
        length = float(np.linalg.norm(offset))
        center = 0.5 * (start + end)
        drawn.append(
            CapsuleDraw(
                segment.name,
                segment.start,
                segment.end,
                (float(center[0]), float(center[1]), float(center[2])),
                _z_to(offset),
                length,
                radius,
            )
        )
    return tuple(drawn)


def link_radius(name: str, arm: float, wrist: float, base: float) -> float:
    """Tube radius from the link name. Wrist and base are thinner and thicker."""
    lowered = name.lower()
    if "wrist" in lowered:
        return wrist
    if "base" in lowered:
        return base
    return arm


def avoidance_velocity(
    segments: Sequence[BodySegment],
    axes: Sequence[JointAxis],
    obstacle: SphereObstacle,
    radii: Sequence[float],
    *,
    k_rep: float,
    d0: float,
    d_min: float,
    k_self: float,
    self_d0: float,
    self_d_min: float,
    qdot_max: float,
) -> AvoidanceCommand:
    """Push every moving link off the obstacle, and separate non-adjacent links.

    Adjacent links share a joint, so they are allowed to touch, matching
    MoveIt's default allowed-collision pairs. A fixed base does not move;
    it still keeps the arm off the obstacle.
    """
    if len(radii) != len(segments):
        raise ValueError(f"expected {len(segments)} radii, got {len(radii)}")
    count = len(axes)
    velocity = np.zeros(count)
    obstacle_clearance = math.inf
    for segment, radius in zip(segments, radii):
        gap, point, normal = _sphere_gap(segment, radius, obstacle)
        if segment.joint_count == 0:
            continue
        obstacle_clearance = min(obstacle_clearance, gap)
        gain = k_rep * _potential_slope(gap, d0)
        if gain == 0.0:
            continue
        jacobian = _point_jacobian(point, segment.joint_count, axes)
        velocity = velocity + jacobian.T @ (normal * gain)

    self_clearance = math.inf
    for earlier, later in _nonadjacent_pairs(len(segments)):
        gap, point_a, point_b = _capsule_gap(
            segments[earlier],
            radii[earlier],
            segments[later],
            radii[later],
        )
        self_clearance = min(self_clearance, gap)
        gain = k_self * _potential_slope(gap, self_d0)
        if gain == 0.0:
            continue
        normal = _separation_normal(
            point_a, point_b, segments[earlier], segments[later]
        )
        push = normal * gain
        jacobian_a = _point_jacobian(point_a, segments[earlier].joint_count, axes)
        jacobian_b = _point_jacobian(point_b, segments[later].joint_count, axes)
        velocity = velocity + jacobian_a.T @ push - jacobian_b.T @ push

    hold = obstacle_clearance < d_min or self_clearance < self_d_min
    if hold or qdot_max <= 0.0:
        velocity = np.zeros(count)
    else:
        velocity = np.clip(velocity, -qdot_max, qdot_max)
    return AvoidanceCommand(
        tuple(float(item) for item in velocity),
        hold,
        obstacle_clearance,
        self_clearance,
    )


def _nonadjacent_pairs(count: int) -> list[tuple[int, int]]:
    return [
        (earlier, later)
        for earlier in range(count)
        for later in range(earlier + 2, count)
    ]


def _sphere_gap(
    segment: BodySegment, radius: float, obstacle: SphereObstacle
) -> tuple[float, np.ndarray, np.ndarray]:
    start = _array(segment.start)
    end = _array(segment.end)
    center = _array(obstacle.center)
    point = _closest_on_segment(start, end, center)
    offset = point - center
    distance = float(np.linalg.norm(offset))
    gap = distance - obstacle.radius - radius
    if distance < 1e-9:
        normal = _perpendicular(end - start)
    else:
        normal = offset / distance
    return gap, point, normal


def _capsule_gap(
    earlier: BodySegment,
    earlier_radius: float,
    later: BodySegment,
    later_radius: float,
) -> tuple[float, np.ndarray, np.ndarray]:
    point_a, point_b = _closest_segments(
        _array(earlier.start),
        _array(earlier.end),
        _array(later.start),
        _array(later.end),
    )
    gap = float(np.linalg.norm(point_a - point_b)) - earlier_radius - later_radius
    return gap, point_a, point_b


def _separation_normal(
    point_a: np.ndarray,
    point_b: np.ndarray,
    earlier: BodySegment,
    later: BodySegment,
) -> np.ndarray:
    offset = point_a - point_b
    distance = float(np.linalg.norm(offset))
    if distance >= 1e-9:
        return offset / distance
    direction = _array(later.end) - _array(later.start)
    if float(np.linalg.norm(direction)) < 1e-9:
        direction = _array(earlier.end) - _array(earlier.start)
    return _perpendicular(direction)


def _potential_slope(gap: float, influence: float) -> float:
    """Minus the derivative of CHOMP's collision potential, in [0, 1]."""
    if influence <= 0.0 or gap >= influence:
        return 0.0
    if gap >= 0.0:
        return (influence - gap) / influence
    return 1.0


def _point_jacobian(
    point: np.ndarray, joint_count: int, axes: Sequence[JointAxis]
) -> np.ndarray:
    jacobian = np.zeros((3, len(axes)))
    for index in range(joint_count):
        origin = _array(axes[index].origin)
        direction = _array(axes[index].direction)
        jacobian[:, index] = np.cross(direction, point - origin)
    return jacobian


def _closest_on_segment(
    start: np.ndarray, end: np.ndarray, point: np.ndarray
) -> np.ndarray:
    direction = end - start
    length2 = float(direction @ direction)
    if length2 <= 1e-12:
        return start.copy()
    scale = float((point - start) @ direction) / length2
    return start + direction * min(1.0, max(0.0, scale))


def _closest_segments(
    start_a: np.ndarray,
    end_a: np.ndarray,
    start_b: np.ndarray,
    end_b: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Closest points on two segments. See Ericson, Real-Time Collision Detection."""
    direction_a = end_a - start_a
    direction_b = end_b - start_b
    offset = start_a - start_b
    aa = float(direction_a @ direction_a)
    ee = float(direction_b @ direction_b)
    ff = float(direction_b @ offset)
    if aa <= 1e-12 and ee <= 1e-12:
        return start_a.copy(), start_b.copy()
    if aa <= 1e-12:
        scale_a = 0.0
        scale_b = _clip01(ff / ee)
    elif ee <= 1e-12:
        scale_b = 0.0
        scale_a = _clip01(-float(direction_a @ offset) / aa)
    else:
        cc = float(direction_a @ offset)
        bb = float(direction_a @ direction_b)
        denom = aa * ee - bb * bb
        if denom > 1e-12:
            scale_a = _clip01((bb * ff - cc * ee) / denom)
        else:
            scale_a = 0.0
        scale_b = (bb * scale_a + ff) / ee
        if scale_b < 0.0:
            scale_b = 0.0
            scale_a = _clip01(-cc / aa)
        elif scale_b > 1.0:
            scale_b = 1.0
            scale_a = _clip01((bb - cc) / aa)
    return start_a + direction_a * scale_a, start_b + direction_b * scale_b


def _perpendicular(direction: np.ndarray) -> np.ndarray:
    if float(np.linalg.norm(direction)) < 1e-9:
        return np.array([0.0, 0.0, 1.0])
    axis = np.array([0.0, 0.0, 1.0])
    side = np.cross(direction, axis)
    if float(np.linalg.norm(side)) < 1e-6:
        side = np.cross(direction, np.array([1.0, 0.0, 0.0]))
    return side / np.linalg.norm(side)


def _clip01(value: float) -> float:
    return min(1.0, max(0.0, value))


def _z_to(direction: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation that takes the marker Z axis onto this segment."""
    length = float(np.linalg.norm(direction))
    if length < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    target = direction / length
    z_axis = np.array([0.0, 0.0, 1.0])
    alignment = float(z_axis @ target)
    if alignment > 1.0 - 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    if alignment < -1.0 + 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    axis = np.cross(z_axis, target)
    axis = axis / np.linalg.norm(axis)
    angle = math.acos(max(-1.0, min(1.0, alignment)))
    half = 0.5 * angle
    sine = math.sin(half)
    return (
        float(axis[0] * sine),
        float(axis[1] * sine),
        float(axis[2] * sine),
        math.cos(half),
    )


def _array(point: Vec3) -> np.ndarray:
    return np.array(point, dtype=float)
