"""Cartesian artificial potential field for the tool.

Link capsules own the normal push away from the sphere. The tool keeps
attraction, orientation, and a tangential term so it can slide around
the sphere on the way to the goal.
"""

import math
from dataclasses import dataclass

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


@dataclass(frozen=True)
class SphereObstacle:
    center: Vec3
    radius: float


@dataclass(frozen=True)
class ApfParams:
    """Runtime tuning values; defaults live in config/apf_params.yaml."""

    k_att: float
    k_rep: float
    k_tan: float
    k_ori: float
    d0: float
    d_min: float
    v_max: float
    omega_max: float


@dataclass(frozen=True)
class FieldCommand:
    """Cartesian velocity for tool0. hold means keep the current joints."""

    linear: Vec3
    angular: Vec3
    hold: bool
    clearance: float


def orientation_angle(current: Quat, goal: Quat) -> float:
    """Magnitude of the rotation that takes current onto goal, in radians."""
    error = _orientation_error(current, goal)
    return _norm(error)


def field_command(
    position: Vec3,
    orientation: Quat,
    goal_position: Vec3,
    goal_orientation: Quat,
    obstacle: SphereObstacle,
    params: ApfParams,
) -> FieldCommand:
    """Attract toward the goal and steer around a sphere that blocks the line."""
    gap = _surface_clearance(position, obstacle)
    if gap < params.d_min:
        return FieldCommand((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), True, gap)

    attraction = _clip(_scale(_sub(goal_position, position), params.k_att), params.v_max)
    repulsion = _repulsion(position, goal_position, obstacle, params)
    linear = _clip(_add(attraction, repulsion), params.v_max)

    orientation_scale = 1.0 if gap >= params.d0 else gap / params.d0
    rotation = _orientation_error(orientation, goal_orientation)
    angular = _clip(_scale(rotation, params.k_ori * orientation_scale), params.omega_max)
    return FieldCommand(linear, angular, False, gap)


def _surface_clearance(position: Vec3, obstacle: SphereObstacle) -> float:
    return _norm(_sub(position, obstacle.center)) - obstacle.radius


def _repulsion(
    position: Vec3,
    goal_position: Vec3,
    obstacle: SphereObstacle,
    params: ApfParams,
) -> Vec3:
    offset = _sub(position, obstacle.center)
    distance = _norm(offset)
    gap = distance - obstacle.radius
    if distance < 1e-9 or gap >= params.d0:
        return (0.0, 0.0, 0.0)

    # Clearance is at least d_min here, so the singularity at the surface is outside this branch.
    normal = _scale(offset, 1.0 / distance)
    strength = (1.0 / gap - 1.0 / params.d0) / (gap * gap)

    toward_goal = _sub(goal_position, position)
    tangent = _reject(toward_goal, normal)
    if _norm(tangent) < 1e-6:
        # The obstacle sits on the line to the goal, so attraction is straight into it.
        tangent = _perpendicular_unit(normal)
    else:
        tangent = _unit(tangent)
    tangential = _scale(tangent, params.k_tan * strength)

    # The scene goal lies inside the influence radius. Scaling by goal distance
    # keeps that point an equilibrium instead of a permanent slide around the sphere.
    return _scale(tangential, _norm(toward_goal))


def _orientation_error(current: Quat, goal: Quat) -> Vec3:
    """Rotation vector, in the base frame, that takes current onto goal."""
    error = _quat_multiply(goal, _quat_conjugate(current))
    if error[3] < 0.0:
        error = (-error[0], -error[1], -error[2], -error[3])
    xyz = (error[0], error[1], error[2])
    vector_norm = _norm(xyz)
    if vector_norm < 1e-9:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_norm, error[3])
    return _scale(xyz, angle / vector_norm)


def _quat_multiply(left: Quat, right: Quat) -> Quat:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _quat_conjugate(quat: Quat) -> Quat:
    return (-quat[0], -quat[1], -quat[2], quat[3])


def _perpendicular_unit(axis: Vec3) -> Vec3:
    perp = _cross(axis, (0.0, 0.0, 1.0))
    if _norm(perp) < 1e-6:
        perp = _cross(axis, (1.0, 0.0, 0.0))
    return _unit(perp)


def _reject(vector: Vec3, normal: Vec3) -> Vec3:
    return _sub(vector, _scale(normal, _dot(vector, normal)))


def _clip(vector: Vec3, limit: float) -> Vec3:
    length = _norm(vector)
    if length <= limit or length < 1e-12:
        return vector
    return _scale(vector, limit / length)


def _unit(vector: Vec3) -> Vec3:
    return _scale(vector, 1.0 / _norm(vector))


def _add(left: Vec3, right: Vec3) -> Vec3:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def _sub(left: Vec3, right: Vec3) -> Vec3:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def _scale(vector: Vec3, factor: float) -> Vec3:
    return (vector[0] * factor, vector[1] * factor, vector[2] * factor)


def _dot(left: Vec3, right: Vec3) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _cross(left: Vec3, right: Vec3) -> Vec3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _norm(vector: Vec3) -> float:
    return math.sqrt(_dot(vector, vector))
