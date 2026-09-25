"""Forward kinematics and damped joint velocities from a URDF chain."""

import math
import xml.etree.ElementTree as ET
from collections.abc import Sequence

import numpy as np
import PyKDL

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


class ArmKinematics:
    """tool0 pose, Jacobian, and one damped integration step."""

    def __init__(self, urdf: str, base_link: str, tip_link: str) -> None:
        # Solvers keep a raw pointer to the chain, so the chain has to stay alive.
        self._chain, names, limits = chain_from_urdf(urdf, base_link, tip_link)
        self.joint_names = names
        self.limits = limits
        self._fk = PyKDL.ChainFkSolverPos_recursive(self._chain)
        self._jacobian = PyKDL.ChainJntToJacSolver(self._chain)

    def pose(self, positions: Sequence[float]) -> tuple[Vec3, Quat]:
        frame = PyKDL.Frame()
        if self._fk.JntToCart(self._joints(positions), frame) < 0:
            raise RuntimeError("forward kinematics failed")
        position = (frame.p.x(), frame.p.y(), frame.p.z())
        return position, frame.M.GetQuaternion()

    def integrate(
        self,
        positions: Sequence[float],
        twist: Sequence[float],
        dt: float,
        damping: float,
        k_lim: float = 0.0,
        rho_lim: float = 0.3,
        qdot_lim: float = 0.5,
    ) -> tuple[list[float], list[float]]:
        jacobian = self._numeric_jacobian(positions)
        velocity = damped_least_squares(jacobian, np.asarray(twist, dtype=float), damping)
        # A 6-DoF arm has no null space when J is full rank, so the limit
        # field is added in joint space. The clamp below still cannot be crossed.
        velocity = velocity + joint_limit_velocity(
            positions, self.limits, k_lim, rho_lim, qdot_lim
        )
        stepped: list[float] = []
        effective: list[float] = []
        for index, position in enumerate(positions):
            lower, upper = self.limits[index]
            next_position = min(
                upper, max(lower, position + float(velocity[index]) * dt)
            )
            stepped.append(next_position)
            effective.append((next_position - float(position)) / dt)
        return stepped, effective

    def _numeric_jacobian(self, positions: Sequence[float]) -> np.ndarray:
        count = len(self.joint_names)
        jacobian = PyKDL.Jacobian(count)
        if self._jacobian.JntToJac(self._joints(positions), jacobian) < 0:
            raise RuntimeError("Jacobian evaluation failed")
        values = np.zeros((6, count))
        for row in range(6):
            for column in range(count):
                values[row, column] = jacobian[row, column]
        return values

    def _joints(self, positions: Sequence[float]) -> PyKDL.JntArray:
        if len(positions) != len(self.joint_names):
            raise ValueError(
                f"expected {len(self.joint_names)} joints, got {len(positions)}"
            )
        array = PyKDL.JntArray(len(positions))
        for index, position in enumerate(positions):
            array[index] = position
        return array


def damped_least_squares(jacobian: np.ndarray, twist: np.ndarray, damping: float) -> np.ndarray:
    """Joint velocity dq = J^T (J J^T + λ^2 I)^{-1} twist."""
    rows = jacobian.shape[0]
    gram = jacobian @ jacobian.T + (damping * damping) * np.eye(rows)
    return jacobian.T @ np.linalg.solve(gram, twist)


def joint_limit_velocity(
    positions: Sequence[float],
    limits: Sequence[tuple[float, float]],
    k_lim: float,
    rho: float,
    qdot_max: float,
) -> np.ndarray:
    """Repulsive joint velocity from U = 1/2 k (1/δ − 1/ρ)^2 when δ < ρ.

    δ is the distance to one bound. The speed is −dU/dq, clipped so the
    1/δ^2 singularity stays finite. Continuous joints (infinite limits) are skipped.
    """
    velocity = np.zeros(len(positions))
    if k_lim == 0.0 or rho <= 0.0:
        return velocity
    for index, position in enumerate(positions):
        lower, upper = limits[index]
        push = 0.0
        if math.isfinite(lower):
            push += _limit_push(float(position) - lower, k_lim, rho)
        if math.isfinite(upper):
            push -= _limit_push(upper - float(position), k_lim, rho)
        velocity[index] = max(-qdot_max, min(qdot_max, push))
    return velocity


def _limit_push(delta: float, k_lim: float, rho: float) -> float:
    """Positive speed that increases clearance to one bound."""
    if delta >= rho:
        return 0.0
    # Floor the clearance so a joint sitting on the bound does not blow up.
    safe = max(delta, 1e-3)
    return k_lim * (1.0 / safe - 1.0 / rho) / (safe * safe)


def chain_from_urdf(
    urdf: str, base_link: str, tip_link: str
) -> tuple[PyKDL.Chain, tuple[str, ...], tuple[tuple[float, float], ...]]:
    root = ET.fromstring(urdf)
    joints_by_child: dict[str, tuple[str, ET.Element]] = {}
    for joint in _named(root, "joint"):
        parent = _child(joint, "parent")
        child = _child(joint, "child")
        if parent is None or child is None:
            continue
        joints_by_child[child.get("link", "")] = (parent.get("link", ""), joint)

    path = _path_to_base(joints_by_child, base_link, tip_link)
    chain = PyKDL.Chain()
    names: list[str] = []
    limits: list[tuple[float, float]] = []
    for joint in path:
        segment, joint_name, joint_limits = _segment(joint)
        chain.addSegment(segment)
        if joint_name is not None and joint_limits is not None:
            names.append(joint_name)
            limits.append(joint_limits)
    if not names:
        raise ValueError(f"URDF chain from {base_link} to {tip_link} has no movable joints")
    return chain, tuple(names), tuple(limits)


def _path_to_base(
    joints_by_child: dict[str, tuple[str, ET.Element]], base_link: str, tip_link: str
) -> list[ET.Element]:
    path: list[ET.Element] = []
    link = tip_link
    seen: set[str] = set()
    while link != base_link:
        if link in seen:
            raise ValueError(f"URDF loop while walking from {tip_link} to {base_link}")
        seen.add(link)
        if link not in joints_by_child:
            raise ValueError(f"no joint path from {base_link} to {tip_link}; stopped at {link}")
        parent, joint = joints_by_child[link]
        path.append(joint)
        link = parent
    path.reverse()
    return path


def _segment(
    joint: ET.Element,
) -> tuple[PyKDL.Segment, str | None, tuple[float, float] | None]:
    name = joint.get("name", "")
    child = _child(joint, "child")
    child_name = "" if child is None else child.get("link", "")
    frame = _origin_frame(joint)
    kind = joint.get("type", "")
    if kind == "fixed":
        kdl_joint = PyKDL.Joint(name, PyKDL.Joint.Fixed)
        return PyKDL.Segment(child_name, kdl_joint, frame), None, None
    if kind not in ("revolute", "continuous"):
        raise ValueError(f"joint {name} has unsupported type {kind!r}")

    axis = _axis(joint)
    axis_in_parent = frame.M * PyKDL.Vector(*axis)
    axis_norm = math.sqrt(
        axis_in_parent.x() ** 2 + axis_in_parent.y() ** 2 + axis_in_parent.z() ** 2
    )
    if axis_norm < 1e-9:
        raise ValueError(f"joint {name} has a zero axis")
    axis_in_parent = axis_in_parent / axis_norm
    kdl_joint = PyKDL.Joint(
        name,
        PyKDL.Vector(frame.p),
        axis_in_parent,
        PyKDL.Joint.RotAxis,
    )
    return PyKDL.Segment(child_name, kdl_joint, frame), name, _limits(joint, kind)


def _origin_frame(joint: ET.Element) -> PyKDL.Frame:
    origin = _child(joint, "origin")
    xyz = (0.0, 0.0, 0.0)
    rpy = (0.0, 0.0, 0.0)
    if origin is not None:
        xyz = _floats(origin.get("xyz"), xyz)
        rpy = _floats(origin.get("rpy"), rpy)
    return PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))


def _axis(joint: ET.Element) -> Vec3:
    axis = _child(joint, "axis")
    if axis is None:
        return (1.0, 0.0, 0.0)
    return _floats(axis.get("xyz"), (1.0, 0.0, 0.0))


def _limits(joint: ET.Element, kind: str) -> tuple[float, float]:
    if kind == "continuous":
        return (float("-inf"), float("inf"))
    limit = _child(joint, "limit")
    if limit is None or limit.get("lower") is None or limit.get("upper") is None:
        return (float("-inf"), float("inf"))
    return (float(limit.get("lower", "0")), float(limit.get("upper", "0")))


def _floats(text: str | None, default: Vec3) -> Vec3:
    if not text:
        return default
    values = tuple(float(item) for item in text.split())
    if len(values) != 3:
        raise ValueError(f"expected 3 floats, got {text!r}")
    return values


def _named(root: ET.Element, name: str) -> list[ET.Element]:
    return [element for element in root.iter() if _local(element.tag) == name]


def _child(element: ET.Element, name: str) -> ET.Element | None:
    for child in element:
        if _local(child.tag) == name:
            return child
    return None


def _local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]
