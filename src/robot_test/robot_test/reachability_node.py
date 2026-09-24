#!/usr/bin/env python3
"""Cartesian line to the scene target with per-step collision checks."""

import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rclpy.node import Node
from scipy.spatial.transform import Rotation, Slerp
from urdf_parser_py.urdf import URDF

ACTUATED_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
POSITION_TOLERANCE_M = 1e-3
ORIENTATION_TOLERANCE_RAD = math.radians(1.0)
TRACK_POSITION_M = 0.02
TRACK_ORIENTATION_RAD = math.radians(5.0)


@dataclass(frozen=True)
class Sphere:
    center: np.ndarray
    radius: float


@dataclass(frozen=True)
class TargetPose:
    position: np.ndarray
    rpy: np.ndarray


@dataclass(frozen=True)
class DhParameters:
    d1: float
    a2: float
    a3: float
    d4: float
    d5: float
    d6: float


@dataclass(frozen=True)
class TrajectoryReport:
    reached_goal: bool
    collision_free: bool
    ik_failed: bool
    steps: int
    first_collision_step: int | None
    final_position_error_m: float
    final_orientation_error_rad: float
    min_clearance_m: float


def world_path() -> Path:
    try:
        installed = Path(get_package_share_directory("robot_sim")) / "worlds" / "empty.sdf"
    except PackageNotFoundError:
        installed = None
    if installed is not None and installed.is_file():
        return installed
    source = Path(__file__).resolve().parents[2] / "robot_sim" / "worlds" / "empty.sdf"
    if not source.is_file():
        raise FileNotFoundError("robot_sim world empty.sdf was not found")
    return source


def load_scene(path: Path) -> tuple[Sphere, TargetPose]:
    root = ET.parse(path).getroot()
    sphere_pose = root.find("./world/model[@name='red_sphere']/pose")
    radius = root.find("./world/model[@name='red_sphere']//sphere/radius")
    target = root.find("./world/model[@name='target_pose']/pose")
    if sphere_pose is None or radius is None or target is None:
        raise ValueError(f"{path} is missing red_sphere or target_pose")

    def parse_pose(text: str) -> tuple[np.ndarray, np.ndarray]:
        values = [float(item) for item in text.split()]
        position = np.asarray(values[:3], dtype=float)
        rpy = np.asarray(values[3:], dtype=float)
        return position, rpy

    center, _ = parse_pose(sphere_pose.text or "")
    position, rpy = parse_pose(target.text or "")
    return Sphere(center, float(radius.text)), TargetPose(position, rpy)


def load_home_joints() -> np.ndarray:
    initial = Path(get_package_share_directory("ur_description")) / "config" / "initial_positions.yaml"
    values: dict[str, float] = {}
    for line in initial.read_text().splitlines():
        if ":" not in line:
            continue
        name, raw = line.split(":", maxsplit=1)
        values[name.strip()] = float(raw.strip())
    return np.asarray([values[name] for name in ACTUATED_JOINTS], dtype=float)


def load_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = path.read_bytes()
    if data.lstrip().lower().startswith(b"solid") and b"facet" in data[:1000].lower():
        vertices: list[list[float]] = []
        for line in data.decode("utf-8", errors="ignore").splitlines():
            parts = line.split()
            if len(parts) == 4 and parts[0] == "vertex":
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
        points = np.asarray(vertices, dtype=float)
        faces = np.arange(points.shape[0], dtype=int).reshape(-1, 3)
        return points, faces
    count = int(np.frombuffer(data, dtype="<u4", count=1, offset=80)[0])
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertex", "<f4", (3, 3)), ("attr", "<u2")])
    triangles = np.frombuffer(data, dtype=dtype, count=count, offset=84)
    vertices = np.asarray(triangles["vertex"], dtype=float).reshape(-1, 3)
    faces = np.arange(vertices.shape[0], dtype=int).reshape(-1, 3)
    return vertices, faces


def triangle_distances(triangles: np.ndarray, point: np.ndarray) -> np.ndarray:
    a = triangles[:, 0]
    b = triangles[:, 1]
    c = triangles[:, 2]
    ab = b - a
    ac = c - a
    ap = point - a
    d1 = np.einsum("ij,ij->i", ab, ap)
    d2 = np.einsum("ij,ij->i", ac, ap)
    d3 = np.einsum("ij,ij->i", ab, b - point)
    d4 = np.einsum("ij,ij->i", ac, b - point)
    d5 = np.einsum("ij,ij->i", ab, c - point)
    d6 = np.einsum("ij,ij->i", ac, c - point)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    closest = np.empty_like(a)
    claimed = np.zeros(a.shape[0], dtype=bool)

    def take(mask: np.ndarray) -> np.ndarray:
        use = mask & ~claimed
        claimed[use] = True
        return use

    use = take((d1 <= 0.0) & (d2 <= 0.0))
    closest[use] = a[use]
    use = take((d3 >= 0.0) & (d4 <= d3))
    closest[use] = b[use]
    use = take((vc <= 0.0) & (d1 >= 0.0) & (d3 <= 0.0))
    edge = np.maximum(d1[use] - d3[use], 1e-12)
    closest[use] = a[use] + (d1[use] / edge)[:, None] * ab[use]
    use = take((d6 >= 0.0) & (d5 <= d6))
    closest[use] = c[use]
    use = take((vb <= 0.0) & (d2 >= 0.0) & (d6 <= 0.0))
    edge = np.maximum(d2[use] - d6[use], 1e-12)
    closest[use] = a[use] + (d2[use] / edge)[:, None] * ac[use]
    use = take((va <= 0.0) & ((d4 - d3) >= 0.0) & ((d5 - d6) >= 0.0))
    edge = np.maximum((d4[use] - d3[use]) + (d5[use] - d6[use]), 1e-12)
    closest[use] = b[use] + ((d4[use] - d3[use]) / edge)[:, None] * (c[use] - b[use])
    use = ~claimed
    denom = np.maximum(va[use] + vb[use] + vc[use], 1e-12)
    v = vb[use] / denom
    w = vc[use] / denom
    closest[use] = a[use] + v[:, None] * ab[use] + w[:, None] * ac[use]
    delta = closest - point
    return np.sqrt(np.einsum("ij,ij->i", delta, delta))


def sphere_clearance(
    vertices: np.ndarray,
    faces: np.ndarray,
    center: np.ndarray,
    radius: float,
) -> float:
    triangles = vertices[faces]
    return float(np.min(triangle_distances(triangles, center)) - radius)


def pose_matrix(position: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    transform[:3, 3] = position
    return transform


def pose_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    translation = target[:3, 3] - current[:3, 3]
    relative = current[:3, :3].T @ target[:3, :3]
    rotation_base = current[:3, :3] @ Rotation.from_matrix(relative).as_rotvec()
    return np.concatenate([translation, rotation_base])


def _joint_xyz(joint) -> np.ndarray:
    if joint.origin is None or joint.origin.xyz is None:
        return np.zeros(3)
    return np.asarray(joint.origin.xyz, dtype=float)


def _dh_from_joints(joints: dict) -> DhParameters:
    shoulder = _joint_xyz(joints["shoulder_pan_joint"])
    elbow = _joint_xyz(joints["elbow_joint"])
    wrist_1 = _joint_xyz(joints["wrist_1_joint"])
    wrist_2 = _joint_xyz(joints["wrist_2_joint"])
    wrist_3 = _joint_xyz(joints["wrist_3_joint"])
    return DhParameters(shoulder[2], elbow[0], wrist_1[0], wrist_1[2], -wrist_2[1], wrist_3[1])


def _sign(value: float) -> int:
    return (value > 0) - (value < 0)


def _ur_ik_frame(tool_in_base: np.ndarray) -> np.ndarray:
    base_rotation = np.diag([-1.0, -1.0, 1.0, 1.0])
    tool_rotation = np.eye(4)
    tool_rotation[:3, :3] = np.array(
        [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ]
    )
    return base_rotation @ tool_in_base @ tool_rotation


def _analytical_ik(tool_in_base: np.ndarray, dh: DhParameters) -> list[np.ndarray]:
    matrix = _ur_ik_frame(tool_in_base).reshape(-1)
    t02 = -matrix[0]
    t00 = matrix[1]
    t01 = matrix[2]
    t03 = -matrix[3]
    t12 = -matrix[4]
    t10 = matrix[5]
    t11 = matrix[6]
    t13 = -matrix[7]
    t22 = matrix[8]
    t20 = -matrix[9]
    t21 = -matrix[10]
    t23 = matrix[11]
    d1, a2, a3, d4, d5, d6 = dh.d1, dh.a2, dh.a3, dh.d4, dh.d5, dh.d6
    zero = 1e-8

    shoulder_y = d6 * t12 - t13
    shoulder_x = d6 * t02 - t03
    radius_sq = shoulder_y * shoulder_y + shoulder_x * shoulder_x
    q1 = [0.0, 0.0]
    if abs(shoulder_y) < zero:
        ratio = (
            -_sign(d4) * _sign(shoulder_x)
            if abs(abs(d4) - abs(shoulder_x)) < zero
            else -d4 / shoulder_x
        )
        arcsin = math.asin(float(np.clip(ratio, -1.0, 1.0)))
        q1[0] = arcsin + (2.0 * math.pi if arcsin < 0.0 else 0.0)
        q1[1] = math.pi - arcsin
    elif abs(shoulder_x) < zero:
        ratio = (
            _sign(d4) * _sign(shoulder_y)
            if abs(abs(d4) - abs(shoulder_y)) < zero
            else d4 / shoulder_y
        )
        arccos = math.acos(float(np.clip(ratio, -1.0, 1.0)))
        q1[0] = arccos
        q1[1] = 2.0 * math.pi - arccos
    elif d4 * d4 > radius_sq:
        return []
    else:
        arccos = math.acos(float(np.clip(d4 / math.sqrt(radius_sq), -1.0, 1.0)))
        arctan = math.atan2(-shoulder_x, shoulder_y)
        positive = arccos + arctan
        negative = -arccos + arctan
        q1[0] = positive if positive >= 0.0 else 2.0 * math.pi + positive
        q1[1] = negative if negative >= 0.0 else 2.0 * math.pi + negative

    q5 = [[0.0, 0.0], [0.0, 0.0]]
    for index in range(2):
        numer = t03 * math.sin(q1[index]) - t13 * math.cos(q1[index]) - d4
        ratio = _sign(numer) * _sign(d6) if abs(abs(numer) - abs(d6)) < zero else numer / d6
        arccos = math.acos(float(np.clip(ratio, -1.0, 1.0)))
        q5[index][0] = arccos
        q5[index][1] = 2.0 * math.pi - arccos

    solutions: list[np.ndarray] = []
    for shoulder in range(2):
        for wrist in range(2):
            c1 = math.cos(q1[shoulder])
            s1 = math.sin(q1[shoulder])
            c5 = math.cos(q5[shoulder][wrist])
            s5 = math.sin(q5[shoulder][wrist])
            if abs(s5) < zero:
                q6 = 0.0
            else:
                q6 = math.atan2(
                    _sign(s5) * -(t01 * s1 - t11 * c1),
                    _sign(s5) * (t00 * s1 - t10 * c1),
                )
                if q6 < 0.0:
                    q6 += 2.0 * math.pi
            c6 = math.cos(q6)
            s6 = math.sin(q6)
            x04x = -s5 * (t02 * c1 + t12 * s1) - c5 * (
                s6 * (t01 * c1 + t11 * s1) - c6 * (t00 * c1 + t10 * s1)
            )
            x04y = c5 * (t20 * c6 - t21 * s6) - t22 * s5
            p13x = (
                d5 * (s6 * (t00 * c1 + t10 * s1) + c6 * (t01 * c1 + t11 * s1))
                - d6 * (t02 * c1 + t12 * s1)
                + t03 * c1
                + t13 * s1
            )
            p13y = t23 - d1 - d6 * t22 + d5 * (t21 * c6 + t20 * s6)
            c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
            if abs(c3) > 1.0 + 1e-9:
                continue
            c3 = float(np.clip(c3, -1.0, 1.0))
            elbow_angle = math.acos(c3)
            q3_pair = [elbow_angle, 2.0 * math.pi - elbow_angle]
            denom = a2 * a2 + a3 * a3 + 2.0 * a2 * a3 * c3
            s3 = math.sin(elbow_angle)
            lift_a = a2 + a3 * c3
            lift_b = a3 * s3
            q2_pair = [
                math.atan2(
                    (lift_a * p13y - lift_b * p13x) / denom,
                    (lift_a * p13x + lift_b * p13y) / denom,
                ),
                math.atan2(
                    (lift_a * p13y + lift_b * p13x) / denom,
                    (lift_a * p13x - lift_b * p13y) / denom,
                ),
            ]
            for branch in range(2):
                c23 = math.cos(q2_pair[branch] + q3_pair[branch])
                s23 = math.sin(q2_pair[branch] + q3_pair[branch])
                q4 = math.atan2(c23 * x04y - s23 * x04x, x04x * c23 + x04y * s23)
                q2 = q2_pair[branch]
                if q2 < 0.0:
                    q2 += 2.0 * math.pi
                if q4 < 0.0:
                    q4 += 2.0 * math.pi
                solutions.append(
                    np.array([q1[shoulder], q2, q3_pair[branch], q4, q5[shoulder][wrist], q6])
                )
    return solutions


def _fold_into_limits(joints: np.ndarray, limits: np.ndarray) -> np.ndarray | None:
    folded: list[float] = []
    for angle, (lower, upper) in zip(joints, limits, strict=True):
        candidates = [
            angle + turn * 2.0 * math.pi
            for turn in range(-2, 3)
            if lower - 1e-9 <= angle + turn * 2.0 * math.pi <= upper + 1e-9
        ]
        if not candidates:
            return None
        folded.append(min(candidates, key=abs))
    return np.asarray(folded, dtype=float)


def _joint_distance(left: np.ndarray, right: np.ndarray) -> float:
    delta = (left - right + math.pi) % (2.0 * math.pi) - math.pi
    return float(np.linalg.norm(delta))


def pose_delta(current: np.ndarray, target: np.ndarray) -> tuple[float, float]:
    error = pose_error(current, target)
    return float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))


def close_enough(
    current: np.ndarray,
    target: np.ndarray,
    position_tol: float,
    orientation_tol: float,
) -> bool:
    position, orientation = pose_delta(current, target)
    return position <= position_tol and orientation <= orientation_tol


def reaches(current: np.ndarray, target: np.ndarray) -> tuple[bool, float, float]:
    position, orientation = pose_delta(current, target)
    ok = position <= POSITION_TOLERANCE_M and orientation <= ORIENTATION_TOLERANCE_RAD
    return ok, position, orientation


class Ur16e:
    def __init__(self) -> None:
        xacro = Path(get_package_share_directory("ur_description")) / "urdf" / "ur.urdf.xacro"
        xml = subprocess.run(
            ["xacro", str(xacro), "name:=ur", "ur_type:=ur16e"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        xml = re.sub(r"<ros2_control\b.*?</ros2_control>", "", xml, flags=re.DOTALL)
        robot = URDF.from_xml_string(xml)
        self._parents: dict[str, tuple[str, str]] = {}
        for joint in robot.joints:
            self._parents[joint.child] = (joint.name, joint.parent)
        self._joints = {joint.name: joint for joint in robot.joints}
        self.limits = np.array(
            [
                [self._joints[name].limit.lower, self._joints[name].limit.upper]
                for name in ACTUATED_JOINTS
            ],
            dtype=float,
        )
        self._dh = _dh_from_joints(self._joints)
        self.bodies = self._collision_bodies(robot)

    def _resolve_mesh(self, filename: str) -> Path:
        if filename.startswith("package://"):
            package, _, relative = filename[len("package://") :].partition("/")
            return Path(get_package_share_directory(package)) / relative
        if filename.startswith("file://"):
            return Path(filename[len("file://") :])
        return Path(filename)

    def _origin_matrix(self, origin) -> np.ndarray:
        transform = np.eye(4)
        if origin is None:
            return transform
        xyz = np.zeros(3) if origin.xyz is None else np.asarray(origin.xyz, dtype=float)
        rpy = np.zeros(3) if origin.rpy is None else np.asarray(origin.rpy, dtype=float)
        transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        transform[:3, 3] = xyz
        return transform

    def _collision_bodies(self, robot: URDF):
        bodies = []
        for link in robot.links:
            if link.name == "ground_plane" or not link.collisions:
                continue
            for collision in link.collisions:
                geometry = collision.geometry
                if geometry is None or not hasattr(geometry, "filename"):
                    continue
                vertices, faces = load_stl(self._resolve_mesh(geometry.filename))
                origin = self._origin_matrix(collision.origin)
                local = (origin[:3, :3] @ vertices.T).T + origin[:3, 3]
                bodies.append((link.name, local, faces))
        if not bodies:
            raise RuntimeError("UR16e URDF has no collision meshes")
        return bodies

    def link_transform(self, link_name: str, joints: np.ndarray) -> np.ndarray:
        q_by_name = dict(zip(ACTUATED_JOINTS, joints, strict=True))
        chain: list[str] = []
        link = link_name
        while link in self._parents:
            joint_name, parent = self._parents[link]
            chain.append(joint_name)
            link = parent
        transform = np.eye(4)
        for joint_name in reversed(chain):
            joint = self._joints[joint_name]
            transform = transform @ self._origin_matrix(joint.origin)
            if joint.type in ("revolute", "continuous"):
                axis = np.asarray(joint.axis, dtype=float)
                axis = axis / np.linalg.norm(axis)
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis * q_by_name[joint_name]).as_matrix()
                transform = transform @ rotation
        return transform

    def tip_transform(self, joints: np.ndarray) -> np.ndarray:
        return self.link_transform("tool0", joints)

    def min_clearance(self, joints: np.ndarray, sphere: Sphere) -> float:
        signed: dict[str, float] = {}
        for link_name, vertices, faces in self.bodies:
            posed = self.link_transform(link_name, joints)
            world = (posed[:3, :3] @ vertices.T).T + posed[:3, 3]
            gap = sphere_clearance(world, faces, sphere.center, sphere.radius)
            signed[link_name] = min(signed.get(link_name, gap), gap)
        return min(signed.values())

    def _solve_ik_position(self, target: np.ndarray, seed: np.ndarray) -> np.ndarray | None:
        """Track the Cartesian line in position; orientation is checked at the goal."""
        joints = np.clip(seed.astype(float), self.limits[:, 0], self.limits[:, 1])
        goal = target[:3, 3]
        for _ in range(100):
            current = self.tip_transform(joints)[:3, 3]
            error = goal - current
            if float(np.linalg.norm(error)) <= TRACK_POSITION_M:
                return joints
            jacobian = np.zeros((3, 6))
            for index in range(6):
                bumped = joints.copy()
                bumped[index] += 1e-6
                jacobian[:, index] = (self.tip_transform(bumped)[:3, 3] - current) / 1e-6
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + (1e-4**2) * np.eye(3),
                error,
            )
            joints = np.clip(joints + np.clip(step, -0.2, 0.2), self.limits[:, 0], self.limits[:, 1])
        if float(np.linalg.norm(goal - self.tip_transform(joints)[:3, 3])) <= TRACK_POSITION_M:
            return joints
        return None

    def solve_ik(
        self,
        target: np.ndarray,
        seed: np.ndarray,
        *,
        final: bool = False,
    ) -> np.ndarray | None:
        if not final:
            return self._solve_ik_position(target, seed)

        base = self.link_transform("base", np.zeros(6))
        tool_in_base = np.linalg.inv(base) @ target
        best: np.ndarray | None = None
        best_distance = math.inf
        for raw in _analytical_ik(tool_in_base, self._dh):
            joints = _fold_into_limits(raw, self.limits)
            if joints is None:
                continue
            if not close_enough(
                self.tip_transform(joints),
                target,
                POSITION_TOLERANCE_M,
                ORIENTATION_TOLERANCE_RAD,
            ):
                continue
            distance = _joint_distance(joints, seed)
            if distance < best_distance:
                best_distance = distance
                best = joints
        if best is not None:
            return best

        joints = np.clip(seed.astype(float), self.limits[:, 0], self.limits[:, 1])
        for _ in range(120):
            current = self.tip_transform(joints)
            if close_enough(
                current,
                target,
                POSITION_TOLERANCE_M,
                ORIENTATION_TOLERANCE_RAD,
            ):
                return joints
            error = pose_error(current, target)
            jacobian = np.zeros((6, 6))
            for index in range(6):
                bumped = joints.copy()
                bumped[index] += 1e-6
                jacobian[:, index] = (pose_error(self.tip_transform(bumped), target) - error) / 1e-6
            step = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + (1e-3**2) * np.eye(6),
                error,
            )
            joints = np.clip(joints + np.clip(step, -0.15, 0.15), self.limits[:, 0], self.limits[:, 1])
        current = self.tip_transform(joints)
        if close_enough(
            current,
            target,
            POSITION_TOLERANCE_M,
            ORIENTATION_TOLERANCE_RAD,
        ):
            return joints
        return None


def cartesian_poses(start: np.ndarray, goal: np.ndarray, step_m: float) -> list[np.ndarray]:
    distance = float(np.linalg.norm(goal[:3, 3] - start[:3, 3]))
    count = max(int(math.ceil(distance / step_m)), 1)
    rotations = Rotation.from_matrix(np.stack([start[:3, :3], goal[:3, :3]]))
    slerp = Slerp([0.0, 1.0], rotations)
    poses: list[np.ndarray] = []
    for index in range(count + 1):
        alpha = index / count
        pose = np.eye(4)
        pose[:3, :3] = slerp(alpha).as_matrix()
        pose[:3, 3] = (1.0 - alpha) * start[:3, 3] + alpha * goal[:3, 3]
        poses.append(pose)
    return poses


def follow_cartesian_line(
    arm: Ur16e,
    start_joints: np.ndarray,
    goal_pose: np.ndarray,
    sphere: Sphere,
    step_m: float,
) -> TrajectoryReport:
    start_pose = arm.tip_transform(start_joints)
    poses = cartesian_poses(start_pose, goal_pose, step_m)
    joints = start_joints.copy()
    first_collision_step: int | None = None
    min_clearance = math.inf
    ik_failed = False

    gap = arm.min_clearance(joints, sphere)
    min_clearance = min(min_clearance, gap)

    for step, pose in enumerate(poses[1:], start=1):
        final_step = step == len(poses) - 1
        solved = arm.solve_ik(pose, joints, final=final_step)
        if solved is None:
            ik_failed = True
            break
        joints = solved
        gap = arm.min_clearance(joints, sphere)
        min_clearance = min(min_clearance, gap)
        if gap <= 0.0 and first_collision_step is None:
            first_collision_step = step

    final = arm.tip_transform(joints)
    reached, pos_err, rot_err = reaches(final, goal_pose)
    return TrajectoryReport(
        reached_goal=reached,
        collision_free=first_collision_step is None,
        ik_failed=ik_failed,
        steps=len(poses) - 1,
        first_collision_step=first_collision_step,
        final_position_error_m=pos_err,
        final_orientation_error_rad=rot_err,
        min_clearance_m=min_clearance if math.isfinite(min_clearance) else -math.inf,
    )


class ReachabilityNode(Node):
    def __init__(self) -> None:
        super().__init__("reachability_node")
        self.declare_parameter("world", str(world_path()))
        self.declare_parameter("cartesian_step_m", 0.01)

    def run(self) -> TrajectoryReport:
        world = Path(self.get_parameter("world").get_parameter_value().string_value)
        step_m = self.get_parameter("cartesian_step_m").get_parameter_value().double_value
        sphere, target = load_scene(world)
        goal = pose_matrix(target.position, target.rpy)
        home = load_home_joints()
        arm = Ur16e()
        report = follow_cartesian_line(arm, home, goal, sphere, step_m)

        if report.ik_failed:
            self.get_logger().error("IK failed while tracking the Cartesian line")
        if not report.collision_free:
            self.get_logger().error(
                f"collision at step {report.first_collision_step}, "
                f"min clearance {report.min_clearance_m:.4f} m"
            )
        if report.reached_goal:
            self.get_logger().info(
                "reached target_pose: "
                f"pos err {report.final_position_error_m:.2e} m, "
                f"rot err {math.degrees(report.final_orientation_error_rad):.2f} deg"
            )
        else:
            self.get_logger().error(
                "did not reach target_pose: "
                f"pos err {report.final_position_error_m:.2e} m, "
                f"rot err {math.degrees(report.final_orientation_error_rad):.2f} deg"
            )
        return report


def main() -> None:
    rclpy.init()
    node = ReachabilityNode()
    report = node.run()
    node.destroy_node()
    rclpy.shutdown()
    success = report.reached_goal and report.collision_free and not report.ik_failed
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
