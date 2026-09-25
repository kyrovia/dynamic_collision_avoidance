"""Build a PyKDL chain from URDF for the UR arm."""

import math
import xml.etree.ElementTree as ET

import PyKDL

Vec3 = tuple[float, float, float]


def chain_from_urdf(
    urdf: str, base_link: str, tip_link: str
) -> tuple[PyKDL.Chain, tuple[str, ...], tuple[tuple[float, float], ...]]:
    root = ET.fromstring(urdf)
    inertias = _link_inertias(root)
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
        segment, joint_name, joint_limits = _segment(joint, inertias)
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


def _link_inertias(root: ET.Element) -> dict[str, PyKDL.RigidBodyInertia]:
    inertias: dict[str, PyKDL.RigidBodyInertia] = {}
    for link in _named(root, "link"):
        name = link.get("name", "")
        if not name:
            continue
        inertial = _child(link, "inertial")
        if inertial is None:
            inertias[name] = PyKDL.RigidBodyInertia.Zero()
            continue
        mass_elem = _child(inertial, "mass")
        inertia_elem = _child(inertial, "inertia")
        origin = _child(inertial, "origin")
        if mass_elem is None or inertia_elem is None:
            inertias[name] = PyKDL.RigidBodyInertia.Zero()
            continue
        mass = float(mass_elem.get("value", "0"))
        xyz = (0.0, 0.0, 0.0)
        if origin is not None and origin.get("xyz"):
            xyz = _floats(origin.get("xyz"), xyz)
        rot_inertia = PyKDL.RotationalInertia(
            float(inertia_elem.get("ixx", "0")),
            float(inertia_elem.get("iyy", "0")),
            float(inertia_elem.get("izz", "0")),
            float(inertia_elem.get("ixy", "0")),
            float(inertia_elem.get("ixz", "0")),
            float(inertia_elem.get("iyz", "0")),
        )
        inertias[name] = PyKDL.RigidBodyInertia(
            mass, PyKDL.Vector(*xyz), rot_inertia
        )
    return inertias


def _segment(
    joint: ET.Element,
    inertias: dict[str, PyKDL.RigidBodyInertia],
) -> tuple[PyKDL.Segment, str | None, tuple[float, float] | None]:
    name = joint.get("name", "")
    child = _child(joint, "child")
    child_name = "" if child is None else child.get("link", "")
    frame = _origin_frame(joint)
    inertia = inertias.get(child_name, PyKDL.RigidBodyInertia.Zero())
    kind = joint.get("type", "")
    if kind == "fixed":
        kdl_joint = PyKDL.Joint(name, PyKDL.Joint.Fixed)
        return PyKDL.Segment(child_name, kdl_joint, frame, inertia), None, None
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
    return PyKDL.Segment(child_name, kdl_joint, frame, inertia), name, _limits(joint, kind)


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
