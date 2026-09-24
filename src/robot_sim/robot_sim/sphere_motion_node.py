#!/usr/bin/env python3
"""Move the red sphere relative to the home tool0-target line."""

import math
import random
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

Vec3 = tuple[float, float, float]

MOTION_MODES = ("along", "perpendicular", "oblique")
EEF_LINK = "tool0"
PLANNING_FRAME = "base_link"
MODEL_NAME = "red_sphere"
OBSTACLE_POSE_TOPIC = "/obstacle/pose"
SET_POSE_SERVICE = "/world/robot_sim/set_pose"
# Keep oblique motion distinct from the other two modes.
OBLIQUE_ANGLE_MIN_DEG = 10.0
OBLIQUE_ANGLE_MAX_DEG = 80.0
# Along-mode center stays at least this far from each endpoint.
# The tool side is larger so the ball stays clear of the home arm.
DEFAULT_ALONG_TOOL_MARGIN_M = 0.45
DEFAULT_ALONG_TARGET_MARGIN_M = 0.15


@dataclass(frozen=True)
class Sphere:
    center: Vec3
    radius: float


@dataclass
class Shuttle:
    """Constant-speed travel that reverses at the ends of a segment."""

    origin: Vec3
    direction: Vec3
    low: float
    high: float
    offset: float
    sign: float
    oblique_angle_deg: float | None = None

    def step(self, distance: float) -> Vec3:
        self.offset += self.sign * distance
        if self.offset > self.high:
            self.offset = self.high - (self.offset - self.high)
            self.sign = -1.0
        elif self.offset < self.low:
            self.offset = self.low + (self.low - self.offset)
            self.sign = 1.0
        self.offset = min(self.high, max(self.low, self.offset))
        return add(self.origin, scale(self.direction, self.offset))


def world_path() -> Path:
    try:
        installed = Path(get_package_share_directory("robot_sim")) / "worlds" / "empty.sdf"
    except PackageNotFoundError:
        installed = None
    if installed is not None and installed.is_file():
        return installed
    source = Path(__file__).resolve().parents[1] / "worlds" / "empty.sdf"
    if not source.is_file():
        raise FileNotFoundError("robot_sim world empty.sdf was not found")
    return source


def load_scene(path: Path) -> tuple[Sphere, Vec3]:
    root = ET.parse(path).getroot()
    sphere_pose = root.find("./world/model[@name='red_sphere']/pose")
    radius = root.find("./world/model[@name='red_sphere']//sphere/radius")
    target = root.find("./world/model[@name='target_pose']/pose")
    if sphere_pose is None or radius is None or target is None:
        raise ValueError(f"{path} is missing red_sphere or target_pose")

    def xyz(text: str) -> Vec3:
        values = [float(item) for item in text.split()]
        return (values[0], values[1], values[2])

    return Sphere(xyz(sphere_pose.text or ""), float(radius.text)), xyz(target.text or "")


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a: Vec3, factor: float) -> Vec3:
    return (a[0] * factor, a[1] * factor, a[2] * factor)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a: Vec3) -> float:
    return math.sqrt(dot(a, a))


def unit(a: Vec3) -> Vec3:
    length = norm(a)
    if length < 1e-9:
        raise ValueError("cannot normalize a zero vector")
    return scale(a, 1.0 / length)


def perpendicular_axis(axis: Vec3) -> Vec3:
    """A unit axis perpendicular to the tool0-target line, preferably horizontal."""
    perp = cross(axis, (0.0, 0.0, 1.0))
    if norm(perp) < 1e-6:
        perp = cross(axis, (1.0, 0.0, 0.0))
    return unit(perp)


def sample_oblique_angle_deg(rng: random.Random) -> float:
    magnitude = rng.uniform(OBLIQUE_ANGLE_MIN_DEG, OBLIQUE_ANGLE_MAX_DEG)
    return magnitude if rng.random() < 0.5 else -magnitude


def make_shuttle(
    mode: str,
    tool0: Vec3,
    target: Vec3,
    sphere_center: Vec3,
    rng: random.Random,
    along_tool_margin: float = DEFAULT_ALONG_TOOL_MARGIN_M,
    along_target_margin: float = DEFAULT_ALONG_TARGET_MARGIN_M,
    sphere_radius: float = 0.0,
) -> Shuttle:
    """Build one back-and-forth path. Oblique samples a new angle each call."""
    axis = unit(sub(target, tool0))
    length = norm(sub(target, tool0))
    if mode == "along":
        # Keep the ball body off both endpoints, then apply the larger gaps.
        low = max(along_tool_margin, sphere_radius)
        high = length - max(along_target_margin, sphere_radius)
        if low >= high:
            raise ValueError("along margins leave no room between tool0 and target")
        offset = min(high, max(low, dot(sub(sphere_center, tool0), axis)))
        return Shuttle(tool0, axis, low, high, offset, 1.0)

    perp = perpendicular_axis(axis)
    angle_deg = None
    if mode == "perpendicular":
        direction = perp
    elif mode == "oblique":
        angle_deg = sample_oblique_angle_deg(rng)
        angle = math.radians(angle_deg)
        direction = unit(add(scale(axis, math.cos(angle)), scale(perp, math.sin(angle))))
    else:
        raise ValueError(f"unknown motion mode {mode!r}")

    half = 0.5 * length
    return Shuttle(sphere_center, direction, -half, half, 0.0, 1.0, angle_deg)


class SphereMotionNode(Node):
    def __init__(self) -> None:
        super().__init__("sphere_motion_node")
        self.declare_parameter("world", str(world_path()))
        self.declare_parameter("motion_mode", "along")
        self.declare_parameter("speed", 0.05)
        self.declare_parameter("along_tool_margin", DEFAULT_ALONG_TOOL_MARGIN_M)
        self.declare_parameter("along_target_margin", DEFAULT_ALONG_TARGET_MARGIN_M)
        self.declare_parameter("eef_link", EEF_LINK)
        self.declare_parameter("planning_frame", PLANNING_FRAME)
        self.declare_parameter("update_rate", 20.0)

        self._rng = random.Random()
        self._sphere: Sphere | None = None
        self._target: Vec3 | None = None
        self._tool0: Vec3 | None = None
        self._shuttle: Shuttle | None = None
        self._last_time = None
        self._pose_busy = False

        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._collision = self.create_publisher(CollisionObject, "/collision_object", 10)
        self._obstacle_pose = self.create_publisher(PoseStamped, OBSTACLE_POSE_TOPIC, 10)
        self._set_pose = self.create_client(SetEntityPose, SET_POSE_SERVICE)
        self.add_on_set_parameters_callback(self._on_parameters)

        rate = float(self.get_parameter("update_rate").value)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

    def _on_parameters(self, params: list) -> SetParametersResult:
        rebuild = False
        for param in params:
            if param.name == "motion_mode":
                if param.value not in MOTION_MODES:
                    return SetParametersResult(
                        successful=False,
                        reason=f"motion_mode must be one of {', '.join(MOTION_MODES)}",
                    )
                rebuild = True
            if param.name == "speed" and float(param.value) <= 0.0:
                return SetParametersResult(successful=False, reason="speed must be positive")
            if param.name in ("along_tool_margin", "along_target_margin"):
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{param.name} must be positive",
                    )
                rebuild = True
        if rebuild:
            self._shuttle = None
        return SetParametersResult(successful=True)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if self._shuttle is None and not self._configure():
            self._last_time = now
            return
        if self._last_time is None or self._shuttle is None or self._sphere is None:
            self._last_time = now
            return

        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0:
            return
        dt = min(dt, 0.1)
        position = self._shuttle.step(float(self.get_parameter("speed").value) * dt)
        self._publish_collision(position)
        self._publish_obstacle_pose(position)
        self._publish_gazebo(position)

    def _configure(self) -> bool:
        if self._sphere is None or self._target is None:
            world = Path(self.get_parameter("world").get_parameter_value().string_value)
            self._sphere, self._target = load_scene(world)
        if self._tool0 is None:
            looked_up = self._lookup_tool0()
            if looked_up is None:
                return False
            self._tool0 = looked_up

        mode = str(self.get_parameter("motion_mode").value)
        if self._sphere is None or self._target is None or self._tool0 is None:
            return False
        try:
            self._shuttle = make_shuttle(
                mode,
                self._tool0,
                self._target,
                self._sphere.center,
                self._rng,
                float(self.get_parameter("along_tool_margin").value),
                float(self.get_parameter("along_target_margin").value),
                self._sphere.radius,
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return False

        speed = float(self.get_parameter("speed").value)
        if mode == "along":
            length = norm(sub(self._target, self._tool0))
            target_gap = length - self._shuttle.high
            self.get_logger().info(
                f"red sphere motion=along speed={speed:.3f} m/s, "
                f"stays {self._shuttle.low:.3f} m from tool0 and "
                f"{target_gap:.3f} m from target"
            )
        elif self._shuttle.oblique_angle_deg is None:
            self.get_logger().info(f"red sphere motion={mode} speed={speed:.3f} m/s")
        else:
            angle = self._shuttle.oblique_angle_deg
            self.get_logger().info(
                f"red sphere motion=oblique angle={angle:.1f} deg "
                f"from the tool0-target line, speed={speed:.3f} m/s"
            )
        return True

    def _lookup_tool0(self) -> Vec3 | None:
        frame = str(self.get_parameter("planning_frame").value)
        link = str(self.get_parameter("eef_link").value)
        if not self._tf.can_transform(frame, link, rclpy.time.Time()):
            self.get_logger().info(
                f"waiting for {frame} -> {link} before choosing the motion line",
                throttle_duration_sec=5.0,
            )
            return None
        transform = self._tf.lookup_transform(frame, link, rclpy.time.Time())
        point = transform.transform.translation
        tool0 = (point.x, point.y, point.z)
        self.get_logger().info(
            f"locked tool0-target line from tool0 [{tool0[0]:.3f}, {tool0[1]:.3f}, {tool0[2]:.3f}]"
        )
        return tool0

    def _publish_collision(self, position: Vec3) -> None:
        if self._sphere is None:
            return
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self._sphere.radius]
        pose = _pose(position)

        object_msg = CollisionObject()
        object_msg.header.frame_id = str(self.get_parameter("planning_frame").value)
        object_msg.header.stamp = self.get_clock().now().to_msg()
        object_msg.id = MODEL_NAME
        object_msg.primitives.append(primitive)
        object_msg.primitive_poses.append(pose)
        object_msg.operation = CollisionObject.ADD
        self._collision.publish(object_msg)

    def _publish_obstacle_pose(self, position: Vec3) -> None:
        stamped = PoseStamped()
        stamped.header.frame_id = str(self.get_parameter("planning_frame").value)
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = _pose(position)
        self._obstacle_pose.publish(stamped)

    def _publish_gazebo(self, position: Vec3) -> None:
        if self._pose_busy or not self._set_pose.service_is_ready():
            if not self._set_pose.service_is_ready():
                self.get_logger().warning(
                    "waiting for %s",
                    SET_POSE_SERVICE,
                    throttle_duration_sec=5.0,
                )
            return
        request = SetEntityPose.Request()
        request.entity.name = MODEL_NAME
        request.entity.type = Entity.MODEL
        request.pose = _pose(position)
        self._pose_busy = True
        future = self._set_pose.call_async(request)
        future.add_done_callback(self._on_pose_done)

    def _on_pose_done(self, future) -> None:
        self._pose_busy = False
        result = future.result()
        if result is None or not result.success:
            self.get_logger().warning(
                "Gazebo rejected the red sphere pose",
                throttle_duration_sec=5.0,
            )


def _pose(position: Vec3) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.w = 1.0
    return pose


def main() -> None:
    rclpy.init()
    node = SphereMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
