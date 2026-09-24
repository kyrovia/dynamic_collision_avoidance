#!/usr/bin/env python3
"""Plan and execute to the scene target pose with MoveIt."""

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
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler

PLANNING_GROUP = "ur_manipulator"
EEF_LINK = "tool0"
PLANNING_FRAME = "base_link"
MOVE_ACTION = "move_action"


@dataclass(frozen=True)
class Sphere:
    center: tuple[float, float, float]
    radius: float


@dataclass(frozen=True)
class TargetPose:
    position: tuple[float, float, float]
    rpy: tuple[float, float, float]


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

    def parse_pose(text: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        values = [float(item) for item in text.split()]
        return (values[0], values[1], values[2]), (values[3], values[4], values[5])

    center, _ = parse_pose(sphere_pose.text or "")
    position, rpy = parse_pose(target.text or "")
    return Sphere(center, float(radius.text)), TargetPose(position, rpy)


def pose_from_rpy(position: tuple[float, float, float], rpy: tuple[float, float, float]) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    quat = quaternion_from_euler(*rpy)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


def pose_constraints(pose: Pose, frame_id: str, link_name: str) -> Constraints:
    constraints = Constraints()

    position = PositionConstraint()
    position.header.frame_id = frame_id
    position.link_name = link_name
    position.weight = 1.0
    region = BoundingVolume()
    ball = SolidPrimitive()
    ball.type = SolidPrimitive.SPHERE
    ball.dimensions = [1e-3]
    region.primitives.append(ball)
    region.primitive_poses.append(pose)
    position.constraint_region = region
    constraints.position_constraints.append(position)

    orientation = OrientationConstraint()
    orientation.header.frame_id = frame_id
    orientation.link_name = link_name
    orientation.orientation = pose.orientation
    orientation.absolute_x_axis_tolerance = 0.02
    orientation.absolute_y_axis_tolerance = 0.02
    orientation.absolute_z_axis_tolerance = 0.02
    orientation.weight = 1.0
    constraints.orientation_constraints.append(orientation)
    return constraints


class ReachabilityNode(Node):
    def __init__(self) -> None:
        super().__init__("reachability_node")
        self.declare_parameter("world", str(world_path()))
        self.declare_parameter("planning_group", PLANNING_GROUP)
        self.declare_parameter("eef_link", EEF_LINK)
        self.declare_parameter("planning_frame", PLANNING_FRAME)
        self.declare_parameter("allowed_planning_time", 10.0)
        self._move = ActionClient(self, MoveGroup, MOVE_ACTION)
        self._scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def run(self) -> bool:
        world = Path(self.get_parameter("world").get_parameter_value().string_value)
        sphere, target = load_scene(world)
        pose = pose_from_rpy(target.position, target.rpy)

        if not self._scene.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("ApplyPlanningScene is not available")
            return False
        if not self._move.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("MoveGroup action is not available")
            return False

        if not self._add_sphere(sphere):
            return False
        return self._go_to_pose(pose)

    def _add_sphere(self, sphere: Sphere) -> bool:
        object_msg = CollisionObject()
        object_msg.header.frame_id = self.get_parameter("planning_frame").value
        object_msg.id = "red_sphere"
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [sphere.radius]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = sphere.center
        pose.orientation.w = 1.0
        object_msg.primitives.append(primitive)
        object_msg.primitive_poses.append(pose)
        object_msg.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(object_msg)
        future = self._scene.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error("failed to add red_sphere to the planning scene")
            return False
        self.get_logger().info("added red_sphere to the planning scene")
        return True

    def _go_to_pose(self, pose: Pose) -> bool:
        group = self.get_parameter("planning_group").value
        link = self.get_parameter("eef_link").value
        frame = self.get_parameter("planning_frame").value
        allowed = self.get_parameter("allowed_planning_time").value

        stamped = PoseStamped()
        stamped.header.frame_id = frame
        stamped.pose = pose

        request = MotionPlanRequest()
        request.group_name = group
        request.num_planning_attempts = 10
        request.allowed_planning_time = float(allowed)
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2
        request.goal_constraints.append(pose_constraints(pose, frame, link))

        options = PlanningOptions()
        options.plan_only = False
        options.replan = True
        options.replan_attempts = 5

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options

        self.get_logger().info(
            "planning to target_pose "
            f"[{pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}]"
        )
        send = self._move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MoveGroup rejected the goal")
            return False

        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self, done)
        result = done.result()
        if result is None:
            self.get_logger().error("MoveGroup returned no result")
            return False

        error_code = result.result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("MoveIt did not reach target_pose, error_code=%s", error_code)
            return False
        self.get_logger().info("reached target_pose with MoveIt")
        return True


def main() -> None:
    rclpy.init()
    node = ReachabilityNode()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
