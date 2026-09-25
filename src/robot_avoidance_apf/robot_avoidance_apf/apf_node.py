#!/usr/bin/env python3
"""Stream a joint trajectory that follows a static-obstacle potential field."""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_avoidance_apf.apf import (
    ApfParams,
    SphereObstacle,
    field_command,
    orientation_angle,
)
from robot_avoidance_apf.kinematics import ArmKinematics

ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


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


def load_goal_and_radius(
    path: Path,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float], float]:
    root = ET.parse(path).getroot()
    radius = root.find("./world/model[@name='red_sphere']//sphere/radius")
    target = root.find("./world/model[@name='target_pose']/pose")
    if radius is None or target is None or not target.text:
        raise ValueError(f"{path} is missing red_sphere radius or target_pose")
    values = [float(item) for item in target.text.split()]
    position = (values[0], values[1], values[2])
    orientation = quaternion_from_rpy(values[3], values[4], values[5])
    return position, orientation, float(radius.text)


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """SDF roll-pitch-yaw to a ROS xyzw quaternion."""
    half_roll, half_pitch, half_yaw = roll * 0.5, pitch * 0.5, yaw * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def duration_from_seconds(seconds: float) -> Duration:
    whole = int(seconds)
    nanos = int(round((seconds - whole) * 1e9))
    if nanos >= 1_000_000_000:
        whole += 1
        nanos -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanos)


class ApfNode(Node):
    def __init__(self) -> None:
        super().__init__("apf_node")
        self.declare_parameter("world", str(world_path()))
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "tool0")
        self.declare_parameter("k_att", 1.0)
        self.declare_parameter("k_rep", 0.02)
        self.declare_parameter("k_tan", 0.04)
        self.declare_parameter("k_ori", 1.0)
        self.declare_parameter("d0", 0.20)
        self.declare_parameter("d_min", 0.03)
        self.declare_parameter("v_max", 0.08)
        self.declare_parameter("omega_max", 0.5)
        self.declare_parameter("damping", 0.05)
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("trajectory_horizon", 0.1)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("orientation_tolerance", 0.05)

        goal_position, goal_orientation, radius = load_goal_and_radius(
            Path(self.get_parameter("world").get_parameter_value().string_value)
        )
        self._goal_position = goal_position
        self._goal_orientation = goal_orientation
        self._radius = radius
        self._kinematics: ArmKinematics | None = None
        self._joints: dict[str, float] = {}
        self._obstacle: tuple[float, float, float] | None = None
        self._arrived = False

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._trajectory = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self.create_subscription(String, "/robot_description", self._on_description, latched)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(PoseStamped, "/obstacle/pose", self._on_obstacle, 10)

        rate = float(self.get_parameter("control_rate").value)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f"goal [{goal_position[0]:.3f}, {goal_position[1]:.3f}, "
            f"{goal_position[2]:.3f}], sphere radius {radius:.3f} m"
        )

    def _on_description(self, message: String) -> None:
        if self._kinematics is not None:
            return
        base = str(self.get_parameter("base_link").value)
        tip = str(self.get_parameter("tip_link").value)
        try:
            self._kinematics = ArmKinematics(message.data, base, tip)
        except (RuntimeError, ValueError) as exc:
            self.get_logger().error(f"failed to build kinematics: {exc}")
            return
        names = self._kinematics.joint_names
        if names != ARM_JOINTS:
            self.get_logger().warning(f"URDF joint order is {', '.join(names)}")
        self.get_logger().info(f"kinematics ready for {', '.join(names)}")

    def _on_joint_states(self, message: JointState) -> None:
        self._joints = dict(zip(message.name, message.position))

    def _on_obstacle(self, message: PoseStamped) -> None:
        frame = str(self.get_parameter("base_link").value)
        if message.header.frame_id and message.header.frame_id != frame:
            self.get_logger().warning(
                f"ignoring obstacle pose in {message.header.frame_id}; expected {frame}",
                throttle_duration_sec=5.0,
            )
            return
        point = message.pose.position
        self._obstacle = (point.x, point.y, point.z)

    def _on_timer(self) -> None:
        if self._arrived:
            return
        positions = self._arm_positions()
        if positions is None or self._kinematics is None or self._obstacle is None:
            self.get_logger().info(
                "waiting for robot description, joint states, and obstacle pose",
                throttle_duration_sec=5.0,
            )
            return

        pose, orientation = self._kinematics.pose(positions)
        command = field_command(
            pose,
            orientation,
            self._goal_position,
            self._goal_orientation,
            SphereObstacle(self._obstacle, self._radius),
            self._params(),
        )
        if command.hold:
            self.get_logger().error(
                f"tool0 clearance {command.clearance:.3f} m is inside the stop distance",
                throttle_duration_sec=1.0,
            )
            self._publish(positions)
            return
        if self._at_goal(pose, orientation):
            self._publish(positions)
            self._arrived = True
            self._timer.cancel()
            self.get_logger().info("tool0 reached target_pose")
            return

        horizon = float(self.get_parameter("trajectory_horizon").value)
        damping = float(self.get_parameter("damping").value)
        # Step across the trajectory horizon so the arm tracks field speed.
        # A single control tick stretched over that horizon would move too slowly.
        commanded, joint_velocities = self._kinematics.integrate(
            positions,
            (*command.linear, *command.angular),
            horizon,
            damping,
        )
        self._publish(commanded, joint_velocities)
        speed = math.sqrt(sum(value * value for value in command.linear))
        pos_err, ori_err = self._goal_errors(pose, orientation)
        self.get_logger().info(
            f"clearance {command.clearance:.3f} m, speed {speed:.3f} m/s, "
            f"pos_err {pos_err:.4f} m, ori_err {ori_err:.4f} rad",
            throttle_duration_sec=2.0,
        )

    def _arm_positions(self) -> list[float] | None:
        if self._kinematics is None:
            return None
        if any(name not in self._joints for name in self._kinematics.joint_names):
            return None
        return [self._joints[name] for name in self._kinematics.joint_names]

    def _params(self) -> ApfParams:
        return ApfParams(
            k_att=float(self.get_parameter("k_att").value),
            k_rep=float(self.get_parameter("k_rep").value),
            k_tan=float(self.get_parameter("k_tan").value),
            k_ori=float(self.get_parameter("k_ori").value),
            d0=float(self.get_parameter("d0").value),
            d_min=float(self.get_parameter("d_min").value),
            v_max=float(self.get_parameter("v_max").value),
            omega_max=float(self.get_parameter("omega_max").value),
        )

    def _goal_errors(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
    ) -> tuple[float, float]:
        offset = (
            position[0] - self._goal_position[0],
            position[1] - self._goal_position[1],
            position[2] - self._goal_position[2],
        )
        pos_err = math.sqrt(offset[0] ** 2 + offset[1] ** 2 + offset[2] ** 2)
        ori_err = orientation_angle(orientation, self._goal_orientation)
        return pos_err, ori_err

    def _at_goal(
        self,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
    ) -> bool:
        pos_err, ori_err = self._goal_errors(position, orientation)
        if pos_err > float(self.get_parameter("position_tolerance").value):
            return False
        return ori_err <= float(self.get_parameter("orientation_tolerance").value)

    def _publish(
        self,
        positions: list[float],
        velocities: list[float] | None = None,
    ) -> None:
        if self._kinematics is None:
            return
        horizon = float(self.get_parameter("trajectory_horizon").value)
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(self._kinematics.joint_names)
        point = JointTrajectoryPoint()
        point.positions = positions
        if velocities is None:
            velocities = [0.0] * len(positions)
        point.velocities = velocities
        point.time_from_start = duration_from_seconds(horizon)
        message.points.append(point)
        self._trajectory.publish(message)


def main() -> None:
    rclpy.init()
    node = ApfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
