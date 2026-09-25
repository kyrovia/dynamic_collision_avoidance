#!/usr/bin/env python3
"""Computed-torque control: dynamics feedforward + joint PD for APF trajectories."""

import math
import sys

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclTime
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

from robot_sim.dynamics import ARM_JOINTS, ArmDynamics


def _time_to_sec(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _wrap_joint_delta(delta: float) -> float:
    """Shortest signed angle between two revolute joint readings."""
    return (delta + math.pi) % (2.0 * math.pi) - math.pi


class ComputedTorqueNode(Node):
    def __init__(self) -> None:
        super().__init__("computed_torque_node")
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "tool0")
        self.declare_parameter("control_rate", 500.0)
        self.declare_parameter(
            "trajectory_topic", "/joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter("effort_topic", "/effort_controller/commands")
        self.declare_parameter("kp", [800.0, 800.0, 400.0, 200.0, 200.0, 200.0])
        self.declare_parameter("kd", [40.0, 40.0, 20.0, 10.0, 10.0, 10.0])
        self.declare_parameter("effort_limits", [330.0, 330.0, 150.0, 54.0, 54.0, 54.0])
        self.declare_parameter("default_trajectory_horizon", 0.1)

        try:
            self._kp = self._vector_param("kp")
            self._kd = self._vector_param("kd")
            self._effort_limits = self._vector_param("effort_limits")
        except ValueError as exc:
            self.get_logger().fatal(str(exc))
            raise

        self._default_horizon = float(
            self.get_parameter("default_trajectory_horizon").value
        )
        self._dynamics: ArmDynamics | None = None
        self._positions: dict[str, float] = {}
        self._velocities: dict[str, float] = {}
        self._desired: list[float] | None = None
        self._desired_velocities: list[float] | None = None
        self._prev_desired: list[float] | None = None
        self._prev_trajectory_stamp: RclTime | None = None
        self._prev_measured: list[float] | None = None
        self._prev_measured_time: RclTime | None = None

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/robot_description", self._on_description, latched)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.create_subscription(JointTrajectory, trajectory_topic, self._on_trajectory, 10)

        effort_topic = str(self.get_parameter("effort_topic").value)
        self._effort = self.create_publisher(Float64MultiArray, effort_topic, 10)

        rate = float(self.get_parameter("control_rate").value)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f"computed torque listening on {trajectory_topic}, publishing to {effort_topic}"
        )

    def _on_description(self, message: String) -> None:
        if self._dynamics is not None:
            return
        base = str(self.get_parameter("base_link").value)
        tip = str(self.get_parameter("tip_link").value)
        try:
            self._dynamics = ArmDynamics(message.data, base, tip)
        except (RuntimeError, ValueError) as exc:
            self.get_logger().error(f"failed to build dynamics: {exc}")
            return
        self.get_logger().info("arm dynamics ready")

    def _on_joint_states(self, message: JointState) -> None:
        try:
            self._positions = dict(zip(message.name, message.position))
            if message.velocity:
                self._velocities = dict(zip(message.name, message.velocity))
            else:
                self._velocities = dict.fromkeys(message.name, 0.0)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f"invalid joint state: {exc}", throttle_duration_sec=2.0
            )

    def _on_trajectory(self, message: JointTrajectory) -> None:
        try:
            if not message.points:
                return
            latest = message.points[-1]
            if len(latest.positions) != len(message.joint_names):
                self.get_logger().warning(
                    "trajectory point size does not match joint_names",
                    throttle_duration_sec=5.0,
                )
                return

            ordered = dict(zip(message.joint_names, latest.positions))
            desired = [float(ordered[name]) for name in ARM_JOINTS]
            desired_vel = self._desired_velocities_from_point(
                message, latest, desired
            )

            self._desired = desired
            self._desired_velocities = desired_vel
            self._prev_desired = desired
            self._prev_trajectory_stamp = RclTime.from_msg(message.header.stamp)
        except KeyError:
            self.get_logger().warning(
                f"trajectory joints {message.joint_names} do not match {ARM_JOINTS}",
                throttle_duration_sec=5.0,
            )
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f"invalid trajectory: {exc}", throttle_duration_sec=2.0
            )

    def _desired_velocities_from_point(
        self,
        message: JointTrajectory,
        point,
        desired: list[float],
    ) -> list[float]:
        if (
            point.velocities
            and len(point.velocities) == len(message.joint_names)
        ):
            ordered = dict(zip(message.joint_names, point.velocities))
            return [float(ordered[name]) for name in ARM_JOINTS]

        horizon = _time_to_sec(point.time_from_start)
        if horizon <= 0.0:
            horizon = self._default_horizon

        if self._prev_desired is not None and self._prev_trajectory_stamp is not None:
            stamp = RclTime.from_msg(message.header.stamp)
            dt = (stamp - self._prev_trajectory_stamp).nanoseconds * 1e-9
            if dt > 1e-6:
                return [
                    (current - previous) / dt
                    for current, previous in zip(desired, self._prev_desired)
                ]

        if all(name in self._positions for name in ARM_JOINTS):
            return [
                (target - self._positions[name]) / horizon
                for name, target in zip(ARM_JOINTS, desired)
            ]

        return [0.0] * len(ARM_JOINTS)

    def _on_timer(self) -> None:
        if self._dynamics is None:
            return
        if any(name not in self._positions for name in ARM_JOINTS):
            return

        positions = [self._positions[name] for name in ARM_JOINTS]
        velocities = self._measured_velocities(positions)
        desired = self._desired if self._desired is not None else positions
        desired_velocities = (
            self._desired_velocities
            if self._desired_velocities is not None
            else [0.0] * len(ARM_JOINTS)
        )

        try:
            torques = self._dynamics.compute_torque(
                positions,
                velocities,
                desired,
                desired_velocities,
                self._kp,
                self._kd,
                self._effort_limits,
            )
        except (RuntimeError, ValueError) as exc:
            self.get_logger().error(
                f"torque computation failed: {exc}", throttle_duration_sec=2.0
            )
            self._publish_effort([0.0] * len(ARM_JOINTS))
            return

        self._publish_effort(torques)

    def _measured_velocities(self, positions: list[float]) -> list[float]:
        """Differentiate wrapped joint positions; Gazebo /joint_states velocity lies near ±pi."""
        now = self.get_clock().now()
        if (
            self._prev_measured is not None
            and self._prev_measured_time is not None
        ):
            dt = (now - self._prev_measured_time).nanoseconds * 1e-9
            if dt > 1e-6:
                velocities = [
                    _wrap_joint_delta(current - previous) / dt
                    for current, previous in zip(positions, self._prev_measured)
                ]
            else:
                velocities = [0.0] * len(ARM_JOINTS)
        else:
            velocities = [self._velocities.get(name, 0.0) for name in ARM_JOINTS]

        self._prev_measured = list(positions)
        self._prev_measured_time = now
        return velocities

    def _publish_effort(self, torques: list[float]) -> None:
        message = Float64MultiArray()
        message.data = torques
        self._effort.publish(message)

    def _vector_param(self, name: str) -> list[float]:
        values = list(self.get_parameter(name).value)
        if len(values) != len(ARM_JOINTS):
            raise ValueError(f"{name} must have {len(ARM_JOINTS)} elements")
        return [float(value) for value in values]


def main() -> None:
    rclpy.init()
    node = ComputedTorqueNode()
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
