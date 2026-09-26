"""Sphere clearance, CHOMP joint-space repulsion, and a UR16e straight-arm gap."""

import math

import numpy as np

from robot_avoidance_apf.apf import SphereObstacle
from robot_avoidance_apf.capsules import (
    AvoidanceCommand,
    _potential_slope,
    avoidance_velocity,
    capsule_draws,
    link_radius,
)
from robot_avoidance_apf.kinematics import ArmKinematics, BodySegment, JointAxis

# Offsets from ur_description ur16e default_kinematics.yaml.
UR16E = """
<robot name="ur16e">
  <link name="base_link"/>
  <link name="base_link_inertia"/>
  <joint name="base_inertia" type="fixed">
    <parent link="base_link"/><child link="base_link_inertia"/>
    <origin xyz="0 0 0" rpy="0 0 3.141592653589793"/>
  </joint>
  <link name="shoulder_link"/>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link_inertia"/><child link="shoulder_link"/>
    <origin xyz="0 0 0.1807" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="1" velocity="1"/>
  </joint>
  <link name="upper_arm_link"/>
  <joint name="shoulder_lift_joint" type="revolute">
    <parent link="shoulder_link"/><child link="upper_arm_link"/>
    <origin xyz="0 0 0" rpy="1.570796327 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="1" velocity="1"/>
  </joint>
  <link name="forearm_link"/>
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/><child link="forearm_link"/>
    <origin xyz="-0.4784 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="1" velocity="1"/>
  </joint>
  <link name="wrist_1_link"/>
  <joint name="wrist_1_joint" type="revolute">
    <parent link="forearm_link"/><child link="wrist_1_link"/>
    <origin xyz="-0.36 0 0.17415" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="1" velocity="1"/>
  </joint>
  <link name="wrist_2_link"/>
  <joint name="wrist_2_joint" type="revolute">
    <parent link="wrist_1_link"/><child link="wrist_2_link"/>
    <origin xyz="0 -0.11985 0" rpy="1.570796327 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="1" velocity="1"/>
  </joint>
  <link name="wrist_3_link"/>
  <joint name="wrist_3_joint" type="revolute">
    <parent link="wrist_2_link"/><child link="wrist_3_link"/>
    <origin xyz="0 0.11655 0"
      rpy="1.570796326589793 3.141592653589793 3.141592653589793"/>
    <axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="1" velocity="1"/>
  </joint>
  <link name="flange"/>
  <joint name="wrist_3_flange" type="fixed">
    <parent link="wrist_3_link"/><child link="flange"/>
    <origin xyz="0 0 0" rpy="0 -1.57079632679 -1.57079632679"/>
  </joint>
  <link name="tool0"/>
  <joint name="flange_tool0" type="fixed">
    <parent link="flange"/><child link="tool0"/>
    <origin xyz="0 0 0" rpy="1.57079632679 0 1.57079632679"/>
  </joint>
</robot>
"""

FOLD = """
<robot name="fold">
  <link name="base"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="1" velocity="1"/>
  </joint>
  <link name="link2"/>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="1" velocity="1"/>
  </joint>
  <link name="link3"/>
  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-6.28" upper="6.28" effort="1" velocity="1"/>
  </joint>
  <link name="tool"/>
  <joint name="tool_fixed" type="fixed">
    <parent link="link3"/><child link="tool"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""

# Controller defaults. Self influence must stay inside the straight wrist gap.
ARM_RADIUS = 0.05
WRIST_RADIUS = 0.03
BASE_RADIUS = 0.08
SELF_D0 = 0.03
READY = (0.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, 0.0, 0.0)


def test_parallel_segments_report_the_separating_distance() -> None:
    segments = (
        BodySegment("a", (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), 1),
        BodySegment("mid", (0.0, 10.0, 0.0), (1.0, 10.0, 0.0), 1),
        BodySegment("b", (0.5, 0.4, 0.0), (0.5, 1.0, 0.0), 1),
    )
    command = _avoid(segments, _one_axis(), (0.0, 0.0, 0.0), radii=(0.0, 0.0, 0.0))
    assert abs(command.self_clearance - 0.4) < 1e-6


def test_sphere_clearance_and_push_use_the_closest_point() -> None:
    segment = BodySegment("arm", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 1)
    command = _avoid(
        (segment,),
        _one_axis(),
        (0.5, 0.2, 0.0),
        radii=(0.05,),
        obstacle_radius=0.05,
        k_rep=0.02,
        d0=0.2,
        d_min=0.01,
    )
    assert abs(command.obstacle_clearance - 0.1) < 1e-6
    assert command.hold is False
    assert command.velocity[0] < 0.0


def test_clearance_inside_stop_distance_holds() -> None:
    segment = BodySegment("arm", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 1)
    command = _avoid(
        (segment,),
        _one_axis(),
        (0.5, 0.04, 0.0),
        radii=(0.02,),
        obstacle_radius=0.02,
        d_min=0.03,
    )
    assert command.obstacle_clearance < 0.03
    assert command.hold is True
    assert command.velocity == (0.0,)


def test_fixed_base_does_not_freeze_the_arm_for_a_distant_sphere() -> None:
    segments = (
        BodySegment("base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.2), 0),
        BodySegment("arm", (0.0, 0.0, 0.2), (0.5, 0.0, 0.2), 1),
    )
    command = _avoid(
        segments,
        _one_axis((0.0, 0.0, 0.2)),
        (0.0, 0.02, 0.05),
        radii=(0.05, 0.05),
        obstacle_radius=0.02,
        d_min=0.02,
    )
    assert command.hold is False
    assert command.obstacle_clearance > 0.02


def test_obstacle_step_increases_link_clearance() -> None:
    arm = ArmKinematics(FOLD, "base", "tool")
    positions = [0.4, -0.3, 0.2]
    segments, _ = arm.body(positions)
    link = next(segment for segment in segments if segment.name == "link2")
    start = np.array(link.start)
    end = np.array(link.end)
    direction = end - start
    side = np.cross(direction, np.array([0.0, 0.0, 1.0]))
    side = side / np.linalg.norm(side)
    center = 0.5 * (start + end) + side * 0.12
    obstacle = (float(center[0]), float(center[1]), float(center[2]))
    before = _scene(arm, positions, obstacle)
    assert before.hold is False
    assert any(abs(item) > 1e-4 for item in before.velocity)
    stepped = [pos + vel * 0.05 for pos, vel in zip(positions, before.velocity)]
    after = _scene(arm, stepped, obstacle)
    assert after.obstacle_clearance > before.obstacle_clearance


def test_self_step_increases_nonadjacent_clearance() -> None:
    arm = ArmKinematics(FOLD, "base", "tool")
    found = False
    for angle in (index * 0.12 for index in range(1, 26)):
        positions = [0.0, 0.0, angle]
        before = _scene(arm, positions, (10.0, 0.0, 0.0), k_rep=0.0, self_d0=0.08)
        if before.hold or before.self_clearance >= 0.08:
            continue
        if all(abs(item) < 1e-5 for item in before.velocity):
            continue
        stepped = [pos + vel * 0.05 for pos, vel in zip(positions, before.velocity)]
        after = _scene(arm, stepped, (10.0, 0.0, 0.0), k_rep=0.0, self_d0=0.08)
        assert after.self_clearance > before.self_clearance
        found = True
        break
    assert found


def test_upper_arm_midpoint_matches_the_joint_jacobian() -> None:
    arm = ArmKinematics(UR16E, "base_link", "tool0")
    positions = [0.2, -0.4, 0.3, -0.5, 0.1, -0.2]
    segments, axes = arm.body(positions)
    upper = next(segment for segment in segments if segment.name == "upper_arm_link")
    assert upper.joint_count == 2
    point = 0.5 * (np.array(upper.start) + np.array(upper.end))
    delta = 1e-6
    for index, position in enumerate(positions):
        bumped = list(positions)
        bumped[index] = position + delta
        bumped_segments, _ = arm.body(bumped)
        bumped_upper = next(
            segment for segment in bumped_segments if segment.name == "upper_arm_link"
        )
        bumped_point = 0.5 * (np.array(bumped_upper.start) + np.array(bumped_upper.end))
        numerical = (bumped_point - point) / delta
        origin = np.array(axes[index].origin)
        direction = np.array(axes[index].direction)
        if index < upper.joint_count:
            analytic = np.cross(direction, point - origin)
            assert np.allclose(numerical, analytic, atol=1e-4)
        else:
            assert np.linalg.norm(numerical) < 1e-4


def test_ready_pose_self_gap_stays_outside_the_influence_radius() -> None:
    arm = ArmKinematics(UR16E, "base_link", "tool0")
    command = _scene(arm, list(READY), (0.35, 0.21, 0.63), obstacle_radius=0.0315)
    assert command.self_clearance > SELF_D0, command.self_clearance
    names = [segment.name for segment in arm.body(READY)[0]]
    assert names == [
        "base_link_inertia",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
    ]


def test_capsule_draw_aligns_the_cylinder_with_the_segment() -> None:
    along_z = capsule_draws(
        (BodySegment("z", (0.0, 0.0, 0.0), (0.0, 0.0, 0.4), 1),), (0.05,)
    )[0]
    assert along_z.center == (0.0, 0.0, 0.2)
    assert along_z.length == 0.4
    assert along_z.radius == 0.05
    assert along_z.orientation == (0.0, 0.0, 0.0, 1.0)

    along_x = capsule_draws(
        (BodySegment("x", (0.0, 0.0, 0.0), (0.8, 0.0, 0.0), 1),), (0.03,)
    )[0]
    assert along_x.center == (0.4, 0.0, 0.0)
    assert abs(along_x.orientation[1]) > 0.5
    assert abs(along_x.orientation[3]) > 0.5


def test_chomp_slope_is_zero_outside_and_capped_inside() -> None:
    assert _potential_slope(0.03, 0.03) == 0.0
    assert _potential_slope(0.031, 0.03) == 0.0
    # Halfway through the influence distance the slope is one half, not a spike.
    assert abs(_potential_slope(0.015, 0.03) - 0.5) < 1e-9
    assert _potential_slope(0.0, 0.03) == 1.0
    assert _potential_slope(-0.01, 0.03) == 1.0


def test_link_radius_follows_the_link_name() -> None:
    assert link_radius("forearm_link", 0.05, 0.03, 0.08) == 0.05
    assert link_radius("wrist_2_link", 0.05, 0.03, 0.08) == 0.03
    assert link_radius("base_link_inertia", 0.05, 0.03, 0.08) == 0.08


def _scene(
    arm: ArmKinematics,
    positions: list[float],
    obstacle: tuple[float, float, float],
    *,
    obstacle_radius: float = 0.05,
    k_rep: float = 0.05,
    self_d0: float = SELF_D0,
) -> AvoidanceCommand:
    segments, axes = arm.body(positions)
    radii = [
        link_radius(segment.name, ARM_RADIUS, WRIST_RADIUS, BASE_RADIUS)
        for segment in segments
    ]
    return avoidance_velocity(
        segments,
        axes,
        SphereObstacle(obstacle, obstacle_radius),
        radii,
        k_rep=k_rep,
        d0=0.2,
        d_min=0.01,
        k_self=0.05,
        self_d0=self_d0,
        self_d_min=0.0,
        qdot_max=1.0,
    )


def _avoid(
    segments: tuple[BodySegment, ...],
    axes: tuple[JointAxis, ...],
    obstacle: tuple[float, float, float],
    *,
    radii: tuple[float, ...],
    obstacle_radius: float = 0.05,
    k_rep: float = 0.0,
    d0: float = 0.2,
    d_min: float = 0.01,
) -> AvoidanceCommand:
    return avoidance_velocity(
        segments,
        axes,
        SphereObstacle(obstacle, obstacle_radius),
        radii,
        k_rep=k_rep,
        d0=d0,
        d_min=d_min,
        k_self=0.0,
        self_d0=1.0,
        self_d_min=0.0,
        qdot_max=1.0,
    )


def _one_axis(
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[JointAxis, ...]:
    return (JointAxis(origin, (0.0, 0.0, 1.0)),)
