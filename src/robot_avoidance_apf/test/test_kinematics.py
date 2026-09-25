"""PyKDL chain built from a minimal URDF, without a running robot."""

import math

from robot_avoidance_apf.kinematics import ArmKinematics, joint_limit_velocity

URDF = """
<robot name="arm">
  <link name="base"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="0" effort="1" velocity="1"/>
  </joint>
  <link name="tool"/>
  <joint name="tool_fixed" type="fixed">
    <parent link="link1"/>
    <child link="tool"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_zero_and_quarter_turn_match_the_joint_origin() -> None:
    arm = ArmKinematics(URDF, "base", "tool")
    assert arm.joint_names == ("joint1",)
    assert arm.limits == ((-1.0, 0.0),)

    position, _ = arm.pose([0.0])
    assert _close(position, (0.2, 0.0, 0.5))

    position, _ = arm.pose([math.pi / 2.0])
    assert _close(position, (0.0, 0.2, 0.5))


def test_joint_limit_stops_further_motion() -> None:
    arm = ArmKinematics(URDF, "base", "tool")
    # Positive y velocity rotates joint1 positive, which the upper limit forbids.
    held, _ = arm.integrate([0.0], [0.0, 0.5, 0.0, 0.0, 0.0, 0.0], dt=1.0, damping=0.05)
    assert held[0] == 0.0


def test_joint_limit_velocity_is_zero_away_from_bounds() -> None:
    velocity = joint_limit_velocity(
        [-0.5], [(-1.0, 0.0)], k_lim=0.01, rho=0.2, qdot_max=0.5
    )
    assert velocity[0] == 0.0


def test_joint_limit_velocity_pushes_away_from_each_bound() -> None:
    upper = joint_limit_velocity(
        [-0.05], [(-1.0, 0.0)], k_lim=0.01, rho=0.2, qdot_max=0.5
    )
    lower = joint_limit_velocity(
        [-0.95], [(-1.0, 0.0)], k_lim=0.01, rho=0.2, qdot_max=0.5
    )
    assert upper[0] < 0.0
    assert lower[0] > 0.0
    assert abs(upper[0]) <= 0.5
    assert abs(lower[0]) <= 0.5


def test_integrate_joint_limit_field_leaves_the_bound() -> None:
    arm = ArmKinematics(URDF, "base", "tool")
    stepped, velocities = arm.integrate(
        [0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        dt=0.1,
        damping=0.05,
        k_lim=0.01,
        rho_lim=0.2,
        qdot_lim=0.5,
    )
    assert stepped[0] < 0.0
    assert stepped[0] >= -1.0
    assert velocities[0] < 0.0


def _close(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(left - right) < 1e-6 for left, right in zip(actual, expected))
