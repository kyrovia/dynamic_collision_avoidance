"""Computed-torque feedback scales with inertia, including a light wrist."""

import math

from robot_sim.dynamics import ArmDynamics

# Last link matches a UR wrist: about 1e-3 kg m^2. The first links are heavier.
_LIGHT = 0.001


def _urdf() -> str:
    links = [
        ("base", 1.0),
        ("shoulder_pan_link", 0.2),
        ("shoulder_lift_link", 0.2),
        ("forearm_link", 0.2),
        ("wrist_1_link", 0.2),
        ("wrist_2_link", 0.2),
        ("wrist_3_link", _LIGHT),
        ("tool0", 0.0),
    ]
    joints = [
        ("shoulder_pan_joint", "base", "shoulder_pan_link"),
        ("shoulder_lift_joint", "shoulder_pan_link", "shoulder_lift_link"),
        ("elbow_joint", "shoulder_lift_link", "forearm_link"),
        ("wrist_1_joint", "forearm_link", "wrist_1_link"),
        ("wrist_2_joint", "wrist_1_link", "wrist_2_link"),
        ("wrist_3_joint", "wrist_2_link", "wrist_3_link"),
    ]
    parts = ["<robot name='arm'>"]
    for name, inertia in links:
        parts.append(
            f"""
            <link name="{name}">
              <inertial>
                <mass value="1"/>
                <inertia ixx="{inertia}" ixy="0" ixz="0"
                         iyy="{inertia}" iyz="0" izz="{inertia}"/>
              </inertial>
            </link>
            """
        )
    for name, parent, child in joints:
        parts.append(
            f"""
            <joint name="{name}" type="revolute">
              <parent link="{parent}"/>
              <child link="{child}"/>
              <origin xyz="0 0 0.1" rpy="0 0 0"/>
              <axis xyz="0 0 1"/>
              <limit lower="-6" upper="6" effort="100" velocity="3"/>
            </joint>
            """
        )
    parts.append(
        """
        <joint name="flange" type="fixed">
          <parent link="wrist_3_link"/>
          <child link="tool0"/>
          <origin xyz="0 0 0.05" rpy="0 0 0"/>
        </joint>
        </robot>
        """
    )
    return "".join(parts)


def _arm() -> ArmDynamics:
    return ArmDynamics(_urdf(), "base", "tool0", gravity=(0.0, 0.0, 0.0))


def test_light_wrist_torque_scales_with_inertia() -> None:
    arm = _arm()
    zeros = [0.0] * 6
    error = 0.1
    kp = [100.0] * 6
    desired = zeros[:]
    desired[5] = error
    torque = arm.compute_torque(
        zeros,
        zeros,
        desired,
        zeros,
        kp,
        [0.0] * 6,
        [100.0] * 6,
    )
    # tau = I * Kp * e. Treating Kp*e as torque would be 10 N·m and saturates the wrist.
    assert math.isclose(torque[5], _LIGHT * kp[5] * error, rel_tol=1e-6, abs_tol=1e-9)
    assert abs(torque[5]) < 1.0


def test_false_pi_wrist_speed_does_not_saturate() -> None:
    arm = _arm()
    zeros = [0.0] * 6
    measured = zeros[:]
    measured[5] = math.pi
    kd = 10.0
    torque = arm.compute_torque(
        zeros,
        measured,
        zeros,
        zeros,
        [0.0] * 6,
        [0.0, 0.0, 0.0, 0.0, 0.0, kd],
        [54.0] * 6,
    )
    assert math.isclose(torque[5], _LIGHT * kd * (-math.pi), rel_tol=1e-6, abs_tol=1e-9)
    assert abs(torque[5]) < 1.0
