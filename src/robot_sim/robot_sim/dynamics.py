"""Gravity torques and computed-torque feedforward + PD from a URDF arm chain."""

from collections.abc import Sequence

import PyKDL

from robot_sim.chain import chain_from_urdf

ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class ArmDynamics:
    """Inverse dynamics on the UR arm chain for feedforward + PD."""

    def __init__(
        self,
        urdf: str,
        base_link: str = "base_link",
        tip_link: str = "tool0",
        gravity: tuple[float, float, float] = (0.0, 0.0, -9.81),
    ) -> None:
        self._chain, names, _limits = chain_from_urdf(urdf, base_link, tip_link)
        if names != ARM_JOINTS:
            raise ValueError(f"expected joints {ARM_JOINTS}, got {names}")
        self.joint_names = names
        self._dyn = PyKDL.ChainDynParam(self._chain, PyKDL.Vector(*gravity))

    def compute_torque(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        desired: Sequence[float],
        desired_velocities: Sequence[float],
        kp: Sequence[float],
        kd: Sequence[float],
        effort_limits: Sequence[float],
    ) -> list[float]:
        """Return tau = g + C + M (Kp e + Kd (qdot_des - qdot)), clipped to limits.

        Kp and Kd are acceleration gains. Wrist inertia is about 1e-3 kg·m², so
        adding Kp e + Kd edot directly as torque makes the 500 Hz step unstable
        and the wrist torque changes sign every tick.
        """
        count = len(self.joint_names)
        joints = self._joint_array(positions)
        joint_vel = self._joint_array(velocities)

        gravity = PyKDL.JntArray(count)
        if self._dyn.JntToGravity(joints, gravity) < 0:
            raise RuntimeError("gravity torques failed")

        coriolis = PyKDL.JntArray(count)
        if self._dyn.JntToCoriolis(joints, joint_vel, coriolis) < 0:
            raise RuntimeError("coriolis torques failed")

        mass = PyKDL.JntSpaceInertiaMatrix(count)
        if self._dyn.JntToMass(joints, mass) < 0:
            raise RuntimeError("inertia matrix failed")

        accelerations = [
            float(kp[index]) * (float(desired[index]) - float(positions[index]))
            + float(kd[index])
            * (float(desired_velocities[index]) - float(velocities[index]))
            for index in range(count)
        ]
        torques: list[float] = []
        for index in range(count):
            inertial = sum(
                mass[index, column] * accelerations[column] for column in range(count)
            )
            torque = float(gravity[index]) + float(coriolis[index]) + inertial
            limit = float(effort_limits[index])
            torques.append(max(-limit, min(limit, torque)))
        return torques

    def _joint_array(self, values: Sequence[float]) -> PyKDL.JntArray:
        if len(values) != len(self.joint_names):
            raise ValueError(
                f"expected {len(self.joint_names)} joints, got {len(values)}"
            )
        array = PyKDL.JntArray(len(values))
        for index, value in enumerate(values):
            array[index] = float(value)
        return array
