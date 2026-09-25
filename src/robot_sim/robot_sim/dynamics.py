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
        """Return tau = g + C + Kp e + Kd (qdot_des - qdot), clipped to limits."""
        count = len(self.joint_names)
        joints = self._joint_array(positions)
        joint_vel = self._joint_array(velocities)
        desired_vel = self._joint_array(desired_velocities)

        gravity = PyKDL.JntArray(count)
        if self._dyn.JntToGravity(joints, gravity) < 0:
            raise RuntimeError("gravity torques failed")

        coriolis = PyKDL.JntArray(count)
        if self._dyn.JntToCoriolis(joints, joint_vel, coriolis) < 0:
            raise RuntimeError("coriolis torques failed")

        torques: list[float] = []
        for index in range(count):
            error = float(desired[index]) - float(positions[index])
            velocity_error = float(desired_vel[index]) - float(velocities[index])
            torque = (
                float(gravity[index])
                + float(coriolis[index])
                + float(kp[index]) * error
                + float(kd[index]) * velocity_error
            )
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
