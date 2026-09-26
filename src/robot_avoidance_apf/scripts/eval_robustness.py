#!/usr/bin/env python3
"""Batch robustness evaluation for the APF controller.

Kinematic closed loop, no Gazebo needed. Each case replays one
``apf_node._on_timer`` step: ``body -> avoidance_velocity ->
field_command -> integrate``, until success / hold / stuck / timeout.

Run (workspace must be sourced so xacro and PyKDL resolve)::

    python3 scripts/eval_robustness.py --cases 100 --seed 0 --mode position
    python3 scripts/eval_robustness.py --cases 100 --seed 0 --mode full
"""

import argparse
import csv
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from robot_avoidance_apf.apf import (
    ApfParams,
    SphereObstacle,
    field_command,
    orientation_angle,
)
from robot_avoidance_apf.capsules import (
    avoidance_velocity,
    link_radius,
)
from robot_avoidance_apf.kinematics import ArmKinematics

Quat = tuple[float, float, float, float]
Vec3 = tuple[float, float, float]

READY = (0.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, 0.0, 0.0)
FAR_OBSTACLE = (10.0, 0.0, 0.0)

# Cylindrical workspace that covers the UR16e reach without the floor.
R_MIN, R_MAX = 0.25, 0.85
Z_MIN, Z_MAX = 0.05, 1.00

# Below this tool speed the arm is considered not progressing.
STUCK_SPEED = 0.008
STUCK_WINDOW_S = 2.0
STUCK_IMPROVEMENT_M = 0.002


@dataclass
class EvalParams:
    """Subset of apf_node parameters used by the closed loop."""

    base_link: str = "base_link"
    tip_link: str = "tool0"
    k_att: float = 1.0
    k_rep: float = 0.15
    k_tan: float = 0.04
    k_ori: float = 1.0
    d0: float = 0.20
    d_min: float = 0.0
    v_max: float = 0.04
    omega_max: float = 0.3
    damping: float = 0.05
    horizon: float = 0.1
    pos_tol: float = 0.02
    ori_tol: float = 0.05
    k_lim: float = 0.005
    rho_lim: float = 0.30
    qdot_lim: float = 0.5
    link_avoidance: bool = False
    self_avoidance: bool = False
    k_self: float = 0.15
    self_d0: float = 0.03
    self_d_min: float = 0.005
    envelope_extra: float = 0.03
    capsule_radius: float = 0.05
    wrist_radius: float = 0.03
    base_radius: float = 0.08
    qdot_avoid: float = 0.5


@dataclass
class CaseResult:
    idx: int
    goal_pos: Vec3
    goal_quat: Quat
    outcome: str
    baseline: str
    steps: int
    time_s: float
    pos_err: float
    ori_err: float
    min_tool: float
    min_link: float
    min_self: float
    path_m: float


def load_eval_params(path: Path) -> EvalParams:
    """Read apf_params.yaml; missing keys fall back to node defaults."""
    raw: dict = {}
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    node = data.get("apf_node", data)
    raw = node.get("ros__parameters", node)
    known = {field for field in EvalParams.__dataclass_fields__}
    aliases = {
        "position_tolerance": "pos_tol",
        "orientation_tolerance": "ori_tol",
        "trajectory_horizon": "horizon",
        "link_envelope_extra": "envelope_extra",
    }
    kwargs: dict = {}
    for key, value in raw.items():
        name = aliases.get(key, key)
        if name in known:
            kwargs[name] = value
    return EvalParams(**kwargs)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    """SDF roll-pitch-yaw to xyzw quaternion (no rclpy dependency)."""
    hr, hp, hy = roll * 0.5, pitch * 0.5, yaw * 0.5
    cr, sr = math.cos(hr), math.sin(hr)
    cp, sp = math.cos(hp), math.sin(hp)
    cy, sy = math.cos(hy), math.sin(hy)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def load_world(path: Path) -> tuple[Vec3, float, Vec3, Quat]:
    """Obstacle center/radius and reference target from the SDF world."""
    root = ET.parse(path).getroot()

    def text(xpath: str) -> str:
        node = root.find(xpath)
        if node is None or not node.text:
            raise ValueError(f"{path} is missing {xpath}")
        return node.text

    sphere = [float(v) for v in text("./world/model[@name='red_sphere']/pose").split()]
    target = [float(v) for v in text("./world/model[@name='target_pose']/pose").split()]
    radius = float(text("./world/model[@name='red_sphere']//sphere/radius").split()[0])
    center = (sphere[0], sphere[1], sphere[2])
    pos = (target[0], target[1], target[2])
    return center, radius, pos, quaternion_from_rpy(target[3], target[4], target[5])


def resolve_urdf(explicit: str | None, src_dir: Path) -> str:
    """Use --urdf, else expand ur_sim.urdf.xacro via the sourced xacro."""
    if explicit:
        return Path(explicit).read_text()
    result = subprocess.run(
        [
            "xacro",
            str(src_dir / "robot_sim/urdf/ur_sim.urdf.xacro"),
            "name:=ur",
            "ur_type:=ur16e",
            "sim_ignition:=true",
            "simulation_controllers:=" + str(src_dir / "robot_sim/config/ur_controllers.yaml"),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0 or len(result.stdout) < 1000:
        raise RuntimeError(
            "xacro failed to expand the URDF (is the workspace sourced?). "
            + result.stderr.strip()[-500:]
        )
    return result.stdout


def random_quat(rng: np.random.Generator) -> Quat:
    """Uniform random quaternion."""
    vec = rng.normal(size=4)
    vec /= float(np.linalg.norm(vec))
    return (float(vec[0]), float(vec[1]), float(vec[2]), float(vec[3]))


def sample_goal(rng: np.random.Generator, mode: str, start_quat: Quat) -> tuple[Vec3, Quat]:
    theta = float(rng.uniform(-math.pi, math.pi))
    radius = math.sqrt(float(rng.uniform(R_MIN**2, R_MAX**2)))
    pos = (
        radius * math.cos(theta),
        radius * math.sin(theta),
        float(rng.uniform(Z_MIN, Z_MAX)),
    )
    quat = start_quat if mode == "position" else random_quat(rng)
    return pos, quat


def goal_errors(
    pos: Vec3, quat: Quat, goal_pos: Vec3, goal_quat: Quat
) -> tuple[float, float]:
    offset = (pos[0] - goal_pos[0], pos[1] - goal_pos[1], pos[2] - goal_pos[2])
    pos_err = math.sqrt(offset[0] ** 2 + offset[1] ** 2 + offset[2] ** 2)
    return pos_err, orientation_angle(quat, goal_quat)


def run_closed_loop(
    kin: ArmKinematics,
    params: EvalParams,
    start: list[float],
    goal_pos: Vec3,
    goal_quat: Quat,
    obstacle: SphereObstacle,
    timeout: float,
) -> tuple[str, int, float, float, float, float, float, float]:
    """Replay apf_node steps; return outcome and trajectory statistics."""
    dt = params.horizon
    max_steps = max(1, int(round(timeout / dt)))
    apf = ApfParams(
        k_att=params.k_att,
        k_rep=params.k_rep,
        k_tan=params.k_tan,
        k_ori=params.k_ori,
        d0=params.d0,
        d_min=params.d_min,
        v_max=params.v_max,
        omega_max=params.omega_max,
    )
    positions = list(start)
    min_tool, min_link, min_self = math.inf, math.inf, math.inf
    path_m = 0.0
    prev_pose: Vec3 | None = None
    best_err = math.inf
    slow_steps = 0
    slow_window = max(1, int(round(STUCK_WINDOW_S / dt)))
    pos_err, ori_err, speed = math.inf, math.inf, 0.0

    for step in range(1, max_steps + 1):
        segments, axes = kin.body(positions)
        radii = [
            link_radius(seg.name, params.capsule_radius, params.wrist_radius, params.base_radius)
            for seg in segments
        ]
        avoid = avoidance_velocity(
            segments,
            axes,
            obstacle,
            radii,
            k_rep=params.k_rep,
            d0=params.d0,
            d_min=params.d_min,
            k_self=params.k_self,
            self_d0=params.self_d0,
            self_d_min=params.self_d_min,
            qdot_max=params.qdot_avoid,
            self_avoidance=params.self_avoidance,
            link_envelope_extra=params.envelope_extra,
        )
        pose, quat = kin.pose(positions)
        cmd = field_command(pose, quat, goal_pos, goal_quat, obstacle, apf)
        min_tool = min(min_tool, cmd.clearance)
        min_link = min(min_link, avoid.obstacle_clearance)
        min_self = min(min_self, avoid.self_clearance)
        if prev_pose is not None:
            delta = (
                pose[0] - prev_pose[0],
                pose[1] - prev_pose[1],
                pose[2] - prev_pose[2],
            )
            path_m += math.sqrt(delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2)
        prev_pose = pose
        pos_err, ori_err = goal_errors(pose, quat, goal_pos, goal_quat)
        speed = math.sqrt(sum(v * v for v in cmd.linear))
        best_err = min(best_err, pos_err)

        # Same order as apf_node: avoidance hold wins over arrival.
        if avoid.hold:
            if avoid.obstacle_clearance <= params.d_min:
                return "obstacle_hold", step, step * dt, pos_err, ori_err, min_tool, min_link, min_self, path_m
            return "self_hold", step, step * dt, pos_err, ori_err, min_tool, min_link, min_self, path_m
        if cmd.hold:
            # Tool point inside d_min. apf_node currently ignores this
            # branch, so flag it separately instead of silently driving on.
            return "tool_hold", step, step * dt, pos_err, ori_err, min_tool, min_link, min_self, path_m
        if pos_err <= params.pos_tol and ori_err <= params.ori_tol:
            return "success", step, step * dt, pos_err, ori_err, min_tool, min_link, min_self, path_m

        if speed < STUCK_SPEED:
            slow_steps += 1
        else:
            slow_steps = 0
            best_err = pos_err
        if slow_steps >= slow_window and best_err - pos_err < STUCK_IMPROVEMENT_M:
            return "stuck", step, step * dt, pos_err, ori_err, min_tool, min_link, min_self, path_m

        commanded, _ = kin.integrate(
            positions,
            (*cmd.linear, *cmd.angular),
            dt,
            params.damping,
            k_lim=params.k_lim,
            rho_lim=params.rho_lim,
            qdot_lim=params.qdot_lim,
            extra_velocity=avoid.velocity if params.link_avoidance else None,
        )
        positions = commanded

    return "timeout", max_steps, timeout, pos_err, ori_err, min_tool, min_link, min_self, path_m


def target_inside_obstacle(
    goal_pos: Vec3, obstacle: SphereObstacle, margin: float
) -> bool:
    dist = math.sqrt(sum((g - c) ** 2 for g, c in zip(goal_pos, obstacle.center)))
    return dist - obstacle.radius < margin


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=int, default=100)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--mode", choices=("position", "full"), default="position")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--world", type=str, default="")
    parser.add_argument("--params", type=str, default="")
    parser.add_argument("--urdf", type=str, default="")
    parser.add_argument("--obstacle", type=str, default="")
    parser.add_argument("--csv", type=str, default="")
    parser.add_argument("--start", type=str, default="")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    src_dir = Path(__file__).resolve().parents[2]
    params_path = Path(args.params) if args.params else src_dir / "robot_avoidance_apf/config/apf_params.yaml"
    world_path = Path(args.world) if args.world else src_dir / "robot_sim/worlds/empty.sdf"
    params = load_eval_params(params_path)
    center, radius, _, _ = load_world(world_path)
    if args.obstacle:
        parts = [float(v) for v in args.obstacle.split(",")]
        center, radius = (parts[0], parts[1], parts[2]), parts[3]
    obstacle = SphereObstacle(center, radius)

    kin = ArmKinematics(resolve_urdf(args.urdf or None, src_dir), params.base_link, params.tip_link)
    if args.start:
        start = [float(v) for v in args.start.split(",")]
    else:
        start = list(READY)
    _, start_quat = kin.pose(start)

    rng = np.random.default_rng(args.seed)
    # Goals that start inside the inflated obstacle can never count as
    # reachable; margin matches the stop distance plus the tool envelope.
    invalid_margin = params.d_min + 0.05
    results: list[CaseResult] = []
    invalid = 0
    for idx in range(args.cases):
        goal_pos, goal_quat = sample_goal(rng, args.mode, start_quat)
        if target_inside_obstacle(goal_pos, obstacle, invalid_margin):
            invalid += 1
            results.append(
                CaseResult(idx, goal_pos, goal_quat, "invalid_target", "invalid_target",
                           0, 0.0, math.inf, math.inf, math.inf, math.inf, math.inf, 0.0)
            )
            continue
        outcome, steps, time_s, pos_err, ori_err, min_tool, min_link, min_self, path_m = run_closed_loop(
            kin, params, start, goal_pos, goal_quat, obstacle, args.timeout
        )
        baseline = "-"
        if outcome != "success":
            # Same target without the obstacle: separates APF failures
            # (local minima, holds) from targets APF can never reach.
            free = SphereObstacle(FAR_OBSTACLE, radius)
            baseline, *_ = run_closed_loop(kin, params, start, goal_pos, goal_quat, free, args.timeout)
        results.append(
            CaseResult(idx, goal_pos, goal_quat, outcome, baseline,
                       steps, time_s, pos_err, ori_err, min_tool, min_link, min_self, path_m)
        )

    valid = [r for r in results if r.outcome != "invalid_target"]
    counts: dict[str, int] = {}
    for result in valid:
        counts[result.outcome] = counts.get(result.outcome, 0) + 1
    n_valid = max(1, len(valid))
    n_success = counts.get("success", 0)
    unreachable = sum(1 for r in valid if r.outcome != "success" and r.baseline != "success")

    print(f"params: {params_path}")
    print(f"world obstacle: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}] r={radius:.4f} m")
    print(f"mode={args.mode} cases={args.cases} seed={args.seed} "
          f"timeout={args.timeout:.1f}s dt={params.horizon:.3f}s "
          f"link_avoid={params.link_avoidance} self_avoid={params.self_avoidance}")
    print(f"invalid_target (inside obstacle): {invalid}")
    for outcome in ("success", "stuck", "timeout", "obstacle_hold", "self_hold", "tool_hold"):
        if outcome in counts:
            print(f"  {outcome:14s} {counts[outcome]:4d}  {100.0 * counts[outcome] / n_valid:5.1f}%")
    print(f"reachable without obstacle but failed with it: "
          f"{sum(1 for r in valid if r.outcome != 'success' and r.baseline == 'success')}")
    print(f"unreachable even without obstacle (APF/IK limit): {unreachable}  "
          f"{100.0 * unreachable / n_valid:.1f}% of valid")
    print(f"success rate on valid targets: {100.0 * n_success / n_valid:.1f}%")

    failures = [r for r in valid if r.outcome != "success"][:10]
    for result in failures:
        print(f"  case {result.idx}: {result.outcome} (baseline {result.baseline}), "
              f"goal [{result.goal_pos[0]:.3f}, {result.goal_pos[1]:.3f}, {result.goal_pos[2]:.3f}], "
              f"pos_err {result.pos_err:.3f} m ori_err {result.ori_err:.3f} rad, "
              f"min link {result.min_link:.3f} self {result.min_self:.3f} m")

    if args.csv:
        with open(args.csv, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["idx", "gx", "gy", "gz", "qx", "qy", "qz", "qw",
                             "outcome", "baseline", "steps", "time_s",
                             "pos_err", "ori_err", "min_tool", "min_link",
                             "min_self", "path_m"])
            for result in results:
                writer.writerow([result.idx, *result.goal_pos, *result.goal_quat,
                                 result.outcome, result.baseline, result.steps,
                                 round(result.time_s, 3), round(result.pos_err, 4),
                                 round(result.ori_err, 4), round(result.min_tool, 4),
                                 round(result.min_link, 4), round(result.min_self, 4),
                                 round(result.path_m, 4)])
        print(f"wrote {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
