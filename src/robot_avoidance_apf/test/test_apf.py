"""Field behavior that the static scene depends on."""

from robot_avoidance_apf.apf import ApfParams, SphereObstacle, field_command

IDENTITY = (0.0, 0.0, 0.0, 1.0)
PARAMS = ApfParams(
    k_att=1.0,
    k_rep=0.02,
    k_tan=0.04,
    k_ori=1.0,
    d0=0.20,
    d_min=0.03,
    v_max=0.08,
    omega_max=0.5,
)


def test_repulsion_is_zero_far_from_obstacle() -> None:
    command = field_command(
        position=(0.0, 0.0, 0.0),
        orientation=IDENTITY,
        goal_position=(0.0, 0.0, 0.0),
        goal_orientation=IDENTITY,
        obstacle=SphereObstacle((10.0, 0.0, 0.0), 0.05),
        params=PARAMS,
    )
    assert command.hold is False
    assert command.linear == (0.0, 0.0, 0.0)


def test_collinear_obstacle_has_perpendicular_component() -> None:
    command = field_command(
        position=(0.0, 0.0, 0.0),
        orientation=IDENTITY,
        goal_position=(1.0, 0.0, 0.0),
        goal_orientation=IDENTITY,
        obstacle=SphereObstacle((0.15, 0.0, 0.0), 0.05),
        params=PARAMS,
    )
    assert abs(command.linear[1]) > 1e-3 or abs(command.linear[2]) > 1e-3


def test_clearance_below_minimum_holds() -> None:
    command = field_command(
        position=(0.0, 0.0, 0.0),
        orientation=IDENTITY,
        goal_position=(1.0, 0.0, 0.0),
        goal_orientation=IDENTITY,
        obstacle=SphereObstacle((0.05, 0.0, 0.0), 0.04),
        params=PARAMS,
    )
    assert command.hold is True
    assert command.clearance < 0.03
    assert command.linear == (0.0, 0.0, 0.0)
    assert command.angular == (0.0, 0.0, 0.0)


def test_goal_inside_influence_radius_is_equilibrium() -> None:
    command = field_command(
        position=(0.25, 0.0, 0.0),
        orientation=IDENTITY,
        goal_position=(0.25, 0.0, 0.0),
        goal_orientation=IDENTITY,
        obstacle=SphereObstacle((0.0, 0.0, 0.0), 0.15),
        params=PARAMS,
    )
    assert command.hold is False
    assert command.clearance < 0.20
    assert command.linear == (0.0, 0.0, 0.0)
