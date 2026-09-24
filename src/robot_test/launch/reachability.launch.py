from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="robot_test",
                executable="reachability_node",
                output="screen",
                parameters=[
                    {"cartesian_step_m": 0.01},
                ],
            ),
        ]
    )
