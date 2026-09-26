"""Publish the closest point of an obstacle cloud to the frame origin."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution(
        [
            FindPackageShare("camera_perception"),
            "config",
            "closest_obstacle.yaml",
        ]
    )
    return LaunchDescription(
        [
            Node(
                package="camera_perception",
                executable="closest_obstacle_node",
                output="screen",
                parameters=[params],
            )
        ]
    )
