"""Filter the depth cloud, then publish the closest obstacle point."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("camera_perception")
    obstacle_params = PathJoinSubstitution(
        [share, "config", "obstacle_cloud.yaml"]
    )
    closest_params = PathJoinSubstitution(
        [share, "config", "closest_obstacle.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="camera_perception",
                executable="obstacle_cloud_node",
                output="screen",
                parameters=[obstacle_params],
            ),
            Node(
                package="camera_perception",
                executable="closest_obstacle_node",
                output="screen",
                parameters=[closest_params],
            ),
        ]
    )
