"""Start the simulator and the static-obstacle potential-field controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ur_type = LaunchConfiguration("ur_type")
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_sim"), "launch", "sim.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "gui": gui,
            "world": world,
            "sphere_motion": "static",
        }.items(),
    )
    apf_params = PathJoinSubstitution(
        [FindPackageShare("robot_avoidance_apf"), "config", "apf_params.yaml"]
    )
    apf = Node(
        package="robot_avoidance_apf",
        executable="apf_node",
        output="screen",
        parameters=[
            apf_params,
            {
                "use_sim_time": True,
                "world": ParameterValue(world, value_type=str),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur16e"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_sim"), "worlds", "empty.sdf"]
                ),
            ),
            sim,
            apf,
        ]
    )
