"""Launch the live OHM mapping node with default parameters.

Override any parameter from the CLI, e.g.:
    ros2 launch ohm_ros2 ohm_live.launch.py use_ndt:=true resolution:=0.05
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("ohm_ros2")
    default_params = PathJoinSubstitution([pkg_share, "config", "ohm_live.yaml"])

    params_file = LaunchConfiguration("params_file")
    use_ndt = LaunchConfiguration("use_ndt")
    resolution = LaunchConfiguration("resolution")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params,
                                  description="YAML params file for ohm_live_node"),
            DeclareLaunchArgument("use_ndt", default_value="false",
                                  description="Use NDT-OM mapper instead of plain occupancy"),
            DeclareLaunchArgument("resolution", default_value="0.1",
                                  description="Voxel edge length in metres"),
            Node(
                package="ohm_ros2",
                executable="ohm_live_node",
                name="ohm_live_node",
                output="screen",
                parameters=[
                    params_file,
                    {"use_ndt": use_ndt, "resolution": resolution},
                ],
            ),
        ]
    )
