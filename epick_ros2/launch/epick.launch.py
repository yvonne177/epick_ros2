from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_driver(context):
    parameters = [LaunchConfiguration("config").perform(context)]
    device = LaunchConfiguration("device").perform(context)
    if device:
        parameters.append({"device": device})
    return [Node(
        package="epick_ros2",
        executable="epick_driver",
        name="epick_driver",
        namespace="",
        output="screen",
        parameters=parameters,
    )]


def generate_launch_description():
    parameters_file = Path(get_package_share_directory("epick_ros2")) / "config" / "epick.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true",
                                  description="Open the ePick operator GUI"),
            DeclareLaunchArgument("config", default_value=str(parameters_file),
                                  description="Driver parameter YAML file"),
            DeclareLaunchArgument("device", default_value="",
                                  description="Optional Linux serial device override"),
            OpaqueFunction(function=launch_driver),
            Node(
                package="epick_ros2",
                executable="epick_gui",
                name="epick_gui",
                output="screen",
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
        ]
    )
