import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_cycles_arg = DeclareLaunchArgument(
        "num_cycles",
        default_value="3",
        description="Number of autonomous pick-and-place cycles to execute"
    )

    countdown_arg = DeclareLaunchArgument(
        "countdown_seconds",
        default_value="3",
        description="Warm-up countdown time in seconds before mission start"
    )

    auto_start_arg = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="Whether to automatically begin cycle loop after countdown"
    )

    pnp_node = Node(
        package="dobot_executive",
        executable="dobot_pick_and_place_node",
        name="dobot_pick_and_place_node",
        output="screen",
        parameters=[{
            "num_cycles": LaunchConfiguration("num_cycles"),
            "countdown_seconds": LaunchConfiguration("countdown_seconds"),
            "auto_start": LaunchConfiguration("auto_start"),
        }]
    )

    return LaunchDescription([
        num_cycles_arg,
        countdown_arg,
        auto_start_arg,
        pnp_node
    ])
