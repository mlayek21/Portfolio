import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("dobot_description")
    default_urdf = os.path.join(pkg_share, "urdf", "nova5.urdf")

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch PyBullet with native macOS 3D GUI window"
    )

    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=default_urdf,
        description="Path to Dobot Nova 5 URDF robot model"
    )

    width_arg = DeclareLaunchArgument(
        "window_width",
        default_value="2560",
        description="PyBullet 3D window width in pixels (High-Definition)"
    )

    height_arg = DeclareLaunchArgument(
        "window_height",
        default_value="1440",
        description="PyBullet 3D window height in pixels (High-Definition)"
    )

    dwell_arg = DeclareLaunchArgument(
        "trace_dwell_time",
        default_value="0.5",
        description="Dwell time in seconds before erasing end-effector trace upon phase completion"
    )

    color_arg = DeclareLaunchArgument(
        "trace_color",
        default_value="cycle",
        description="Color of end-effector path trace ('cycle', 'crimson', 'emerald', 'azure', 'purple', 'amber', 'cyan', 'gold')"
    )

    pybullet_node = Node(
        package="dobot_pybullet",
        executable="dobot_pybullet_node",
        name="dobot_pybullet_node",
        output="screen",
        parameters=[{
            "gui": LaunchConfiguration("gui"),
            "urdf_path": LaunchConfiguration("urdf_path"),
            "window_width": LaunchConfiguration("window_width"),
            "window_height": LaunchConfiguration("window_height"),
            "trace_dwell_time": LaunchConfiguration("trace_dwell_time"),
            "trace_color": LaunchConfiguration("trace_color"),
            "sample_dt": 0.02,
        }]
    )

    return LaunchDescription([
        gui_arg,
        urdf_arg,
        width_arg,
        height_arg,
        dwell_arg,
        color_arg,
        pybullet_node
    ])

