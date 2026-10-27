import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dobot_gazebo = get_package_share_directory('dobot_gazebo')

    # 1. Include Simulation Launch (robot_state_publisher, dobot_sim_node, rviz2)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dobot_gazebo, 'launch', 'sim.launch.py')
        )
    )

    # 2. End-to-End Executive Node
    executive_node = Node(
        package='dobot_executive',
        executable='dobot_executive_node',
        name='dobot_executive_node',
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        executive_node
    ])

