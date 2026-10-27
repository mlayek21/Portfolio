import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dobot_description = get_package_share_directory('dobot_description')
    pkg_dobot_gazebo = get_package_share_directory('dobot_gazebo')

    urdf_file = os.path.join(pkg_dobot_description, 'urdf', 'nova5.urdf')
    rviz_config_file = os.path.join(pkg_dobot_description, 'rviz', 'dobot.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Robot State Publisher (Computes TF tree from /joint_states and URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. C++ Dobot Simulation Node (Simulates robot joints and listens to trajectories)
    dobot_sim_node = Node(
        package='dobot_gazebo',
        executable='dobot_sim_node',
        name='dobot_sim_node',
        output='screen'
    )

    # 3. Synchronized RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        dobot_sim_node,
        rviz_node
    ])
