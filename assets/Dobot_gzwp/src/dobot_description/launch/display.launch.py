import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dobot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'nova5.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'dobot.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Node 1: robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Node 2: joint_state_publisher_gui (interactive joint sliders initialized in home pose)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'zeros': {
                    'joint1': -0.75,
                    'joint2': -0.30,
                    'joint3': 0.50,
                    'joint4': -0.20,
                    'joint5': 0.40,
                    'joint6': 0.0
                }
            }]
        ),

        # Node 3: rviz2 with pre-configured dobot.rviz view
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])