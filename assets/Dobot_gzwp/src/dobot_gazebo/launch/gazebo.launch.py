import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dobot_description = get_package_share_directory('dobot_description')
    pkg_dobot_gazebo = get_package_share_directory('dobot_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(pkg_dobot_description, 'urdf', 'nova5.urdf')
    rviz_config_file = os.path.join(pkg_dobot_description, 'rviz', 'dobot.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Resolve package paths in raw URDF
    robot_desc = robot_desc.replace('$(find dobot_gazebo)', pkg_dobot_gazebo)
    robot_desc = robot_desc.replace('$(find dobot_description)', pkg_dobot_description)

    # Configure resource paths so Gazebo Sim finds package://dobot_description meshes
    description_share_parent = os.path.dirname(pkg_dobot_description)
    env_ign_resource = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        description_share_parent
    )
    env_gz_resource = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        description_share_parent
    )

    # Configure plugin paths so Gazebo Sim finds ign_ros2_control system plugins on macOS
    conda_prefix = os.environ.get('CONDA_PREFIX', '/Users/monojitlayek/miniforge3/envs/ros2_humble')
    ros_lib_dir = os.path.join(conda_prefix, 'lib')
    env_ign_plugins = AppendEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        ros_lib_dir
    )
    env_gz_plugins = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        ros_lib_dir
    )

    import platform
    is_macos = platform.system() == 'Darwin'
    default_gui = 'false' if is_macos else 'true'

    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value=default_gui,
        description='Whether to open the Gazebo Sim graphical user interface (disabled by default on macOS due to upstream OGRE/Metal support)'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to open RViz'
    )

    # 1. Gazebo Sim Physics Server (Runs physics and ros2_control in headless server)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-s -r empty.sdf'}.items()
    )

    # 2. Gazebo Sim Graphical Interface (Runs on separate thread/process on macOS)
    gazebo_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # 2. ROS-Gazebo Clock Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 3. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 4. Spawn Robot in Gazebo Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'dobot_nova5', '-world', 'empty', '-z', '0.0'],
        output='screen'
    )

    # 5. Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 6. Dobot Arm Trajectory Controller Spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['dobot_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 7. Synchronized RViz (Optional with rviz:=true / false)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # Ensure controllers spawn after the robot entity is created in Gazebo Sim
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        env_ign_resource,
        env_gz_resource,
        env_ign_plugins,
        env_gz_plugins,
        gazebo_server,
        gazebo_gui,
        gz_bridge,
        robot_state_publisher,
        spawn_entity,
        delay_joint_state_broadcaster,
        delay_arm_controller,
        rviz_node
    ])

