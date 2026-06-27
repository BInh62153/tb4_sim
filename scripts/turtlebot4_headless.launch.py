# Custom headless launch for TurtleBot4 simulation
# Replaces turtlebot4_ignition.launch.py với -s (server-only) flag
# để chạy không cần GUI / DISPLAY trong Docker
#
# Chain: file này → ignition.launch.py (với ign_args override) → turtlebot4_spawn.launch.py

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]
for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    pkg_tb4_ign = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_tb4_desc = get_package_share_directory('turtlebot4_description')
    pkg_create_desc = get_package_share_directory('irobot_create_description')
    pkg_create_ign = get_package_share_directory('irobot_create_ignition_bringup')
    pkg_tb4_gui = get_package_share_directory('turtlebot4_ignition_gui_plugins')
    pkg_create_plugins = get_package_share_directory('irobot_create_ignition_plugins')

    # Set resource paths (same as ignition.launch.py)
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_tb4_ign, 'worlds'), ':' +
            os.path.join(pkg_create_ign, 'worlds'), ':' +
            str(Path(pkg_tb4_desc).parent.resolve()), ':' +
            str(Path(pkg_create_desc).parent.resolve())
        ]
    )

    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=[
            os.path.join(pkg_tb4_gui, 'lib'), ':' +
            os.path.join(pkg_create_plugins, 'lib')
        ]
    )

    # Tìm gz_sim.launch.py qua ros_gz_sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

    # Chạy Gazebo server-only (-s) + run immediately (-r) + verbose 2
    # KHÔNG có --gui-config → không cần DISPLAY
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.sdf -r -v 2 -s']),
            ('gz_version', '6'),
        ]
    )

    # Clock bridge
    from launch_ros.actions import Node
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    )

    # Spawn robot (dùng launch file có sẵn)
    robot_spawn_launch = os.path.join(
        pkg_tb4_ign, 'launch', 'turtlebot4_spawn.launch.py')

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_spawn_launch),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('model',     LaunchConfiguration('model')),
            ('x',         LaunchConfiguration('x')),
            ('y',         LaunchConfiguration('y')),
            ('z',         LaunchConfiguration('z')),
            ('yaw',       LaunchConfiguration('yaw')),
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)
    ld.add_action(ignition_gazebo)
    ld.add_action(clock_bridge)
    ld.add_action(robot_spawn)
    return ld