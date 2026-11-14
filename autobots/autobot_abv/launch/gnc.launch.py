from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_dir = get_package_share_directory("autobot_abv")
    config_dir = os.path.join(robot_dir, "config")

    return LaunchDescription([
        SetEnvironmentVariable(
            name="AUTOBOT_CONFIG_DIR",
            value=config_dir
        ),
        DeclareLaunchArgument('sim', default_value='false'),

        Node(
            package='autobot_goal_manager',
            executable='autobot_goal_manager',
            name='autobot_goal_manager',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='autobot_control',
            executable='autobot_control',
            name='autobot_control',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='autobot_thrusters',
            executable='autobot_thrusters',
            name='autobot_thrusters',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='autobot_navigation',
            executable='autobot_navigation',
            name='autobot_navigation',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='autobot_guidance',
            executable='autobot_guidance',
            name='autobot_guidance',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='autobot_bridge',
            executable='autobot_bridge',
            name='autobot_ptera_bridge',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='autobot_policy_registry',
            executable='autobot_policy_registry',
            name='autobot_policy_registry',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='autobot_rl-cpp',
            executable='autobot_rl-cpp',
            name='rl_policy_node',
            output='screen',
            emulate_tty=True,
        ),

        # ROSBridge WebSocket Server
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rosbridge_server'),
                    'launch',
                    'rosbridge_websocket_launch.xml'
                ])
            )
        ),
        
        # Web GUI Server
        ExecuteProcess(
            cmd=[
                'python3',
                '-m',
                'http.server',
                '8080',
                '--bind',
                '0.0.0.0',
                '--directory',
                PathJoinSubstitution([
                    FindPackageShare('autobot_bringup'),
                    'web'
                ]),
            ],
            output='screen',
            emulate_tty=True,
        ),

        # Simulation-only nodes
        GroupAction(
            condition=IfCondition(LaunchConfiguration('sim')),
            actions=[
                Node(
                    package='autobot_simulator',
                    executable='autobot_simulator',
                    output='screen',
                ),
                Node(
                    package='autobot_gui',
                    executable='autobot_gui',
                    output='screen',
                ),
            ]
        )
    ])