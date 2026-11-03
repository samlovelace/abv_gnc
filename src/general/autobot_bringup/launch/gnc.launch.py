from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix

def generate_launch_description():

    autobot_rl_script = PathJoinSubstitution([
        FindPackagePrefix('autobot_rl'), 'lib', 'autobot_rl', 'autobot_rl_venv.sh'
    ])

    return LaunchDescription([
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

        GroupAction(
            condition=IfCondition(LaunchConfiguration('sim')),
            actions=[
                Node(package='autobot_simulator', executable='autobot_simulator'),
                Node(package='autobot_gui', executable='autobot_gui'),
                Node(package='autobot_commander', executable='autobot_commander'),
            ]
        )
    ])
