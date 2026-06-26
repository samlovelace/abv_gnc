from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix

def generate_launch_description():

    abv_rl_script = PathJoinSubstitution([
        FindPackagePrefix('abv_rl'), 'lib', 'abv_rl', 'abv_rl_venv.sh'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),

        Node(
            package='abv_controller',
            executable='abv_controller',
            name='abv_controller',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='abv_navigation',
            executable='abv_navigation',
            name='abv_navigation',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='abv_guidance',
            executable='abv_guidance',
            name='abv_guidance',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='abv_bridge',
            executable='abv_bridge',
            name='abv_ptera_bridge',
            output='screen',
            emulate_tty=True,
        ),

        ExecuteProcess(
            cmd=[abv_rl_script],
            output='screen',
            emulate_tty=True,
        ),

        GroupAction(
            condition=IfCondition(LaunchConfiguration('sim')),
            actions=[
                Node(package='abv_simulator', executable='abv_simulator'),
                Node(package='abv_gui', executable='abv_gui'),
                Node(package='abv_commander', executable='abv_commander'),
            ]
        )
    ])