from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
    ])
