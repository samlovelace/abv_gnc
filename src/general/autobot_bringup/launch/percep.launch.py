from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),

        Node(
            package='autobot_obj_detection',
            executable='autobot_obj_detection',
            name='autobot_obj_detection',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='autobot_obj_segmentation',
            executable='autobot_obj_segmentation',
            name='autobot_obj_segmentation',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='autobot_obj_registry',
            executable='autobot_obj_registry',
            name='autobot_obj_registry',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='autobot_camera_tf',
            executable='autobot_camera_tf',
            name='autobot_camera_tf',
            output='screen',
            emulate_tty=True,
        )
    ])