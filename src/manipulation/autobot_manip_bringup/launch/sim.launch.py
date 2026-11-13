from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="autobot_grasp_planner",
                executable="autobot_grasp_planner",
                output="screen",
            ),
            Node(
                package="autobot_manip_hardware",
                executable="autobot_manip_hardware",
                output="screen",
            ),
            # # Publishes robot/state for the configured MobileBase (exits if none)
            # Node(
            #     package="autobot_manip_hardware",
            #     executable="base_sim",
            #     output="screen",
            # ),
            Node(
                package="autobot_manip_controller",
                executable="autobot_manip_controller",
                output="screen",
            ),
            Node(
                package="autobot_manip_planner",
                executable="autobot_manip_planner",
                output="screen",
            ),
            Node(
                package="autobot_manip_executor",
                executable="autobot_manip_executor",
                output="screen",
            ),
        ]
    )
