import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # NOTE: abv_description's chassis xacro and rebot_description's rebot.urdf
    # are two separate descriptions today - rebot.urdf is a plain URDF, not a
    # xacro macro, so it can't yet be xacro:included into the chassis tree.
    # robot_state_publisher below only publishes the chassis; unifying the two
    # into one combined robot_description is a follow-up (convert rebot.urdf
    # to a xacro macro, or publish both descriptions separately).
    abv_description_share = get_package_share_directory("abv_description")

    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="false"),

            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": xacro.process_file(
                            os.path.join(abv_description_share, "urdf", "abv.urdf.xacro")
                        ).toxml()
                    }
                ],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("autobot_bringup"), "launch", "gnc.launch.py"])
                ),
                launch_arguments={"sim": LaunchConfiguration("sim")}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("autobot_bringup"), "launch", "percep.launch.py"])
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("autobot_manip_bringup"), "launch", "sim.launch.py"])
                ),
            ),
        ]
    )
