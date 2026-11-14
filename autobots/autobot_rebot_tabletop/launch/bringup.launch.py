import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    robot_dir = get_package_share_directory("autobot_rebot_tabletop")
    config_dir = os.path.join(robot_dir, "config")
    
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="AUTOBOT_CONFIG_DIR",
                value=config_dir
            ),
            
            # Manipulation nodes
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
            
            # Perception nodes
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
            ),
        ]
    )
