import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix

def generate_launch_description():

    abv_rl_script = PathJoinSubstitution([
        FindPackagePrefix('abv_rl'), 'lib', 'abv_rl', 'abv_rl_venv.sh'
    ])

    guidance = ExecuteProcess(
        name='abv_guidance',
        cmd=['sudo', './scripts/run.sh', 'abv_guidance'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    navigation = ExecuteProcess(
        name='abv_navigation',
        cmd=['sudo', './scripts/run.sh', 'abv_navigation'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    controller = ExecuteProcess(
        name='abv_controller',
        cmd=['sudo', './scripts/run.sh', 'abv_controller'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    rl = ExecuteProcess(
        name='abv_rl',
        cmd=[abv_rl_script],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    return LaunchDescription([guidance, navigation, controller, rl])