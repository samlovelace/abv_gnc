import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

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

    goal_manager = ExecuteProcess(
        name='abv_goal_manager',
        cmd=['sudo', './scripts/run.sh', 'abv_goal_manager'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    control = ExecuteProcess(
        name='abv_control',
        cmd=['sudo', './scripts/run.sh', 'abv_control'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    # needs root for GPIO thruster access (see GpioThrusterDriver)
    thrusters = ExecuteProcess(
        name='abv_thrusters',
        cmd=['sudo', './scripts/run.sh', 'abv_thrusters'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    return LaunchDescription([guidance, navigation, goal_manager, control, thrusters])
