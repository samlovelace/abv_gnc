import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    guidance = Node(
        package='abv_guidance',
        executable='abv_guidance',
        output='screen',
        emulate_tty=True)

    navigation = Node(
        package='abv_navigation',
        executable='abv_navigation',
        output='screen',
        emulate_tty=True)

    controller = ExecuteProcess(
        name='abv_controller',
        cmd=['sudo', './run.sh', 'abv_controller'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    return LaunchDescription([guidance, navigation, controller])