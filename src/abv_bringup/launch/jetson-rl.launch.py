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

    controller = ExecuteProcess(
        name='abv_controller',
        cmd=['sudo', './scripts/run.sh', 'abv_controller'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    rl = ExecuteProcess(
        name='abv_rl-cpp',
        cmd=[
            'sudo',
            '--preserve-env=ROS_DOMAIN_ID',
            'bash',
            '-c',
            'export LD_LIBRARY_PATH="/home/optimus/abv_gnc/install/abv_common/lib:'
            '/home/optimus/abv_gnc/install/abv_msgs/lib:'
            '/home/optimus/robot_ws/install/robot_idl/lib:'
            '/home/optimus/robot_ws/install/ptera_msgs/lib:'
            '/opt/ros/humble/opt/rviz_ogre_vendor/lib:'
            '/opt/ros/humble/lib/aarch64-linux-gnu:'
            '/opt/ros/humble/lib:'
            '/home/optimus/JetsonGPIO/build:'
            '/usr/local/lib:'
            '/usr/local/cuda-12.6/lib64:'
            '/home/optimus/onnxruntime/lib" && '
            './scripts/run.sh abv_rl-cpp'
        ],
        additional_env={
            'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')
        },
        output='screen'
    )

    return LaunchDescription([guidance, navigation, controller, rl])
