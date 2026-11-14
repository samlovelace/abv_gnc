import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    robot_dir = get_package_share_directory("autobot_abv")
    config_dir = os.path.join(robot_dir, "config")
    
        
    configs = SetEnvironmentVariable(
                name="AUTOBOT_CONFIG_DIR",
                value=config_dir)

    guidance = ExecuteProcess(
        name='autobot_guidance',
        cmd=['sudo', './scripts/run.sh', 'autobot_guidance'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    navigation = ExecuteProcess(
        name='autobot_navigation',
        cmd=['sudo', './scripts/run.sh', 'autobot_navigation'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    goal_manager = ExecuteProcess(
        name='autobot_goal_manager',
        cmd=['sudo', './scripts/run.sh', 'autobot_goal_manager'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    control = ExecuteProcess(
        name='autobot_control',
        cmd=['sudo', './scripts/run.sh', 'autobot_control'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    # needs root for GPIO thruster access (see GpioThrusterDriver); JetsonGPIO/build
    # is where libjetgpio.so's build output lives on the Jetson, mirrored here from
    # the rl entry below in case it isn't already resolved via run.sh's sourced env
    thrusters = ExecuteProcess(
        name='autobot_thrusters',
        cmd=[
            'sudo',
            '--preserve-env=ROS_DOMAIN_ID',
            'bash',
            '-c',
            'export LD_LIBRARY_PATH="/home/optimus/autobot/install/autobot_common/lib:'
            '/home/optimus/autobot/install/autobot_msgs/lib:'
            '/opt/ros/humble/lib/aarch64-linux-gnu:'
            '/opt/ros/humble/lib:'
            '/home/optimus/JetsonGPIO/build:'
            '/usr/local/lib" && '
            './scripts/run.sh autobot_thrusters'
        ],
        additional_env={
            'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')
        },
        output='screen')

    # TODO: need i explain.. pls fix
    rl = ExecuteProcess(
        name='autobot_rl-cpp',
        cmd=[
            'sudo',
            '--preserve-env=ROS_DOMAIN_ID',
            'bash',
            '-c',
            'export LD_LIBRARY_PATH="/home/optimus/autobot/install/autobot_common/lib:'
            '/home/optimus/autobot/install/autobot_msgs/lib:'
            '/home/optimus/robot_ws/install/robot_idl/lib:'
            '/home/optimus/robot_ws/install/ptera_msgs/lib:'
            '/opt/ros/humble/opt/rviz_ogre_vendor/lib:'
            '/opt/ros/humble/lib/aarch64-linux-gnu:'
            '/opt/ros/humble/lib:'
            '/home/optimus/JetsonGPIO/build:'
            '/usr/local/lib:'
            '/usr/local/cuda-12.6/lib64:'
            '/home/optimus/onnxruntime/lib" && '
            './scripts/run.sh autobot_rl-cpp'
        ],
        additional_env={
            'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')
        },
        output='screen'
    )

    # required alongside any control-policy node (autobot_control/autobot_rl-cpp) -
    # without it abv/policy/active never publishes, and autobot_thrusters'
    # arbiter silently drops every policy-sourced thruster command
    policy_registry = ExecuteProcess(
        name='autobot_policy_registry',
        cmd=['sudo', '--preserve-env=ROS_DOMAIN_ID', './scripts/run.sh', 'autobot_policy_registry'],
        additional_env={'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', '0')},
        output='screen')

    return LaunchDescription([configs, guidance, navigation, goal_manager, control, thrusters, rl, policy_registry])
