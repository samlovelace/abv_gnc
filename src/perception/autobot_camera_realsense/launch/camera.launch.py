from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # curated subset of rs_launch.py's own configurable_parameters - add more
    # the same way if a deployment needs them. Anything not surfaced here can
    # still be set via config_file (see config/camera_params.yaml).
    declared_arguments = [
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),
        # rs_launch.py uses the literal two-character string "''" (not a true
        # empty string) as its own "unset" sentinel for serial_no/config_file/
        # etc. - e.g. config_file's handler does
        # `{} if _config_file == "''" else yaml_to_dict(_config_file)`, so a
        # real '' here makes it try (and fail) to open '' as a yaml file.
        # Must match that sentinel exactly, not pass a plain empty string.
        DeclareLaunchArgument('serial_no', default_value="''"),
        DeclareLaunchArgument('enable_color', default_value='true'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('enable_infra1', default_value='false'),
        DeclareLaunchArgument('enable_infra2', default_value='false'),
        DeclareLaunchArgument('pointcloud.enable', default_value='false'),
        DeclareLaunchArgument('align_depth.enable', default_value='false'),
        # full param override file for anything not declared above (depth/rgb
        # resolution profiles, imu settings, etc.) - "''" means "use
        # realsense2_camera's own built-in defaults" (see comment above)
        DeclareLaunchArgument('config_file', default_value="''"),
    ]

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
            )
        ),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_namespace': LaunchConfiguration('camera_namespace'),
            'serial_no': LaunchConfiguration('serial_no'),
            'enable_color': LaunchConfiguration('enable_color'),
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_infra1': LaunchConfiguration('enable_infra1'),
            'enable_infra2': LaunchConfiguration('enable_infra2'),
            'pointcloud.enable': LaunchConfiguration('pointcloud.enable'),
            'align_depth.enable': LaunchConfiguration('align_depth.enable'),
            'config_file': LaunchConfiguration('config_file'),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [rs_launch])
