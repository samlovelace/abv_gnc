from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    web_share_dir = PathJoinSubstitution([FindPackageShare("autobot_bringup"), "web"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "panels_json",
                description="Path to the robot-specific panels.json listing which GUI panels to show",
            ),

            # Overwrite the served panels.json with this robot's panel list.
            # The frontend fetches it asynchronously well after this
            # near-instant copy completes, so no explicit ordering against
            # the http.server process below is needed.
            ExecuteProcess(
                cmd=[
                    "cp",
                    LaunchConfiguration("panels_json"),
                    PathJoinSubstitution([web_share_dir, "panels.json"]),
                ],
                output="screen",
            ),

            # ROSBridge WebSocket Server
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("rosbridge_server"),
                        "launch",
                        "rosbridge_websocket_launch.xml",
                    ])
                )
            ),

            # Web GUI Server
            ExecuteProcess(
                cmd=[
                    "python3",
                    "-m",
                    "http.server",
                    "8080",
                    "--bind",
                    "0.0.0.0",
                    "--directory",
                    web_share_dir,
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
