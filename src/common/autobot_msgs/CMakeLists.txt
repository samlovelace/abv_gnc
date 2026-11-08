cmake_minimum_required(VERSION 3.8)
project(autobot_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlContext.msg"
  "msg/ControllerCommand.msg"
  "msg/ControllerStatus.msg"
  "msg/GuidanceCommand.msg"
  "msg/GuidanceStatus.msg"
  "msg/Heartbeat.msg"
  "msg/ManipHardwareGoal.msg"
  "msg/ManipHardwareState.msg"
  "msg/ManipHardwareTelemetry.msg"
  "msg/NodeStatus.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/Path.msg"
  "msg/PolicyActive.msg"
  "msg/PolicyEntry.msg"
  "msg/PolicyRegistry.msg"
  "msg/Response.msg"
  "msg/State.msg"
  "msg/ThrusterCommand.msg"
  "msg/ThrusterStatus.msg"
  "msg/Vec3.msg"
  "msg/Detection2D.msg"
  "msg/Detections2D.msg"
  "msg/ObjectObservation.msg"
  "msg/ObjectObservations.msg"
  "srv/PolicySwitch.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_generators std_msgs sensor_msgs geometry_msgs)
ament_package()