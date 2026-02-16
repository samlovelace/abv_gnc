cmake_minimum_required(VERSION 3.8)
project(abv_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AbvCommand.msg"
  "msg/AbvControllerStatus.msg"
  "msg/AbvGuidanceCommand.msg"
  "msg/AbvResponse.msg"
  "msg/AbvState.msg"
  "msg/AbvVec3.msg"
  "msg/Euler.msg"
  "msg/Quaternion.msg"
  "msg/RobotState.msg"
  "msg/Vec3.msg"
  "msg/VehicleWaypoint.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_generators std_msgs geometry_msgs)
ament_package()