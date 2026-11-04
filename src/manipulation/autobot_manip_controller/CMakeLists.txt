cmake_minimum_required(VERSION 3.8)
project(autobot_manip_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(autobot_msgs REQUIRED)
find_package(autobot_common REQUIRED)
find_package(autobot_manip_common REQUIRED)

find_package(ptera_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plog REQUIRED)
find_package(ruckig REQUIRED)

set(SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/CommandHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/HardwareInterface.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/JointPositionWaypoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/JointVelocityWaypoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/Manipulator.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/MobileBaseInterface.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/StateMachine.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/TaskPositionWaypoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/TaskVelocityWaypoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/WaypointExecutor.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

include_directories(${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${PCL_INCLUDE_DIRS}
    /usr/local/include
    /opt/ros/humble/include/kdl_parser
    /opt/ros/humble/include/urdf
)

set(COMMON_LIBS
  kdl_parser
  urdf
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  autobot_msgs
  autobot_common
  autobot_manip_common
  std_msgs
  ptera_msgs
)

target_link_libraries(${PROJECT_NAME} yaml-cpp ruckig)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()
