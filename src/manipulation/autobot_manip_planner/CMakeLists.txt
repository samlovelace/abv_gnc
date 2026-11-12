cmake_minimum_required(VERSION 3.8)
project(autobot_manip_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(autobot_common REQUIRED)
find_package(autobot_msgs REQUIRED)
find_package(autobot_manip_common REQUIRED)

find_package(ompl REQUIRED)
find_package(coal REQUIRED)

set(SOURCES
  src/main.cpp
  src/ManipPlannerHandler.cpp
  src/RobotCollisionModel.cpp
  src/CollisionChecker.cpp
  src/MotionPlanner.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  autobot_common::autobot_common
  autobot_manip_common::autobot_manip_common
  ${OMPL_LIBRARIES}
  coal::coal
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
  kdl_parser
  urdf
  autobot_msgs
  autobot_common
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
