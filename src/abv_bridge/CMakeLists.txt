cmake_minimum_required(VERSION 3.8)
project(abv_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(abv_msgs REQUIRED)
find_package(robot_idl REQUIRED)

set(SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/ControllerStatusConvertor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/NavigationConvertor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/WaypointConvertor.cpp
)

# include the common dir cmake file
include(${CMAKE_CURRENT_LIST_DIR}/../../abv_gnc/common/CMakeLists.txt)

add_executable(${PROJECT_NAME} ${SOURCES} ${COMMON_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE 
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${COMMON_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  abv_msgs
  robot_idl
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
