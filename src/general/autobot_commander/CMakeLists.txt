cmake_minimum_required(VERSION 3.8)
project(autobot_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autobot_msgs REQUIRED)
find_package(autobot_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ptera_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/CommanderComms.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/InputHandler.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

include_directories(${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${COMMON_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} autobot_common::autobot_common yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  autobot_msgs
  geometry_msgs
  ptera_msgs
  Eigen3
)

install(TARGETS ${PROJECT_NAME} 
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
