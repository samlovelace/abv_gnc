cmake_minimum_required(VERSION 3.8)
project(abv_guidance)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(abv_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plog REQUIRED)

set(ABV_GUIDANCE_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/FromFileGenerator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/RosCommandSource.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/StateMachine.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/StraightLineGenerator.cpp
)

# include the common dir cmake file
include(${CMAKE_CURRENT_LIST_DIR}/../../abv_gnc/common/CMakeLists.txt)

# Create executable
add_executable(${PROJECT_NAME} ${ABV_GUIDANCE_SOURCES} ${COMMON_SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/include
    ${COMMON_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    abv_msgs
    Eigen3
)

target_link_libraries(${PROJECT_NAME} yaml-cpp plog::plog)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()