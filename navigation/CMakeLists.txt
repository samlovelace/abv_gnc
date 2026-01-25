cmake_minimum_required(VERSION 3.8)
project(abv_navigation)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_idl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static libmotioncapture")
set(LIBMOTIONCAPTURE_ENABLE_OPTITRACK_CLOSED_SOURCE OFF CACHE BOOL "Dont Use NatNetSDK at all")
add_subdirectory(
  ${CMAKE_CURRENT_LIST_DIR}/../abv_gnc/dependencies/libmotioncapture libmotioncapture
)

set(SOURCES
  src/ExtendedKalmanFilter.cpp
	src/OptitrackStateFetcher_LibMocap.cpp
  src/RosStatePublisher.cpp
	src/SimulatedStateFetcher.cpp
	src/VehicleStateTracker.cpp
	src/main.cpp
)

include(${CMAKE_CURRENT_LIST_DIR}/../abv_gnc/common/CMakeLists.txt)

add_executable(${PROJECT_NAME} ${SOURCES} ${COMMON_SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE 
    include
    ${COMMON_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../abv_gnc/dependencies/plog/plog-master/include
	${CMAKE_CURRENT_LIST_DIR}/../abv_gnc/dependencies/libmotioncapture/include
	${EIGEN3_INCLUDE_DIRS}
)

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robot_idl
  Eigen3
)

target_link_libraries(${PROJECT_NAME} yaml-cpp libmotioncapture)
# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()