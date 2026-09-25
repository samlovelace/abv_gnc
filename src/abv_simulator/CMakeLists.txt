cmake_minimum_required(VERSION 3.8)
project(abv_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(abv_msgs REQUIRED)
find_package(abv_common REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(plog REQUIRED)
find_package(Threads REQUIRED)

##################################################################
# VRPN (for the OptiTrack emulator)
#
# OptitrackEmulator stands up a real VRPN server so that
# OptitrackStateFetcher_LibMocap (abv_navigation) can be pointed at
# it unmodified during simulation, exercising the same code path
# used on the real vehicle.
#
# libmotioncapture (the fork built by scripts/setup.sh) only
# installs/exports its own public API - the vrpn server classes we
# need live in its private build tree. Point at the checkout used by
# that script (cloned to ~/libs by default) to find them.
##################################################################
set(VRPN_SRC_DIR "$ENV{HOME}/libs/libmotioncapture" CACHE PATH
    "Path to the libmotioncapture checkout containing the vrpn dependency")

find_path(VRPN_INCLUDE_DIR vrpn_Tracker.h
  HINTS ${VRPN_SRC_DIR}/deps/vrpn)

find_library(VRPN_LIBRARY vrpn
  HINTS ${VRPN_SRC_DIR}/build/deps/vrpn)

find_library(VRPN_QUAT_LIBRARY quat
  HINTS ${VRPN_SRC_DIR}/build/deps/vrpn/quat)

if(NOT VRPN_INCLUDE_DIR OR NOT VRPN_LIBRARY OR NOT VRPN_QUAT_LIBRARY)
  message(FATAL_ERROR
    "Could not find vrpn under ${VRPN_SRC_DIR}. "
    "Build libmotioncapture per scripts/setup.sh first, or set -DVRPN_SRC_DIR=<path>.")
endif()

set(SIMULATOR_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/UdpServer.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/VehicleSimulator.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/OptitrackEmulator.cpp
)

add_executable(${PROJECT_NAME} ${SIMULATOR_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${VRPN_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  abv_common::abv_common
  yaml-cpp
  plog::plog
  ${VRPN_LIBRARY}
  ${VRPN_QUAT_LIBRARY}
  Threads::Threads
)
ament_target_dependencies(${PROJECT_NAME}
	Eigen3
	rclcpp
	abv_msgs
)

# Install the executable
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
