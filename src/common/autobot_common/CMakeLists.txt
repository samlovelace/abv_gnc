cmake_minimum_required(VERSION 3.8)
project(autobot_common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autobot_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── ONNX Runtime ──────────────────────────────────────────────────────────────
# Set ORT_ROOT to your local ONNX Runtime install if it isn't on the system
# path, e.g.: cmake -DORT_ROOT=/opt/onnxruntime ..
# Expected layout under ORT_ROOT: include/onnxruntime_cxx_api.h, lib/libonnxruntime.so
if(DEFINED ORT_ROOT)
    set(ORT_INCLUDE_DIR "${ORT_ROOT}/include")
    set(ORT_LIB_DIR     "${ORT_ROOT}/lib")
else()
    find_path(ORT_INCLUDE_DIR
        NAMES onnxruntime_cxx_api.h
        PATHS /usr/local/include /usr/include
    )
    find_library(ORT_LIB
        NAMES onnxruntime
        PATHS /usr/local/lib /usr/lib
    )
endif()

if(NOT ORT_LIB)
    find_library(ORT_LIB NAMES onnxruntime PATHS "${ORT_LIB_DIR}")
endif()

if(NOT ORT_INCLUDE_DIR OR NOT ORT_LIB)
    message(FATAL_ERROR
        "ONNX Runtime not found. Install it and set -DORT_ROOT=<path>.")
endif()

message(STATUS "ORT include : ${ORT_INCLUDE_DIR}")
message(STATUS "ORT lib     : ${ORT_LIB}")

add_library(${PROJECT_NAME} SHARED
  src/ConfigurationManager.cpp
  src/DataLogger.cpp
  src/HeartbeatPublisher.cpp
  src/RosNavigationListener.cpp
  src/RosTopicManager.cpp
  src/UdpClient.cpp
  src/Watchdog.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ORT_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} PUBLIC
  yaml-cpp
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${autobot_msgs_TARGETS}
  ${ORT_LIB})

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}_targets
        LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs autobot_msgs yaml-cpp)
ament_package()