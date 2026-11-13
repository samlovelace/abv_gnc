cmake_minimum_required(VERSION 3.8)
project(autobot_obj_segmentation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autobot_common REQUIRED)
find_package(autobot_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)

set(SOURCES
  src/main.cpp
  src/SegmentationHandler.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  autobot_common::autobot_common
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
  ${PCL_LIBRARIES}
)
add_definitions(${PCL_DEFINITIONS})

# Bake PCL's actual lib dir into the binary's RPATH to avoid needing to use LD_LIBRARY_PATH at runtime
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${PCL_LIBRARY_DIRS}"
  INSTALL_RPATH "${PCL_LIBRARY_DIRS}"
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  autobot_msgs
  autobot_common
  Eigen3
  message_filters
  pcl_conversions
  tf2_ros
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
