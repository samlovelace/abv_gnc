cmake_minimum_required(VERSION 3.8)
project(autobot_obj_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(abv_common REQUIRED)
find_package(abv_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/ObjectDetectionHandler.cpp
  src/Letterbox.cpp
  src/YoloSegDetector.cpp
  src/DetectorFactory.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
  abv_common::abv_common
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  abv_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
