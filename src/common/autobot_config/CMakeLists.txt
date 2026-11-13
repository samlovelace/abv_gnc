cmake_minimum_required(VERSION 3.8)
project(autobot_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(common_config STATIC src/CommonConfigManager.cpp)
add_library(gnc_config    STATIC src/GncConfigManager.cpp)
add_library(manip_config  STATIC src/ManipConfigManager.cpp)
add_library(percep_config STATIC src/PercepConfigManager.cpp)

foreach(target common_config gnc_config manip_config percep_config)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${target} PUBLIC yaml-cpp rclcpp::rclcpp)
  ament_target_dependencies(${target} PUBLIC ament_index_cpp)
endforeach()

ament_target_dependencies(manip_config PUBLIC kdl_parser urdf)

install(TARGETS common_config gnc_config manip_config percep_config
        EXPORT autobot_config_targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/
        DESTINATION include)

ament_export_targets(autobot_config_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp yaml-cpp ament_index_cpp kdl_parser urdf Eigen3)
ament_package()
