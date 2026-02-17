cmake_minimum_required(VERSION 3.8)
project(abv_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(abv_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SFML COMPONENTS window graphics REQUIRED)

set(SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/SfmlControlDevice.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/TeleopController.cpp
)

include_directories(${PROJECT_NAME}
  ${CMAKE_CURRENT_LIST_DIR}/include
)

add_executable(${PROJECT_NAME} ${SOURCES})

ament_target_dependencies(${PROJECT_NAME} rclcpp abv_msgs Eigen3)
target_link_libraries(${PROJECT_NAME} sfml-window sfml-graphics )

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
