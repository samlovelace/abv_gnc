cmake_minimum_required(VERSION 3.1)
project(thruster_commander_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include_directories(${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/../include
    ${CMAKE_CURRENT_LIST_DIR}/../include/plog/plog-master/include
)

ament_add_gtest(thruster_commander_tests
  ThrusterCommander_Test.cpp
  ../src/ThrusterCommander.cpp
  ../src/UdpClient.cpp
  ../src/ConfigurationManager.cpp
)

target_link_libraries(thruster_commander_tests
  yaml-cpp
)

# ament_target_dependencies(thruster_commander_tests
#   Eigen3
# )

