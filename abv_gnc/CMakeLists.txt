cmake_minimum_required(VERSION 3.16)
project(abv_gnc)

find_package(ament_cmake REQUIRED)

install(DIRECTORY configuration/ DESTINATION share/${PROJECT_NAME}/configuration)
install(DIRECTORY tools/launch/ DESTINATION share/${PROJECT_NAME})

ament_package()
