cmake_minimum_required(VERSION 3.8)
project(autobot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(<dependency> REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

set(WEB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../web")
find_program(NPM_EXECUTABLE npm REQUIRED)

add_custom_target(build_web ALL
    COMMAND ${NPM_EXECUTABLE} ci
    COMMAND ${NPM_EXECUTABLE} run build
    WORKING_DIRECTORY "${WEB_DIR}"
    COMMENT "Building autobot web frontend"
)

install(
    DIRECTORY "${WEB_DIR}/dist/"
    DESTINATION share/${PROJECT_NAME}/web
)

ament_package()
