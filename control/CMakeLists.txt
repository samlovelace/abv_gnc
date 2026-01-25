cmake_minimum_required(VERSION 3.8)
project(abv_controller)

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

# Architecture detection
set(ARCH_X86 FALSE)
set(ARCH_ARM FALSE)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ARCH_X86 TRUE)
  add_definitions(-DARCH_X86)
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ARCH_ARM TRUE)
  add_definitions(-DARCH_ARM)
endif()

# Source files (common)
set(ABV_CONTROLLER_SOURCES 
  src/CommandHandler.cpp
  src/Controller.cpp
  src/main.cpp 
  src/StateMachine.cpp
  src/ThrusterCommander.cpp
  src/UdpThrusterDriver.cpp
  src/Vehicle.cpp
)

# Conditionally include architecture dependent sources
if(ARCH_ARM)
	list(APPEND ABV_CONTROLLER_SOURCES 
		src/GpioThrusterDriver.cpp)
endif()

# include the common dir cmake file
include(${CMAKE_CURRENT_LIST_DIR}/../abv_gnc/common/CMakeLists.txt)

add_executable(${PROJECT_NAME} ${ABV_CONTROLLER_SOURCES} ${COMMON_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/include
  ${COMMON_INCLUDE_DIR}
	${CMAKE_CURRENT_LIST_DIR}/../abv_gnc/dependencies/plog/plog-master/include
	${EIGEN3_INCLUDE_DIRS}
)

if(ARCH_ARM)
	target_include_directories(${PROJECT_NAME} PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/../abv_gnc/dependencies/JETGPIO-1.2)
endif()

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robot_idl
  Eigen3
)

# Link libraries
target_link_libraries(${PROJECT_NAME} yaml-cpp)

if(ARCH_ARM)
	# manually set because it doesnt use cmake 
	set(jetgpio_LIB ${CMAKE_CURRENT_LIST_DIR}/../abv_gnc/dependencies/JETGPIO-1.2/libjetgpio.so)
    target_link_libraries(${PROJECT_NAME} ${jetgpio_LIB})
	install(FILES ${CMAKE_CURRENT_LIST_DIR}/../abv_gnc/dependencies/JETGPIO-1.2/libjetgpio.so DESTINATION lib/${PROJECT_NAME})
endif()

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS ../run.sh
  DESTINATION bin
)

ament_package()
