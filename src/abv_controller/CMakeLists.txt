cmake_minimum_required(VERSION 3.8)
project(abv_controller)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(abv_msgs REQUIRED)
find_package(abv_common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(plog REQUIRED)

find_package(pca9685 REQUIRED)

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
  src/I2cThrusterDriver.cpp
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

add_executable(${PROJECT_NAME} ${ABV_CONTROLLER_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/include
	${EIGEN3_INCLUDE_DIRS}
)

if(ARCH_ARM)
	target_include_directories(${PROJECT_NAME} 
    PRIVATE
      /usr/include/
  )
endif()

# Link libraries
target_link_libraries(${PROJECT_NAME} 
  abv_common::abv_common 
  yaml-cpp 
  plog::plog 
  pca9685::pca9685
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  abv_msgs
  Eigen3
)

if(ARCH_ARM)
	set(jetgpio_LIB /usr/lib/libjetgpio.so) # manually set because it doesnt use cmake 
  target_link_libraries(${PROJECT_NAME} ${jetgpio_LIB})
endif()

# Install
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
