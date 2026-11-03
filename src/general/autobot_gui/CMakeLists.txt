cmake_minimum_required(VERSION 3.8)
project(autobot_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Charts)
find_package(rclcpp REQUIRED)
find_package(autobot_msgs REQUIRED)
find_package(autobot_common REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(AUTOMOC_HDRS
  include/autobot_gui/LivePlot.h
  include/autobot_gui/TopicAdapter.hpp
  include/autobot_gui/ButtonAdapter.hpp
  include/autobot_gui/StatusIndicator.hpp
  include/autobot_gui/CommandPanel.h
  include/autobot_gui/StatusPanel.h
  include/autobot_gui/NetworkPinger.h
  include/autobot_gui/NodeHealthPanel.h
  include/autobot_gui/TableTopView.h
  include/autobot_gui/PolicySelectorPanel.h
  include/autobot_gui/ActivePolicyBanner.h
)

set(SOURCES
  src/CommandPanel.cpp
  src/main.cpp
  src/LivePlot.cpp
  src/StatusPanel.cpp
  src/NetworkPinger.cpp
  src/NodeHealthPanel.cpp
  src/TableTopView.cpp
  src/PolicySelectorPanel.cpp
  src/ActivePolicyBanner.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES} ${AUTOMOC_HDRS})

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(autobot_gui
  Qt6::Core
  Qt6::Gui
  Qt6::Widgets
  Qt6::Charts
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  autobot_msgs
  autobot_common
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
