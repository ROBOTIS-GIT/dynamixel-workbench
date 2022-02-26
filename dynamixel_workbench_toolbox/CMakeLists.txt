################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_workbench_toolbox)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS include/dynamixel_workbench_toolbox ${rclcpp_INCLUDE_DIRS} ${dynamixel_sdk_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${rclcpp_LIBRARY_DIRS} ${dynamixel_sdk_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS dynamixel_sdk ${rclcpp_LIBRARIES} ${dynamixel_sdk_LIBRARIES} ${ament_cmake_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################

################################################################################
# Build
################################################################################

add_library(dynamixel_workbench_toolbox
  src/dynamixel_workbench/dynamixel_item.cpp
  src/dynamixel_workbench/dynamixel_driver.cpp
  src/dynamixel_workbench/dynamixel_tool.cpp
  src/dynamixel_workbench/dynamixel_workbench.cpp)

target_link_libraries(dynamixel_workbench_toolbox ${LIBS})

################################################################################
# Install
################################################################################
install(TARGETS dynamixel_workbench_toolbox
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/dynamixel_workbench/
  DESTINATION include/${PROJECT_NAME})

################################################################################
# Test
################################################################################

ament_export_dependencies(rclcpp)
ament_export_dependencies(dynamixel_sdk)
ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
