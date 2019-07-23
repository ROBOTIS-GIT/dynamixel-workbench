################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_workbench_toolbox)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare ament specific configuration to be passed to dependent projects
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(dynamixel_sdk)

################################################################################
# Build
################################################################################
include_directories(
  include/${PROJECT_NAME}
)

add_library(dynamixel_workbench_toolbox
  src/${PROJECT_NAME}/dynamixel_item.cpp
  src/${PROJECT_NAME}/dynamixel_driver.cpp
  src/${PROJECT_NAME}/dynamixel_tool.cpp
  src/${PROJECT_NAME}/dynamixel_workbench.cpp
)
ament_target_dependencies(dynamixel_workbench_toolbox rclcpp dynamixel_sdk)
target_link_libraries(dynamixel_workbench_toolbox ${rclcpp_LIBRARIES} ${dynamixel_sdk_LIBRARIES})

################################################################################
# Install
################################################################################
install(
  TARGETS dynamixel_workbench_toolbox
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_package()
