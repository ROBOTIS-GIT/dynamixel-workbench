################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_workbench_toolbox)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rclcpp REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies_lib
  "dynamixel_sdk"
  "rclcpp"
)

set(LIB_NAME "dynamixel_workbench_toolbox")

add_library(${LIB_NAME} SHARED
  src/${PROJECT_NAME}/dynamixel_item.cpp
  src/${PROJECT_NAME}/dynamixel_driver.cpp
  src/${PROJECT_NAME}/dynamixel_tool.cpp
  src/${PROJECT_NAME}/dynamixel_workbench.cpp
)
ament_target_dependencies(${LIB_NAME} ${dependencies_lib})

################################################################################
# Install
################################################################################
install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Test
################################################################################

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(dynamixel_sdk)
ament_export_dependencies(rclcpp)
ament_export_libraries(${LIB_NAME})
ament_package()
