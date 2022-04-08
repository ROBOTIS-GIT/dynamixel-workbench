cmake_minimum_required(VERSION 3.5)
project(dynamixel_workbench_toolbox)

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rclcpp REQUIRED)

set(INCLUDE_DIRS include/dynamixel_workbench_toolbox)
include_directories(${INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_library(dynamixel_workbench_toolbox
  src/dynamixel_workbench/dynamixel_item.cpp
  src/dynamixel_workbench/dynamixel_driver.cpp
  src/dynamixel_workbench/dynamixel_tool.cpp
  src/dynamixel_workbench/dynamixel_workbench.cpp)

ament_target_dependencies(dynamixel_workbench_toolbox dynamixel_sdk)

install(TARGETS dynamixel_workbench_toolbox
        LIBRARY DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/dynamixel_workbench/
  DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(${INCLUDE_DIRS})

ament_export_dependencies(rclcpp)
ament_export_dependencies(dynamixel_sdk)
ament_export_dependencies(ament_cmake)

ament_package()
