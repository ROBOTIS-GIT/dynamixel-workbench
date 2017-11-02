################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_workbench_toolbox)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  dynamixel_sdk
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dynamixel_workbench_toolbox
 CATKIN_DEPENDS roscpp roslib dynamixel_sdk
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(dynamixel_workbench_toolbox
    src/dynamixel_tool.cpp
    src/dynamixel_driver.cpp
    src/dynamixel_multi_driver.cpp
)
add_dependencies(dynamixel_workbench_toolbox ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_workbench_toolbox ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS dynamixel_workbench_toolbox
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY dynamixel/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/dynamixel
)

################################################################################
# Test
################################################################################
