################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_workbench_tutorials)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_workbench_msgs
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
 LIBRARIES dynamixel_workbench_tutorials
 CATKIN_DEPENDS roscpp std_msgs dynamixel_workbench_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(dynamixel_workbench_pan_tilt src/dynamixel_workbench_pan_tilt.cpp)
add_dependencies(dynamixel_workbench_pan_tilt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_workbench_pan_tilt ${catkin_LIBRARIES})

add_executable(dynamixel_workbench_wheel src/dynamixel_workbench_wheel.cpp)
add_dependencies(dynamixel_workbench_wheel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_workbench_wheel ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS dynamixel_workbench_pan_tilt
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS dynamixel_workbench_wheel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
