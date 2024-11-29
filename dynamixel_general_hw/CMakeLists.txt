cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_general_hw)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  hardware_interface
  message_generation
  roscpp
  std_msgs
  transmission_interface
)

add_message_files(
  FILES
  DynamixelState.msg
  DynamixelStateList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_general_hw
  CATKIN_DEPENDS
  controller_manager
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  hardware_interface
  message_runtime
  roscpp
  std_msgs
  transmission_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dynamixel_general_hw src/dynamixel_general_hw.cpp)
add_dependencies(dynamixel_general_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_general_hw ${catkin_LIBRARIES})

add_executable(dynamixel_general_control_node src/dynamixel_general_control_node.cpp)
add_dependencies(dynamixel_general_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_general_control_node dynamixel_general_hw ${catkin_LIBRARIES})

install(TARGETS dynamixel_general_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS dynamixel_general_hw
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
