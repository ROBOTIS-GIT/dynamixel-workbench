cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_workbench_toolbox)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dynamixel_workbench_toolbox
 CATKIN_DEPENDS roscpp roslib
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(dynamixel_workbench_toolbox
  src/dynamixel_tool.cpp
)

add_dependencies(dynamixel_workbench_toolbox ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_workbench_toolbox ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############

