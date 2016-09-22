cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_workbench_controllers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_sdk
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
)

################################################
## Declare ROS messages, services and actions ##
################################################
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dynamixel_workbench_controllers
 CATKIN_DEPENDS roscpp std_msgs dynamixel_workbench_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(dynamixel_workbench_position_control src/dynamixel_workbench_position_control.cpp)
add_dependencies(dynamixel_workbench_position_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_workbench_position_control ${catkin_LIBRARIES})

add_executable(dynamixel_workbench_velocity_control src/dynamixel_workbench_velocity_control.cpp)
add_dependencies(dynamixel_workbench_velocity_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_workbench_velocity_control ${catkin_LIBRARIES})


add_executable(dynamixel_workbench_multi_port_control src/dynamixel_workbench_multi_port_control.cpp)
add_dependencies(dynamixel_workbench_multi_port_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_workbench_multi_port_control ${catkin_LIBRARIES})

add_executable(dynamixel_workbench_torque_control src/dynamixel_workbench_torque_control.cpp)
add_dependencies(dynamixel_workbench_torque_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_workbench_torque_control ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############

