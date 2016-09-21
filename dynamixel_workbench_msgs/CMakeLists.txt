cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_workbench_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  DynamixelAX.msg
  DynamixelRX.msg
  DynamixelMX.msg
  DynamixelMX64.msg
  DynamixelMX106.msg
  DynamixelEX.msg
  DynamixelXL.msg
  DynamixelXM.msg
  DynamixelPro.msg
  DynamixelProL42.msg
  MotorState.msg
  MotorStateList.msg
  DynamixelCommand.msg
  WorkbenchParam.msg
)

add_service_files(
  FILES
  SetPosition.srv
  SetDirection.srv
  GetWorkbenchParam.srv
)

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
   CATKIN_DEPENDS std_msgs message_generation
)

###########
## Build ##
###########


#############
## Install ##
#############


#############
## Testing ##
#############

