#ifndef DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
#define DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_workbench_msgs/MotorStateList.h>
#include <dynamixel_workbench_msgs/GetDirection.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#define LEFT_WHEEL                  0
#define RIGHT_WHEEL                 1

namespace dynamixel_workbench_velocity_control
{
struct ReadData{
 std::vector<bool> dxl_bool_data;
 std::vector<int64_t> dxl_int_data;
};

class DynamixelWorkbenchVelocityControl
{
 public:
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  // ROS Topic Publisher
  ros::Publisher dxl_state_pub_;
  // ROS Service Server
  ros::ServiceServer wheel_control_server_;
  // Parameters
  bool dxl_addparam_result_;
  int dxl_comm_result_;
  bool dxl_getdata_result_;

  dynamixel_tool::DynamixelTool *dxl_motor_;
  std::vector<dynamixel_tool::DynamixelTool *> dynamixel_;

  std::string device_name_;
  std::string motor_model_;
  int motor_id_;
  float protocol_version_;
  float baud_rate_;
  int velocity_;

  std::map<std::string, ReadData *> read_data_;
  int64_t read_value_;
  int left_wheel_velocity;
  int right_wheel_velocity;

 public:
  DynamixelWorkbenchVelocityControl();
  ~DynamixelWorkbenchVelocityControl();
  bool dynamixelControlLoop(void);

 private:
  bool initDynamixelWorkbenchVelocityControl(void);
  bool shutdownDynamixelWorkbenchVelocityControl(void);

  bool initMotor(dynamixel_tool::DynamixelTool *dxl_motor, std::string motor_model, uint8_t motor_id, float protocol_version);
  bool writeDynamixelRegister(dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, int64_t value);
  bool writeSyncDynamixel(uint16_t addr, uint8_t length, int64_t left_wheel_value, int64_t right_wheel_value);
  bool readDynamixelRegister(dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, int64_t *value);
  bool readSyncDynamixel(uint16_t addr, uint8_t length, ReadData *data);

  bool readTorque(void);
  bool readMoving(void);
  bool readGoalPosition(void);
  bool readGoalVelocity(void);
  bool readPresentPosition(void);
  bool readPresentVelocity(void);
  bool readProfileVelocity(void);
  bool readMaxPositionLimit(void);
  bool readMinPositionLimit(void);

  bool writeTorque(bool onoff);
  bool writeVelocity(int64_t left_wheel_velocity, int64_t right_wheel_velocity);

  bool getPublisher(void);
  bool controlTurtlebotCallback(dynamixel_workbench_msgs::GetDirection::Request &req,
                           dynamixel_workbench_msgs::GetDirection::Response &res);
};
}

#endif //DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
