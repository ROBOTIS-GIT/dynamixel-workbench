#ifndef DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
#define DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_workbench_msgs/MotorStateList.h>
#include <dynamixel_workbench_msgs/GetPosition.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

#define PAN_MOTOR                   0
#define TILT_MOTOR                  1

namespace dynamixel_workbench_position_control
{
struct ReadData{
 std::vector<bool> dxl_bool_data;
 std::vector<int64_t> dxl_int_data;
};

class DynamixelWorkbenchPositionControl
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
  ros::ServiceServer position_control_server;
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

  std::map<std::string, ReadData *> read_data_;
  int64_t read_value_;
  int pan_velocity_value_;
  int tilt_velocity_value_;

 public:
  DynamixelWorkbenchPositionControl();
  ~DynamixelWorkbenchPositionControl();
  bool dynamixelControlLoop(void);

 private:
  bool initDynamixelWorkbenchPositionControl(void);
  bool shutdownDynamixelWorkbenchPositionControl(void);

  bool initMotor(dynamixel_tool::DynamixelTool *dxl_motor, std::string motor_model, uint8_t motor_id, float protocol_version);
  bool writeDynamixelRegister(dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, int64_t value);
  bool writeSyncDynamixel(uint16_t addr, uint8_t length, int64_t pan_motor_value, int64_t tilt_motor_value);
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
  bool writeVelocity(int64_t pan_value, int64_t tilt_value);

  bool getPublisher(void);
  bool controlPanTiltMotorCallback(dynamixel_workbench_msgs::GetPosition::Request &req,
                                   dynamixel_workbench_msgs::GetPosition::Response &res);
};
}

#endif //DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
