#ifndef DYNAMIXEL_WORKBENCH_TOOL_H_
#define DYNAMIXEL_WORKBENCH_TOOL_H_

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include "dynamixel_sdk.h"  // Uses Dynamixel SDK Library
#include "dxl_motor.h"

// Control table address (Dynamixel XM series)
#define ADDR_XM_GET_ID              7
#define ADDR_XM_OPERATIING_MODE     11
#define ADDR_XM_TORQUE_ENABLE       64
#define ADDR_XM_PROFILE_VELOCITY    112
#define ADDR_XM_GOAL_VELOCITY       104
#define ADDR_XM_GOAL_POSITION       116
#define ADDR_XM_REALTIME_TICK       120
#define ADDR_XM_PRESENT_VELOCITY    128
#define ADDR_XM_PRESENT_POSITION    132
#define ADDR_XM_VOLTAGE             144
#define ADDR_XM_TEMPERATURE         146
#define ADDR_XM_IS_MOVING           122

#define TORQUE_CONTROL_MODE                 0
#define VELOCITY_CONTROL_MODE               1
#define POSITION_CONTROL_MODE               3       // Default
#define EXTENDED_POSITION_CONTROL_MODE      4
#define CURRENT_BASED_POSITION_CONTROL_MODE 5
#define PWM_CONTROL_MODE                    16

#define TORQUE_ENABLE                       1
#define TORQUE_DISABLE                      0

namespace dynamixel_position_control
{
class DynamixelPositionControl
{
 public:
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  dxl_motor::DxlMotor *dynamixel_[];

 private:
  bool dxl_addparam_result_;
  int dxl_comm_result_;
  bool dxl_getdata_result_;

 public:
  DynamixelPositionControl(std::string device_name, float baud_rate, float protocol_version);
  ~DynamixelPositionControl();

  bool scanDynamixelId(std::vector<dxl_motor::DxlMotor *> *dynamixel);
  bool setTorque(std::vector<dxl_motor::DxlMotor *> dynamixel, bool onoff);
  bool writeGoalPosition(std::vector<dxl_motor::DxlMotor *> dynamixel, uint32_t pos_value);
  bool writeProfileVelocity(std::vector<dxl_motor::DxlMotor *> dynamixel, uint32_t vel_value);
  // SyncRead dynamixel register
  bool readRealtimeTick(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<uint16_t> *dxl_read_data);
  bool readOperatingMode(std::vector<uint8_t> dxl_id_vec, std::vector<uint16_t> *dxl_read_data);
  bool readTorque(std::vector<uint8_t> dxl_id_vec, std::vector<bool> *dxl_read_data);
  bool readGoalPosition(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data);
  bool readPresentPosition(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data);
  bool readProfileVelocity(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data);
  bool readPresentVelocity(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data);
  bool readVoltage(std::vector<uint8_t> dxl_id_vec, std::vector<uint16_t> *dxl_read_data);
  bool readTemperature(std::vector<uint8_t> dxl_id_vec, std::vector<uint8_t> *dxl_read_data);
  bool readIsMoving(std::vector<uint8_t> dxl_id_vec, std::vector<bool> *dxl_read_data);

 private:
  bool initDynamixelPositionControl(void);
  bool shutdownDynamixelPositionControl(void);
};
}

#endif // DYNAMIXEL_WORKBENCH_TOOL_H_

