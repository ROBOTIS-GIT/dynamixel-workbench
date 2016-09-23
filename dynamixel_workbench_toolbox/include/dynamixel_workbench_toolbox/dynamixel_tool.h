#ifndef DYNAMIXEL_TOOL_H
#define DYNAMIXEL_TOOL_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>

namespace dynamixel_tool
{
enum ACCESS_TYPE {
  READ,
  READ_WRITE
};

enum MEMORY_TYPE {
  EEPROM,
  RAM
};

struct ControlTableItem
{
 std::string item_name;
 uint16_t address;
 ACCESS_TYPE access_type;
 MEMORY_TYPE memory_type;
 uint8_t data_length;
};

class DynamixelTool
{
 public:
  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;
  float protocol_version_;

  double velocity_to_value_ratio_;
  double torque_to_current_value_ratio_;
  int32_t value_of_0_radian_position_;
  int32_t value_of_min_radian_position_;
  int32_t value_of_max_radian_position_;
  double  min_radian_;
  double  max_radian_;

  std::string item_path_;
  std::string dynamixel_name_path_;

  std::map<std::string, ControlTableItem *> ctrl_table_;
  std::map<std::string, ControlTableItem *>::iterator it_ctrl_;
  std::map<uint32_t, uint32_t> baud_rate_table_;
  std::map<uint32_t, uint32_t>::iterator it_baud_;

  ControlTableItem *item_;

 public:
  DynamixelTool(uint8_t id, uint16_t model_number, float protocol_version);
  DynamixelTool(uint8_t id, std::string model_name, float protocol_version);
  ~DynamixelTool();

  bool getModelPath(void);
  bool getModelName(uint16_t model_number);
  bool getModelItem();
};
}
#endif //DYNAMIXEL_TOOL_H
