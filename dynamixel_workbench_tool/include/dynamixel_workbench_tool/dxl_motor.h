#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

namespace dxl_motor
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

class DxlMotor
{
 public:
  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;
  float protocol_version_;
  std::string dynamixel_item_path_;
  std::string dynamixel_name_path_;

  std::map<std::string, ControlTableItem *> ctrl_table_;
  std::map<std::string, ControlTableItem *>::iterator it_ctrl_;
  std::map<uint32_t, uint32_t> baud_rate_table_;
  std::map<uint32_t, uint32_t>::iterator it_baud_;
  std::map<uint8_t, std::string> dxl_info_;
  std::map<uint8_t, std::string>::iterator it_dxl_;
  ControlTableItem *dxl_item_;

 public:
  DxlMotor(uint8_t id, uint16_t model_number, float protocol_version);
  DxlMotor(uint8_t id, std::string model_name, float protocol_version);
  ~DxlMotor();
  bool getModelPath(void);
  bool getModelName(uint16_t model_number);
  bool getModelItem();
};
}
#endif
