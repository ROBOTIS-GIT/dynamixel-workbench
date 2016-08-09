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
struct ControlTableItem
{
 std::string item_name;
 uint16_t address;
 uint8_t data_length;
};

class DxlMotor
{
 public:
  uint8_t id_;
  uint16_t model_number_;
  std::string dynamixel_item_path_;

  float protocol_version_;
  std::string model_name_;

  ControlTableItem *realtime_tick_item;
  ControlTableItem *operating_mode_item;
  ControlTableItem *torque_enable_item;
  ControlTableItem *goal_position_item;
  ControlTableItem *goal_velocity_item;
  ControlTableItem *present_position_item;
  ControlTableItem *profile_velocity_item;
  ControlTableItem *present_velocity_item;
  ControlTableItem *present_input_voltage_item;
  ControlTableItem *present_temperature_item;
  ControlTableItem *is_moving_item;

 public:
  DxlMotor(uint8_t id = 1, uint16_t model_number = 0, float protocol_version = 2.0);
  ~DxlMotor();
  bool getPath(void);
  bool getModelName(uint16_t model_number);
  bool getModelItem();
};
}
#endif
