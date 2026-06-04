#ifndef DYNAMIXEL_GENERAL_HW__DYNAMIXEL_GENERAL_HW_HPP_
#define DYNAMIXEL_GENERAL_HW__DYNAMIXEL_GENERAL_HW_HPP_

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dynamixel_general_hw
{

class DynamixelGeneralHw : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelGeneralHw)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    uint8_t id = 0;
    double torque_constant = 0.0;

    double position = std::numeric_limits<double>::quiet_NaN();
    double velocity = std::numeric_limits<double>::quiet_NaN();
    double effort = 0.0;
    double current = 0.0;
    double temperature = 0.0;
    double voltage = 0.0;

    double position_command = std::numeric_limits<double>::quiet_NaN();
    double velocity_command = std::numeric_limits<double>::quiet_NaN();
    double effort_command = std::numeric_limits<double>::quiet_NaN();

    bool command_position = false;
    bool command_velocity = false;
    bool command_effort = false;
    bool state_current = false;
    bool state_temperature = false;
    bool state_voltage = false;
  };

  bool parse_joints();
  bool initialize_workbench();
  bool configure_dynamixels();
  bool initialize_control_items();
  bool add_sync_write_handlers();
  bool read_protocol1_block(Joint & joint);
  bool read_item(uint8_t id, const char * item, int32_t & value) const;
  int32_t extract_read_value(const std::vector<uint32_t> & data, const ControlItem * item) const;
  int32_t velocity_to_raw(const Joint & joint, double velocity);
  double raw_to_velocity(const Joint & joint, int32_t raw_velocity);
  double * state_value(Joint & joint, const std::string & interface_name);
  double * command_value(Joint & joint, const std::string & interface_name);
  static bool parse_uint8(const std::string & text, uint8_t & value);
  static bool parse_double(const std::string & text, double & value);
  static bool parse_int32(const std::string & text, int32_t & value);

  std::unique_ptr<DynamixelWorkbench> dxl_wb_;
  hardware_interface::HardwareInfo info_;
  std::vector<Joint> joints_;
  std::unordered_map<std::string, const ControlItem *> control_items_;

  std::string port_name_;
  uint32_t baud_rate_ = 57600;
  uint16_t read_start_addr_ = 0;
  uint16_t read_length_ = 0;
  float protocol_version_ = 2.0f;
  bool calculate_effort_ = true;
  bool current_equals_load_ = false;
  bool current_control_supported_ = true;
  bool velocity_uses_direction_bit_ = false;
};

}  // namespace dynamixel_general_hw

#endif  // DYNAMIXEL_GENERAL_HW__DYNAMIXEL_GENERAL_HW_HPP_
