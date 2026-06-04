#include "dynamixel_general_hw/dynamixel_general_hw.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace
{
constexpr uint8_t kGoalPositionSyncWrite = 0;
constexpr uint8_t kGoalVelocitySyncWrite = 1;
constexpr uint8_t kGoalCurrentSyncWrite = 2;
constexpr double kRpmToRadPerSecond = 0.104719755;
}  // namespace

namespace dynamixel_general_hw
{

hardware_interface::CallbackReturn DynamixelGeneralHw::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = params.hardware_info;
  dxl_wb_ = std::make_unique<DynamixelWorkbench>();

  const auto port_it = info_.hardware_parameters.find("port_name");
  if (port_it == info_.hardware_parameters.end() || port_it->second.empty())
  {
    RCLCPP_ERROR(get_logger(), "Missing hardware parameter 'port_name'");
    return hardware_interface::CallbackReturn::ERROR;
  }
  port_name_ = port_it->second;

  const auto baud_it = info_.hardware_parameters.find("baud_rate");
  if (baud_it != info_.hardware_parameters.end())
  {
    try
    {
      baud_rate_ = static_cast<uint32_t>(std::stoul(baud_it->second));
    }
    catch (const std::exception &)
    {
      RCLCPP_ERROR(get_logger(), "Invalid baud_rate: '%s'", baud_it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto protocol_it = info_.hardware_parameters.find("protocol_version");
  if (protocol_it != info_.hardware_parameters.end())
  {
    double protocol_version = 0.0;
    if (!parse_double(protocol_it->second, protocol_version) ||
        (protocol_version != 1.0 && protocol_version != 2.0))
    {
      RCLCPP_ERROR(
        get_logger(), "Invalid protocol_version: '%s'. Expected 1.0 or 2.0",
        protocol_it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    protocol_version_ = static_cast<float>(protocol_version);
  }

  const auto effort_it = info_.hardware_parameters.find("calculate_effort");
  if (effort_it != info_.hardware_parameters.end())
  {
    calculate_effort_ = effort_it->second != "false" && effort_it->second != "0";
  }

  if (!parse_joints())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelGeneralHw::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!initialize_workbench() || !configure_dynamixels() || !initialize_control_items() ||
      !add_sync_write_handlers())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & joint : joints_)
  {
    joint.position_command = joint.position;
    joint.velocity_command = 0.0;
    joint.effort_command = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelGeneralHw::on_activate(
  const rclcpp_lifecycle::State &)
{
  const char * log = nullptr;
  for (const auto & joint : joints_)
  {
    if (!dxl_wb_->torqueOn(joint.id, &log))
    {
      RCLCPP_ERROR(
        get_logger(), "Failed to enable torque for %s(id=%u): %s", joint.name.c_str(), joint.id,
        log ? log : "");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelGeneralHw::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  const char * log = nullptr;
  for (const auto & joint : joints_)
  {
    if (!dxl_wb_->torqueOff(joint.id, &log))
    {
      RCLCPP_WARN(
        get_logger(), "Failed to disable torque for %s(id=%u): %s", joint.name.c_str(), joint.id,
        log ? log : "");
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelGeneralHw::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & component : info_.joints)
  {
    auto joint_it = std::find_if(
      joints_.begin(), joints_.end(),
      [&component](const Joint & joint) { return joint.name == component.name; });
    if (joint_it == joints_.end())
    {
      continue;
    }

    for (const auto & state_interface : component.state_interfaces)
    {
      double * value = state_value(*joint_it, state_interface.name);
      if (value != nullptr)
      {
        interfaces.emplace_back(component.name, state_interface.name, value);
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelGeneralHw::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & component : info_.joints)
  {
    auto joint_it = std::find_if(
      joints_.begin(), joints_.end(),
      [&component](const Joint & joint) { return joint.name == component.name; });
    if (joint_it == joints_.end())
    {
      continue;
    }

    for (const auto & command_interface : component.command_interfaces)
    {
      double * value = command_value(*joint_it, command_interface.name);
      if (value != nullptr)
      {
        interfaces.emplace_back(component.name, command_interface.name, value);
      }
    }
  }
  return interfaces;
}

hardware_interface::return_type DynamixelGeneralHw::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & joint : joints_)
  {
    if (protocol_version_ == 1.0f)
    {
      if (!read_protocol1_block(joint))
      {
        return hardware_interface::return_type::ERROR;
      }
      continue;
    }

    int32_t raw_position = 0;
    int32_t raw_velocity = 0;
    int32_t raw_current = 0;

    if (!read_item(joint.id, "Present_Position", raw_position))
    {
      return hardware_interface::return_type::ERROR;
    }
    if (!read_item(joint.id, "Present_Velocity", raw_velocity) &&
        !read_item(joint.id, "Present_Speed", raw_velocity))
    {
      return hardware_interface::return_type::ERROR;
    }

    if (!read_item(joint.id, "Present_Current", raw_current))
    {
      current_equals_load_ = true;
      if (!read_item(joint.id, "Present_Load", raw_current))
      {
        raw_current = 0;
      }
    }

    joint.position = dxl_wb_->convertValue2Radian(joint.id, raw_position);
    joint.velocity = raw_to_velocity(joint, raw_velocity);

    if (current_equals_load_)
    {
      joint.current = dxl_wb_->convertValue2Load(static_cast<int16_t>(raw_current)) / 100.0;
    }
    else
    {
      joint.current = dxl_wb_->convertValue2Current(joint.id, static_cast<int16_t>(raw_current)) /
                      1000.0;
    }
    if (calculate_effort_ && joint.torque_constant > 0.0)
    {
      joint.effort = joint.torque_constant * joint.current;
    }

    if (joint.state_temperature)
    {
      int32_t raw_temperature = 0;
      if (read_item(joint.id, "Present_Temperature", raw_temperature))
      {
        joint.temperature = static_cast<double>(raw_temperature);
      }
    }

    if (joint.state_voltage)
    {
      int32_t raw_voltage = 0;
      if (read_item(joint.id, "Present_Input_Voltage", raw_voltage) ||
          read_item(joint.id, "Present_Voltage", raw_voltage))
      {
        joint.voltage = static_cast<double>(raw_voltage) / 10.0;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

bool DynamixelGeneralHw::read_protocol1_block(Joint & joint)
{
  const char * log = nullptr;
  std::vector<uint32_t> data(read_length_);
  if (!dxl_wb_->readRegister(joint.id, read_start_addr_, read_length_, data.data(), &log))
  {
    RCLCPP_DEBUG(
      get_logger(), "Failed to block-read Dynamixel id=%u: %s", joint.id, log ? log : "");
    return false;
  }

  const auto * position_item = control_items_["Present_Position"];
  const auto * velocity_item = control_items_["Present_Velocity"];
  const auto * current_item = control_items_["Present_Current"];

  const int32_t raw_position = extract_read_value(data, position_item);
  const int32_t raw_velocity = extract_read_value(data, velocity_item);
  const int32_t raw_current = current_item == nullptr ? 0 : extract_read_value(data, current_item);

  joint.position = dxl_wb_->convertValue2Radian(joint.id, raw_position);
  joint.velocity = raw_to_velocity(joint, raw_velocity);

  if (current_item != nullptr)
  {
    if (current_equals_load_)
    {
      joint.current = dxl_wb_->convertValue2Load(static_cast<int16_t>(raw_current)) / 100.0;
    }
    else
    {
      joint.current = dxl_wb_->convertValue2Current(joint.id, static_cast<int16_t>(raw_current)) /
                      1000.0;
    }
    if (calculate_effort_ && joint.torque_constant > 0.0)
    {
      joint.effort = joint.torque_constant * joint.current;
    }
  }

  if (joint.state_temperature && control_items_["Present_Temperature"] != nullptr)
  {
    joint.temperature =
      static_cast<double>(extract_read_value(data, control_items_["Present_Temperature"]));
  }

  if (joint.state_voltage && control_items_["Present_Voltage"] != nullptr)
  {
    joint.voltage = static_cast<double>(extract_read_value(data, control_items_["Present_Voltage"])) /
                    10.0;
  }

  return true;
}

int32_t DynamixelGeneralHw::extract_read_value(
  const std::vector<uint32_t> & data, const ControlItem * item) const
{
  if (item == nullptr)
  {
    return 0;
  }

  const auto offset = item->address - read_start_addr_;
  int32_t value = 0;
  for (uint16_t i = 0; i < item->data_length; ++i)
  {
    value |= static_cast<int32_t>(data[offset + i] & 0xff) << (8 * i);
  }
  return value;
}

hardware_interface::return_type DynamixelGeneralHw::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const char * log = nullptr;

  std::vector<uint8_t> position_ids;
  std::vector<int32_t> position_values;
  std::vector<uint8_t> velocity_ids;
  std::vector<int32_t> velocity_values;
  std::vector<uint8_t> effort_ids;
  std::vector<int32_t> effort_values;

  for (const auto & joint : joints_)
  {
    if (joint.command_position && std::isfinite(joint.position_command))
    {
      position_ids.push_back(joint.id);
      position_values.push_back(dxl_wb_->convertRadian2Value(joint.id, joint.position_command));
    }

    if (joint.command_velocity && std::isfinite(joint.velocity_command))
    {
      velocity_ids.push_back(joint.id);
      velocity_values.push_back(velocity_to_raw(joint, joint.velocity_command));
    }

    if (
      joint.command_effort && current_control_supported_ && joint.torque_constant > 0.0 &&
      std::isfinite(joint.effort_command))
    {
      effort_ids.push_back(joint.id);
      effort_values.push_back(dxl_wb_->convertCurrent2Value(
        joint.id, static_cast<float>((joint.effort_command / joint.torque_constant) * 1000.0)));
    }
  }

  auto sync_write = [this, &log](
                      uint8_t handler, std::vector<uint8_t> & ids,
                      std::vector<int32_t> & values) -> bool
  {
    if (ids.empty())
    {
      return true;
    }
    return dxl_wb_->syncWrite(
      handler, ids.data(), static_cast<uint8_t>(ids.size()), values.data(), 1, &log);
  };

  if (!sync_write(kGoalPositionSyncWrite, position_ids, position_values) ||
      !sync_write(kGoalVelocitySyncWrite, velocity_ids, velocity_values) ||
      !sync_write(kGoalCurrentSyncWrite, effort_ids, effort_values))
  {
    RCLCPP_ERROR(get_logger(), "Failed to write Dynamixel command: %s", log ? log : "");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool DynamixelGeneralHw::parse_joints()
{
  joints_.clear();
  joints_.reserve(info_.joints.size());

  for (const auto & component : info_.joints)
  {
    Joint joint;
    joint.name = component.name;

    const auto id_it = component.parameters.find("id");
    if (id_it == component.parameters.end() || !parse_uint8(id_it->second, joint.id))
    {
      RCLCPP_ERROR(get_logger(), "Joint '%s' needs uint8 param 'id'", component.name.c_str());
      return false;
    }

    const auto torque_it = component.parameters.find("torque_constant");
    if (torque_it != component.parameters.end() &&
        !parse_double(torque_it->second, joint.torque_constant))
    {
      RCLCPP_ERROR(
        get_logger(), "Invalid torque_constant for joint '%s': '%s'", component.name.c_str(),
        torque_it->second.c_str());
      return false;
    }

    for (const auto & command_interface : component.command_interfaces)
    {
      if (command_interface.name == hardware_interface::HW_IF_POSITION)
      {
        joint.command_position = true;
      }
      else if (command_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        joint.command_velocity = true;
      }
      else if (command_interface.name == hardware_interface::HW_IF_EFFORT)
      {
        joint.command_effort = true;
      }
      else
      {
        RCLCPP_ERROR(
          get_logger(), "Unsupported command interface '%s' on joint '%s'",
          command_interface.name.c_str(), component.name.c_str());
        return false;
      }
    }

    for (const auto & state_interface : component.state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_CURRENT)
      {
        joint.state_current = true;
      }
      else if (state_interface.name == hardware_interface::HW_IF_TEMPERATURE)
      {
        joint.state_temperature = true;
      }
      else if (state_interface.name == "voltage")
      {
        joint.state_voltage = true;
      }
      else if (
        state_interface.name != hardware_interface::HW_IF_POSITION &&
        state_interface.name != hardware_interface::HW_IF_VELOCITY &&
        state_interface.name != hardware_interface::HW_IF_EFFORT)
      {
        RCLCPP_ERROR(
          get_logger(), "Unsupported state interface '%s' on joint '%s'",
          state_interface.name.c_str(), component.name.c_str());
        return false;
      }
    }

    joints_.push_back(joint);
  }

  if (joints_.empty())
  {
    RCLCPP_ERROR(get_logger(), "No joints configured for DynamixelGeneralHw");
    return false;
  }

  return true;
}

bool DynamixelGeneralHw::initialize_workbench()
{
  const char * log = nullptr;
  if (!dxl_wb_->init(port_name_.c_str(), baud_rate_, &log))
  {
    RCLCPP_ERROR(get_logger(), "Failed to initialize Dynamixel Workbench: %s", log ? log : "");
    return false;
  }

  if (!dxl_wb_->setPacketHandler(protocol_version_, &log))
  {
    RCLCPP_ERROR(
      get_logger(), "Failed to set Dynamixel protocol %.1f: %s", protocol_version_,
      log ? log : "");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Using Dynamixel protocol %.1f", protocol_version_);

  for (const auto & joint : joints_)
  {
    uint16_t model_number = 0;
    if (!dxl_wb_->ping(joint.id, &model_number, &log))
    {
      RCLCPP_ERROR(
        get_logger(), "Cannot find Dynamixel for joint %s(id=%u): %s", joint.name.c_str(),
        joint.id, log ? log : "");
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "Loaded Dynamixel joint %s: id=%u model_number=%u", joint.name.c_str(),
      joint.id, model_number);
  }

  return true;
}

bool DynamixelGeneralHw::configure_dynamixels()
{
  const char * log = nullptr;
  for (const auto & component : info_.joints)
  {
    const auto joint_it = std::find_if(
      joints_.begin(), joints_.end(),
      [&component](const Joint & joint) { return joint.name == component.name; });
    if (joint_it == joints_.end())
    {
      continue;
    }

    dxl_wb_->torqueOff(joint_it->id);
    for (const auto & param : component.parameters)
    {
      if (param.first == "id" || param.first == "torque_constant")
      {
        continue;
      }

      int32_t value = 0;
      if (!parse_int32(param.second, value))
      {
        RCLCPP_ERROR(
          get_logger(), "Dynamixel item param '%s' on joint '%s' is not int32: '%s'",
          param.first.c_str(), component.name.c_str(), param.second.c_str());
        return false;
      }

      if (!dxl_wb_->itemWrite(joint_it->id, param.first.c_str(), value, &log))
      {
        RCLCPP_ERROR(
          get_logger(), "Failed to write %s=%d to %s(id=%u): %s", param.first.c_str(), value,
          component.name.c_str(), joint_it->id, log ? log : "");
        return false;
      }
    }
  }

  return true;
}

bool DynamixelGeneralHw::initialize_control_items()
{
  control_items_.clear();
  current_equals_load_ = false;
  velocity_uses_direction_bit_ = false;
  const auto id = joints_.front().id;

  const ControlItem * goal_position = dxl_wb_->getItemInfo(id, "Goal_Position");
  const ControlItem * goal_velocity = dxl_wb_->getItemInfo(id, "Goal_Velocity");
  if (goal_velocity == nullptr)
  {
    goal_velocity = dxl_wb_->getItemInfo(id, "Moving_Speed");
    velocity_uses_direction_bit_ = protocol_version_ == 1.0f && goal_velocity != nullptr;
  }
  const ControlItem * goal_current = dxl_wb_->getItemInfo(id, "Goal_Current");
  const ControlItem * present_position = dxl_wb_->getItemInfo(id, "Present_Position");
  const ControlItem * present_velocity = dxl_wb_->getItemInfo(id, "Present_Velocity");
  if (present_velocity == nullptr)
  {
    present_velocity = dxl_wb_->getItemInfo(id, "Present_Speed");
  }
  const ControlItem * present_current = dxl_wb_->getItemInfo(id, "Present_Current");
  if (present_current == nullptr)
  {
    present_current = dxl_wb_->getItemInfo(id, "Present_Load");
    current_equals_load_ = true;
  }
  const ControlItem * present_temperature = dxl_wb_->getItemInfo(id, "Present_Temperature");
  const ControlItem * present_voltage = dxl_wb_->getItemInfo(id, "Present_Input_Voltage");
  if (present_voltage == nullptr)
  {
    present_voltage = dxl_wb_->getItemInfo(id, "Present_Voltage");
  }

  if (
    goal_position == nullptr || goal_velocity == nullptr || present_position == nullptr ||
    present_velocity == nullptr)
  {
    RCLCPP_ERROR(get_logger(), "Required Dynamixel control items are missing");
    return false;
  }

  current_control_supported_ = goal_current != nullptr;
  if (!current_control_supported_)
  {
    RCLCPP_WARN(get_logger(), "Goal_Current is not supported by this Dynamixel model");
  }

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;
  control_items_["Goal_Current"] = goal_current;
  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;
  control_items_["Present_Temperature"] = present_temperature;
  control_items_["Present_Voltage"] = present_voltage;

  if (protocol_version_ == 1.0f)
  {
    std::vector<const ControlItem *> read_items = {
      present_position, present_velocity, present_current, present_temperature, present_voltage};
    read_items.erase(
      std::remove(read_items.begin(), read_items.end(), nullptr), read_items.end());
    const auto minmax = std::minmax_element(
      read_items.begin(), read_items.end(),
      [](const ControlItem * lhs, const ControlItem * rhs) { return lhs->address < rhs->address; });
    read_start_addr_ = (*minmax.first)->address;
    read_length_ =
      ((*minmax.second)->address - read_start_addr_) + (*minmax.second)->data_length;
  }

  return true;
}

int32_t DynamixelGeneralHw::velocity_to_raw(const Joint & joint, double velocity)
{
  if (!velocity_uses_direction_bit_)
  {
    return dxl_wb_->convertVelocity2Value(joint.id, velocity);
  }

  const ModelInfo * model_info = dxl_wb_->getModelInfo(joint.id);
  if (model_info == nullptr || model_info->rpm <= 0.0f)
  {
    return dxl_wb_->convertVelocity2Value(joint.id, velocity);
  }

  const auto max_velocity = static_cast<double>(model_info->rpm) * kRpmToRadPerSecond;
  const auto raw_speed = static_cast<int32_t>(
    std::lround((std::abs(velocity) / max_velocity) * 1023.0));
  const auto clamped_speed = std::clamp(raw_speed, 0, 1023);

  if (clamped_speed == 0)
  {
    return 0;
  }
  return velocity < 0.0 ? clamped_speed + 1024 : clamped_speed;
}

double DynamixelGeneralHw::raw_to_velocity(const Joint & joint, int32_t raw_velocity)
{
  if (!velocity_uses_direction_bit_)
  {
    return dxl_wb_->convertValue2Velocity(joint.id, raw_velocity);
  }

  const ModelInfo * model_info = dxl_wb_->getModelInfo(joint.id);
  if (model_info == nullptr || model_info->rpm <= 0.0f)
  {
    return dxl_wb_->convertValue2Velocity(joint.id, raw_velocity);
  }

  const auto speed = raw_velocity & 0x3ff;
  const auto max_velocity = static_cast<double>(model_info->rpm) * kRpmToRadPerSecond;
  const auto velocity = (static_cast<double>(speed) / 1023.0) * max_velocity;
  return (raw_velocity & 0x400) != 0 ? -velocity : velocity;
}

bool DynamixelGeneralHw::add_sync_write_handlers()
{
  const char * log = nullptr;
  if (!dxl_wb_->addSyncWriteHandler(
        control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length,
        &log))
  {
    RCLCPP_ERROR(get_logger(), "Failed to add Goal_Position sync write handler: %s", log ? log : "");
    return false;
  }

  if (!dxl_wb_->addSyncWriteHandler(
        control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length,
        &log))
  {
    RCLCPP_ERROR(get_logger(), "Failed to add Goal_Velocity sync write handler: %s", log ? log : "");
    return false;
  }

  if (current_control_supported_)
  {
    if (!dxl_wb_->addSyncWriteHandler(
          control_items_["Goal_Current"]->address, control_items_["Goal_Current"]->data_length,
          &log))
    {
      RCLCPP_ERROR(get_logger(), "Failed to add Goal_Current sync write handler: %s", log ? log : "");
      return false;
    }
  }

  return true;
}

bool DynamixelGeneralHw::read_item(uint8_t id, const char * item, int32_t & value) const
{
  const char * log = nullptr;
  if (!dxl_wb_->readRegister(id, item, &value, &log))
  {
    RCLCPP_DEBUG(
      get_logger(), "Failed to read %s from Dynamixel id=%u: %s", item, id, log ? log : "");
    return false;
  }
  return true;
}

double * DynamixelGeneralHw::state_value(Joint & joint, const std::string & interface_name)
{
  if (interface_name == hardware_interface::HW_IF_POSITION)
  {
    return &joint.position;
  }
  if (interface_name == hardware_interface::HW_IF_VELOCITY)
  {
    return &joint.velocity;
  }
  if (interface_name == hardware_interface::HW_IF_EFFORT)
  {
    return &joint.effort;
  }
  if (interface_name == hardware_interface::HW_IF_CURRENT)
  {
    return &joint.current;
  }
  if (interface_name == hardware_interface::HW_IF_TEMPERATURE)
  {
    return &joint.temperature;
  }
  if (interface_name == "voltage")
  {
    return &joint.voltage;
  }
  return nullptr;
}

double * DynamixelGeneralHw::command_value(Joint & joint, const std::string & interface_name)
{
  if (interface_name == hardware_interface::HW_IF_POSITION)
  {
    return &joint.position_command;
  }
  if (interface_name == hardware_interface::HW_IF_VELOCITY)
  {
    return &joint.velocity_command;
  }
  if (interface_name == hardware_interface::HW_IF_EFFORT)
  {
    return &joint.effort_command;
  }
  return nullptr;
}

bool DynamixelGeneralHw::parse_uint8(const std::string & text, uint8_t & value)
{
  try
  {
    const auto parsed = std::stoul(text);
    if (parsed > std::numeric_limits<uint8_t>::max())
    {
      return false;
    }
    value = static_cast<uint8_t>(parsed);
  }
  catch (const std::exception &)
  {
    return false;
  }
  return true;
}

bool DynamixelGeneralHw::parse_double(const std::string & text, double & value)
{
  try
  {
    value = std::stod(text);
  }
  catch (const std::exception &)
  {
    return false;
  }
  return true;
}

bool DynamixelGeneralHw::parse_int32(const std::string & text, int32_t & value)
{
  try
  {
    const auto parsed = std::stol(text);
    if (
      parsed < static_cast<long>(std::numeric_limits<int32_t>::min()) ||
      parsed > static_cast<long>(std::numeric_limits<int32_t>::max()))
    {
      return false;
    }
    value = static_cast<int32_t>(parsed);
  }
  catch (const std::exception &)
  {
    return false;
  }
  return true;
}

}  // namespace dynamixel_general_hw

PLUGINLIB_EXPORT_CLASS(dynamixel_general_hw::DynamixelGeneralHw, hardware_interface::SystemInterface)
