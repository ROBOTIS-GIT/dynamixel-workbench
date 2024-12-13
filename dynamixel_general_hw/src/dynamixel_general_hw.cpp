#include "dynamixel_general_hw/dynamixel_general_hw.h"

namespace dynamixel_general_hw
{

DynamixelGeneralHw::DynamixelGeneralHw()
  : pnh_("~")
  , is_servo_raw_(true)
  , is_servo_(true)
  , prev_is_servo_(true)
  , is_hold_pos_raw_(false)
  , is_hold_pos_(false)
  , prev_is_hold_pos_(false)
  , is_current_ctrl_(true)
  , is_current_eq_load_(false)
  , last_write_tm_(0)
{
  is_calc_effort_ = pnh_.param<bool>("calculate_effort", true);
  is_pub_temp_ = pnh_.param<bool>("publish_temperature", true);
  is_pub_volt_ = pnh_.param<bool>("publish_input_voltage", true);
  write_read_interval_ = pnh_.param<double>("write_read_interval", -1);

  dxl_wb_.reset(new DynamixelWorkbench);
}

// Override resource conflict check (https://github.com/ros-controls/ros_control/blob/0.20.0/hardware_interface/include/hardware_interface/robot_hw.h#L113-L146)
// to accept position and effort commands simultaneously for Current-based Position Control Mode of Dynamixel without creating new JointInterface.
// https://robotics.stackexchange.com/questions/65092/using-both-jointtrajectorycontroller-and-jointpositioncontroller-at-same-time-in/65093#65093
bool DynamixelGeneralHw::checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const
{
  // Map from resource name to all controllers claiming it
  std::map<std::string, std::list<hardware_interface::ControllerInfo>> resource_map;

  // Populate a map of all controllers claiming individual resources.
  // We do this by iterating over every claimed resource of every hardware interface used by every controller
  for (const auto& controller_info : info)
  {
    for (const auto& claimed_resource : controller_info.claimed_resources)
    {
      for (const auto& iface_resource : claimed_resource.resources)
      {
        hardware_interface::ControllerInfo unique_info;
        unique_info.name = controller_info.name;
        unique_info.type = controller_info.type;
        unique_info.claimed_resources.push_back(hardware_interface::InterfaceResources());
        unique_info.claimed_resources[0].hardware_interface = claimed_resource.hardware_interface;
        unique_info.claimed_resources[0].resources.insert(iface_resource);
        resource_map[iface_resource].push_back(unique_info);
      }
    }
  }

  // Enforce resource exclusivity policy: No resource can be claimed by more than one controller
  bool in_conflict = false;
  for (const auto& resource_name_and_claiming_controllers : resource_map)
  {
    if (resource_name_and_claiming_controllers.second.size() > 1)
    {
      bool prev_in_conflict = in_conflict;

      std::string controller_list;
      for (const auto& controller : resource_name_and_claiming_controllers.second)
        controller_list += controller.name + ", ";
      ROS_WARN("Resource conflict on [%s].  Controllers = [%s]", resource_name_and_claiming_controllers.first.c_str(), controller_list.c_str());
      in_conflict = true;

      // Accept position and effort commands simultaneously for Current-based Position Control Mode of Dynamixel
      // with common JointInterface (hardware_interface::PositionJointInterface/EffortJointInterface)
      const auto& rnacc = resource_name_and_claiming_controllers;
      if (rnacc.second.size() == 2 &&
          std::find_if(rnacc.second.begin(), rnacc.second.end(),
                       [](hardware_interface::ControllerInfo ci) {
                         return (ci.claimed_resources[0].hardware_interface ==
                                 "hardware_interface::PositionJointInterface");
                       }) != rnacc.second.end() &&
          std::find_if(rnacc.second.begin(), rnacc.second.end(),
                       [](hardware_interface::ControllerInfo ci) {
                         return (ci.claimed_resources[0].hardware_interface ==
                                 "hardware_interface::EffortJointInterface");
                       }) != rnacc.second.end())
      {
        ROS_WARN_STREAM("However, this conflict is accepted because "
                        << rnacc.second.front().name << " uses "
                        << rnacc.second.front().claimed_resources[0].hardware_interface << " and "
                        << rnacc.second.back().name << " uses "
                        << rnacc.second.back().claimed_resources[0].hardware_interface
                        << ". Commands from both interfaces are used in Current-based Position Control Mode of "
                           "Dynamixel");
        in_conflict = prev_in_conflict;
      }
    }
  }

  return in_conflict;
}

bool DynamixelGeneralHw::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  return result;
}

bool DynamixelGeneralHw::getDynamixelsInfo(XmlRpc::XmlRpcValue& dxl_info)
{
  if (dxl_info.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("dynamixel_info is not dictionary");
    return false;
  }
  for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator it_info = dxl_info.begin();
       it_info != dxl_info.end(); it_info++)
  {
    std::string name = it_info->first;
    if (name.size() == 0)
    {
      continue;
    }

    XmlRpc::XmlRpcValue& item = it_info->second;
    for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator it_item = item.begin();
         it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first;

      if (item_name == "torque_constant")
      {
        torque_consts_[name] = it_item->second;
      }
      else
      {
        int32_t value = it_item->second;

        if (item_name == "ID")
        {
          dynamixel_[name] = value;
        }

        ItemValue item_value = {item_name, value};
        std::pair<std::string, ItemValue> info(name, item_value);

        dynamixel_info_.push_back(info);
      }
    }
  }

  return true;
}

bool DynamixelGeneralHw::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}

bool DynamixelGeneralHw::initDynamixels(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for (auto const& info:dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }

    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}

bool DynamixelGeneralHw::initControlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == NULL)
  {
    return false;
  }

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL)
  {
    goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  }
  if (goal_velocity == NULL)
  {
    return false;
  }

  const ControlItem *goal_current = dxl_wb_->getItemInfo(it->second, "Goal_Current");
  if (goal_current == NULL)
  {
    ROS_WARN("Effort command to %s is currently not supported", dxl_wb_->getModelName(it->second));
    is_current_ctrl_ = false;
  }

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL)
  {
    return false;
  }

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL)
  {
    present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  }
  if (present_velocity == NULL)
  {
    return false;
  }

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL)
  {
    present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
    is_current_eq_load_ = true;
  }
  if (present_current == NULL)
  {
    is_current_eq_load_ = false;
    return false;
  }

  const ControlItem *present_temperature = dxl_wb_->getItemInfo(it->second, "Present_Temperature");
  if (present_temperature == NULL)
  {
    return false;
  }

  const ControlItem *present_input_voltage = dxl_wb_->getItemInfo(it->second, "Present_Input_Voltage");
  if (present_input_voltage == NULL)
  {
    present_input_voltage = dxl_wb_->getItemInfo(it->second, "Present_Voltage");
  }
  if (present_input_voltage == NULL)
  {
    return false;
  }

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;
  control_items_["Goal_Current"] = goal_current;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;
  control_items_["Present_Temperature"] = present_temperature;
  control_items_["Present_Input_Voltage"] = present_input_voltage;

  return true;
}

bool DynamixelGeneralHw::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  if (is_current_ctrl_)
  {
    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Current"]->address, control_items_["Goal_Current"]->data_length, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
    else
    {
      ROS_INFO("%s", log);
    }
  }

  std::vector<std::string> target_items = {"Present_Position", "Present_Velocity", "Present_Current"};
  if (is_pub_temp_)
  {
    target_items.push_back("Present_Temperature");
  }
  if (is_pub_volt_)
  {
    target_items.push_back("Present_Input_Voltage");
  }
  std::vector<uint16_t> target_addrs(target_items.size());
  std::transform(target_items.begin(), target_items.end(), target_addrs.begin(),
                 [this](std::string x) { return control_items_[x]->address; } );

  std::vector<uint16_t>::iterator min_addr = std::min_element(target_addrs.begin(), target_addrs.end());
  std::vector<uint16_t>::iterator max_addr = std::max_element(target_addrs.begin(), target_addrs.end());
  int max_addr_idx = std::distance(target_addrs.begin(), max_addr);
  read_start_addr_ = *min_addr;
  read_length_ = (*max_addr - *min_addr) + control_items_[target_items[max_addr_idx]]->data_length;

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    result = dxl_wb_->addSyncReadHandler(read_start_addr_, read_length_, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
  }

  return result;
}

bool DynamixelGeneralHw::initRosInterface(void)
{
  // Register actuator interfaces to transmission loader
  actr_names_.clear();
  for (const std::pair<std::string, uint32_t>& dxl : dynamixel_)
  {
    actr_names_.push_back(dxl.first);
  }
  actr_curr_pos_.resize(actr_names_.size(), 0);
  actr_curr_vel_.resize(actr_names_.size(), 0);
  actr_curr_eff_.resize(actr_names_.size(), 0);
  actr_cmd_pos_.resize(actr_names_.size(), 0);
  actr_cmd_vel_.resize(actr_names_.size(), 0);
  actr_cmd_eff_.resize(actr_names_.size(), 0);
  for (int i = 0; i < actr_names_.size(); i++)
  {
    hardware_interface::ActuatorStateHandle state_handle(actr_names_[i], &actr_curr_pos_[i], &actr_curr_vel_[i], &actr_curr_eff_[i]);
    actr_state_interface_.registerHandle(state_handle);

    hardware_interface::ActuatorHandle position_handle(state_handle, &actr_cmd_pos_[i]);
    pos_actr_interface_.registerHandle(position_handle);
    hardware_interface::ActuatorHandle velocity_handle(state_handle, &actr_cmd_vel_[i]);
    vel_actr_interface_.registerHandle(velocity_handle);
    hardware_interface::ActuatorHandle effort_handle(state_handle, &actr_cmd_eff_[i]);
    eff_actr_interface_.registerHandle(effort_handle);
  }
  registerInterface(&actr_state_interface_);
  registerInterface(&pos_actr_interface_);
  registerInterface(&vel_actr_interface_);
  registerInterface(&eff_actr_interface_);

  // Initialize transmission loader
  try
  {
    transmission_loader_.reset(new transmission_interface::TransmissionInterfaceLoader(this, &robot_transmissions_));
  }
  catch (const std::invalid_argument& ex)
  {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  }
  catch (const pluginlib::LibraryLoadException& ex)
  {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
    return false;
  }

  // Load URDF from parameter
  std::string urdf_string;
  nh_.getParam("robot_description", urdf_string);
  while (urdf_string.empty() && ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE(10, "Waiting for robot_description...");
    nh_.getParam("robot_description", urdf_string);
    ros::Duration(0.1).sleep();
  }
  if (ros::ok())
  {
    ROS_INFO_STREAM("Got robot_description");
  }
  else
  {
    return false;
  }

  // Extract transmission infos from URDF
  transmission_interface::TransmissionParser parser;
  std::vector<transmission_interface::TransmissionInfo> infos;
  if (!parser.parse(urdf_string, infos))
  {
    ROS_ERROR("Error parsing URDF");
    return false;
  }

  // Load transmissions composed of target actuators
  jnt_names_.clear();
  for (const transmission_interface::TransmissionInfo& info : infos)
  {
    if (std::find(actr_names_.begin(), actr_names_.end(), info.actuators_[0].name_) != actr_names_.end())
    {
      for (const transmission_interface::ActuatorInfo& actuator : info.actuators_)
      {
        if (std::find(actr_names_.begin(), actr_names_.end(), actuator.name_) == actr_names_.end())
        {
          ROS_ERROR_STREAM("Error loading transmission: " << info.name_);
          ROS_ERROR_STREAM("Cannot find " << actuator.name_ << " in target actuators");
          return false;
        }
      }
      if (!transmission_loader_->load(info))
      {
        ROS_ERROR_STREAM("Error loading transmission: " << info.name_);
        return false;
      }
      ROS_INFO_STREAM("Loaded transmission: " << info.name_);
      for (const transmission_interface::JointInfo& joint : info.joints_)
      {
        jnt_names_.push_back(joint.name_);
      }
    }
  }

  // Prepare joint limits interfaces
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_string))
  {
    ROS_ERROR("Error parsing URDF");
    return false;
  }
  hardware_interface::PositionJointInterface* pos_if = get<hardware_interface::PositionJointInterface>();
  hardware_interface::VelocityJointInterface* vel_if = get<hardware_interface::VelocityJointInterface>();
  hardware_interface::EffortJointInterface* eff_if = get<hardware_interface::EffortJointInterface>();
  for (const std::string& jnt_name : jnt_names_)
  {
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_limits_urdf = getJointLimits(urdf_model.getJoint(jnt_name), limits);
    bool has_limits_param = getJointLimits(jnt_name, nh_, limits);
    if (has_limits_urdf || has_limits_param)
    {
      if (pos_if)
      {
        joint_limits_interface::PositionJointSaturationHandle pos_sat_handle(pos_if->getHandle(jnt_name), limits);
        pos_jnt_sat_interface_.registerHandle(pos_sat_handle);
      }
      if (vel_if)
      {
        joint_limits_interface::VelocityJointSaturationHandle vel_sat_handle(vel_if->getHandle(jnt_name), limits);
        vel_jnt_sat_interface_.registerHandle(vel_sat_handle);
      }
      if (eff_if)
      {
        joint_limits_interface::EffortJointSaturationHandle eff_sat_handle(eff_if->getHandle(jnt_name), limits);
        eff_jnt_sat_interface_.registerHandle(eff_sat_handle);
      }
    }
    has_limits_urdf = getSoftJointLimits(urdf_model.getJoint(jnt_name), soft_limits);
    has_limits_param = getSoftJointLimits(jnt_name, nh_, soft_limits);
    if (has_limits_urdf || has_limits_param)
    {
      if (pos_if)
      {
        joint_limits_interface::PositionJointSoftLimitsHandle pos_soft_handle(pos_if->getHandle(jnt_name), limits,
                                                                              soft_limits);
        pos_jnt_soft_interface_.registerHandle(pos_soft_handle);
      }
      if (vel_if)
      {
        joint_limits_interface::VelocityJointSoftLimitsHandle vel_soft_handle(vel_if->getHandle(jnt_name), limits,
                                                                              soft_limits);
        vel_jnt_soft_interface_.registerHandle(vel_soft_handle);
      }
      if (eff_if)
      {
        joint_limits_interface::EffortJointSoftLimitsHandle eff_soft_handle(eff_if->getHandle(jnt_name), limits,
                                                                            soft_limits);
        eff_jnt_soft_interface_.registerHandle(eff_soft_handle);
      }
    }
  }

  // Initialize E-stop interface
  servo_sub_ = pnh_.subscribe("servo", 1, &DynamixelGeneralHw::servoCallback, this);
  hold_pos_sub_ = pnh_.subscribe("hold_position", 1, &DynamixelGeneralHw::holdPosCallback, this);

  // Initialize dynamixel-specific interfaces
  dynamixel_state_pub_ = pnh_.advertise<dynamixel_general_hw::DynamixelStateList>("dynamixel_state", 100);
  dynamixel_cmd_srv_ = pnh_.advertiseService("dynamixel_command", &DynamixelGeneralHw::dynamixelCmdCallback, this);

  // Start spinning
  nh_.setCallbackQueue(&subscriber_queue_);
  subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
  subscriber_spinner_->start();

  return true;
}

void DynamixelGeneralHw::cleanup(void)
{
  subscriber_spinner_->stop();
}

void DynamixelGeneralHw::readDynamixelState(void)
{
  bool result = false;
  const char* log = NULL;

  dynamixel_general_hw::DynamixelState dynamixel_state[dynamixel_.size()];
  dynamixel_state_list_.dynamixel_state.clear();

  int32_t get_current[dynamixel_.size()];
  int32_t get_velocity[dynamixel_.size()];
  int32_t get_position[dynamixel_.size()];
  int32_t get_temperature[dynamixel_.size()];
  int32_t get_voltage[dynamixel_.size()];

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  for (auto const& dxl:dynamixel_)
  {
    dynamixel_state[id_cnt].name = dxl.first;
    dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

  if (write_read_interval_ > 0 && last_write_tm_ > ros::Time(0))
  {
    // Sleep until write_read_interval_ is accomplished
    const ros::Time now = ros::Time::now();
    if (now > last_write_tm_)
    {
      double sleep_dur = write_read_interval_ - (now - last_write_tm_).toSec();
      if (sleep_dur > 0)
      {
        ros::Duration(sleep_dur).sleep();
      }
    }
  }
  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_INFO,
                               id_array,
                               dynamixel_.size(),
                               &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_INFO,
                                      id_array,
                                      id_cnt,
                                      control_items_["Present_Current"]->address,
                                      control_items_["Present_Current"]->data_length,
                                      get_current,
                                      &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_INFO,
                                      id_array,
                                      id_cnt,
                                      control_items_["Present_Velocity"]->address,
                                      control_items_["Present_Velocity"]->data_length,
                                      get_velocity,
                                      &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_INFO,
                                      id_array,
                                      id_cnt,
                                      control_items_["Present_Position"]->address,
                                      control_items_["Present_Position"]->data_length,
                                      get_position,
                                      &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    if (is_pub_temp_)
    {
      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_INFO,
                                        id_array,
                                        id_cnt,
                                        control_items_["Present_Temperature"]->address,
                                        control_items_["Present_Temperature"]->data_length,
                                        get_temperature,
                                        &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }
    }

    if (is_pub_volt_)
    {
      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_INFO,
                                        id_array,
                                        id_cnt,
                                        control_items_["Present_Input_Voltage"]->address,
                                        control_items_["Present_Input_Voltage"]->data_length,
                                        get_voltage,
                                        &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }
    }

    for(uint8_t index = 0; index < id_cnt; index++)
    {
      dynamixel_state[index].present_current = get_current[index];
      dynamixel_state[index].present_velocity = get_velocity[index];
      dynamixel_state[index].present_position = get_position[index];
      if (is_pub_temp_)
      {
        dynamixel_state[index].present_temperature = get_temperature[index];
      }
      if (is_pub_volt_)
      {
        dynamixel_state[index].present_input_voltage = get_voltage[index];
      }

      dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index]);
    }
  }
  else if(dxl_wb_->getProtocolVersion() == 1.0f)
  {
    uint32_t get_all_data[read_length_];
    uint8_t dxl_cnt = 0;
    for (auto const& dxl:dynamixel_)
    {
      result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                     read_start_addr_,
                                     read_length_,
                                     get_all_data,
                                     &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      uint16_t idx;
      uint16_t len;
      idx = control_items_["Present_Current"]->address - read_start_addr_;
      len = control_items_["Present_Current"]->data_length;
      if (len == 1)
      {
        dynamixel_state[dxl_cnt].present_current = get_all_data[idx];
      }
      else if (len == 2)
      {
        dynamixel_state[dxl_cnt].present_current = DXL_MAKEWORD(get_all_data[idx], get_all_data[idx + 1]);
      }
      else if (len == 4)
      {
        dynamixel_state[dxl_cnt].present_current =
            DXL_MAKEDWORD(DXL_MAKEWORD(get_all_data[idx], get_all_data[idx + 1]),
                          DXL_MAKEWORD(get_all_data[idx + 2], get_all_data[idx + 3]));
      }
      idx = control_items_["Present_Velocity"]->address - read_start_addr_;
      len = control_items_["Present_Velocity"]->data_length;
      if (len == 1)
      {
        dynamixel_state[dxl_cnt].present_velocity = get_all_data[idx];
      }
      else if (len == 2)
      {
        dynamixel_state[dxl_cnt].present_velocity = DXL_MAKEWORD(get_all_data[idx], get_all_data[idx + 1]);
      }
      else if (len == 4)
      {
        dynamixel_state[dxl_cnt].present_velocity =
            DXL_MAKEDWORD(DXL_MAKEWORD(get_all_data[idx], get_all_data[idx + 1]),
                          DXL_MAKEWORD(get_all_data[idx + 2], get_all_data[idx + 3]));
      }
      idx = control_items_["Present_Position"]->address - read_start_addr_;
      len = control_items_["Present_Position"]->data_length;
      if (len == 1)
      {
        dynamixel_state[dxl_cnt].present_position = get_all_data[idx];
      }
      else if (len == 2)
      {
        dynamixel_state[dxl_cnt].present_position = DXL_MAKEWORD(get_all_data[idx], get_all_data[idx + 1]);
      }
      else if (len == 4)
      {
        dynamixel_state[dxl_cnt].present_position =
            DXL_MAKEDWORD(DXL_MAKEWORD(get_all_data[idx], get_all_data[idx + 1]),
                          DXL_MAKEWORD(get_all_data[idx + 2], get_all_data[idx + 3]));
      }

      if (is_pub_temp_)
      {
        idx = control_items_["Present_Temperature"]->address - read_start_addr_;
        len = control_items_["Present_Temperature"]->data_length;
        if (len == 1)
        {
          dynamixel_state[dxl_cnt].present_temperature = get_all_data[idx];
        }
        else if (len == 2)
        {
          dynamixel_state[dxl_cnt].present_temperature = DXL_MAKEWORD(get_all_data[idx], get_all_data[idx + 1]);
        }
        else if (len == 4)
        {
          dynamixel_state[dxl_cnt].present_temperature =
              DXL_MAKEDWORD(DXL_MAKEWORD(get_all_data[idx], get_all_data[idx + 1]),
                            DXL_MAKEWORD(get_all_data[idx + 2], get_all_data[idx + 3]));
        }
      }

      if (is_pub_volt_)
      {
        idx = control_items_["Present_Input_Voltage"]->address - read_start_addr_;
        len = control_items_["Present_Input_Voltage"]->data_length;
        if (len == 1)
        {
          dynamixel_state[dxl_cnt].present_input_voltage = get_all_data[idx];
        }
        else if (len == 2)
        {
          dynamixel_state[dxl_cnt].present_input_voltage = DXL_MAKEWORD(get_all_data[idx], get_all_data[idx + 1]);
        }
        else if (len == 4)
        {
          dynamixel_state[dxl_cnt].present_input_voltage =
              DXL_MAKEDWORD(DXL_MAKEWORD(get_all_data[idx], get_all_data[idx + 1]),
                            DXL_MAKEWORD(get_all_data[idx + 2], get_all_data[idx + 3]));
        }
      }

      dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[dxl_cnt]);
      dxl_cnt++;
    }
  }
  dynamixel_state_list_.header.stamp = ros::Time::now();
}

void DynamixelGeneralHw::read(void)
{
  std::lock_guard<std::mutex> lock(mtx_);

  // Read current dynamixel state
  readDynamixelState();
  dynamixel_state_pub_.publish(dynamixel_state_list_);

  // Convert dynamixel state to ros_control actuator state
  uint8_t id_cnt = 0;
  for (const std::pair<std::string, uint32_t>& dxl : dynamixel_)
  {
    double torque_const = torque_consts_[dxl.first];
    if (is_calc_effort_ && torque_const > 0)
    {
      double current = 0;
      if (is_current_eq_load_)
      {
        current = dxl_wb_->convertValue2Load((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
        current /= 100.0;  // % -> dimensionless
      }
      else
      {
        current = dxl_wb_->convertValue2Current((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
        current /= 1000.0;  // mA -> A
      }
      actr_curr_eff_[id_cnt] = torque_const * current;
    }

    actr_curr_vel_[id_cnt] = dxl_wb_->convertValue2Velocity((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_velocity);
    actr_curr_pos_[id_cnt] = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_position);

    id_cnt++;
  }

  // Propagate current actuator state to joints
  if (robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>())
  {
    robot_transmissions_.get<transmission_interface::ActuatorToJointStateInterface>()->propagate();
  }

  // Update E-stop status
  is_servo_ = is_servo_raw_;
  is_hold_pos_ = is_hold_pos_raw_;
}

void DynamixelGeneralHw::write(const ros::Time& time, const ros::Duration& period)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (is_servo_)
  {
    // Servo off -> on
    if (!prev_is_servo_)
    {
      for (const std::pair<std::string, uint32_t>& dxl : dynamixel_)
      {
        dxl_wb_->torqueOn((uint8_t)dxl.second);
      }
    }

    if (robot_transmissions_.get<transmission_interface::JointToActuatorPositionInterface>())
    {
      // Update & send actuator position commands only when is_hold_pos_ is false.
      // This holds current positions of position-controlled actuators while is_hold_pos_ is true
      if (!is_hold_pos_)
      {
        // Enforce joint position limits
        pos_jnt_sat_interface_.enforceLimits(period);
        pos_jnt_soft_interface_.enforceLimits(period);

        // Propagate joint position commands to actuators
        robot_transmissions_.get<transmission_interface::JointToActuatorPositionInterface>()->propagate();

        // Convert ros_control actuator position command to dynamixel command
        std::vector<uint8_t> pos_id_vec;
        std::vector<int32_t> dxl_pos_vec;
        uint8_t id_cnt = 0;
        for (const std::pair<std::string, uint32_t>& dxl : dynamixel_)
        {
          if (std::isnan(actr_cmd_pos_[id_cnt]))
          {
            ROS_DEBUG_STREAM_DELAYED_THROTTLE(10, "Skipping position command to " << dxl.first
                                                                                  << " because it is NaN. Its "
                                                                                     "controller may not work");
          }
          else
          {
            pos_id_vec.push_back((uint8_t)dxl.second);
            dxl_pos_vec.push_back(dxl_wb_->convertRadian2Value((uint8_t)dxl.second, actr_cmd_pos_[id_cnt]));
          }
          id_cnt++;
        }

        // Write position command to dynamixel
        if (pos_id_vec.size() > 0)
        {
          bool result = false;
          const char* log = NULL;
          uint8_t pos_id[pos_id_vec.size()];
          int32_t dxl_pos[dxl_pos_vec.size()];
          std::copy(pos_id_vec.begin(), pos_id_vec.end(), pos_id);
          std::copy(dxl_pos_vec.begin(), dxl_pos_vec.end(), dxl_pos);
          result =
              dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, pos_id, pos_id_vec.size(), dxl_pos, 1, &log);
          if (result == false)
          {
            ROS_ERROR("%s", log);
          }
          last_write_tm_ = ros::Time::now();
        }
      }
    }
    if (robot_transmissions_.get<transmission_interface::JointToActuatorVelocityInterface>())
    {
      // Update actuator velocity commands only when is_hold_pos_ is false.
      // This keeps which actuator is valid (having non-NaN command) while is_hold_pos_ is true
      if (!is_hold_pos_)
      {
        // Enforce joint velocity limits
        vel_jnt_sat_interface_.enforceLimits(period);
        vel_jnt_soft_interface_.enforceLimits(period);

        // Propagate joint velocity commands to actuators
        robot_transmissions_.get<transmission_interface::JointToActuatorVelocityInterface>()->propagate();
      }

      // Convert ros_control actuator velocity command to dynamixel command
      std::vector<uint8_t> vel_id_vec;
      std::vector<int32_t> dxl_vel_vec;
      uint8_t id_cnt = 0;
      for (const std::pair<std::string, uint32_t>& dxl : dynamixel_)
      {
        if (std::isnan(actr_cmd_vel_[id_cnt]))
        {
          ROS_DEBUG_STREAM_DELAYED_THROTTLE(10, "Skipping velocity command to " << dxl.first
                                                                                << " because it is NaN. Its "
                                                                                   "controller may not work");
        }
        else
        {
          if (is_hold_pos_)
          {
            // Forcibly set velocity commands of valid actuators to 0 when is_hold_pos_ is true.
            // This holds current positions of velocity-controlled actuators while is_hold_pos_ is true
            actr_cmd_vel_[id_cnt] = 0;
          }
          uint8_t id = (uint8_t)dxl.second;
          vel_id_vec.push_back(id);
          const char* model_name = NULL;
          model_name = dxl_wb_->getModelName(id);
          if (dxl_wb_->getProtocolVersion() == 2.0f && strcmp(model_name, "XL-320") != 0)
          {
            dxl_vel_vec.push_back(dxl_wb_->convertVelocity2Value((uint8_t)dxl.second, actr_cmd_vel_[id_cnt]));
          }
          else if ((dxl_wb_->getProtocolVersion() == 2.0f && strcmp(model_name, "XL-320") == 0) ||
                   (dxl_wb_->getProtocolVersion() == 1.0f && (strncmp(model_name, "AX", strlen("AX")) == 0 ||
                                                              strncmp(model_name, "RX", strlen("RX")) == 0 ||
                                                              strncmp(model_name, "EX", strlen("EX")) == 0 ||
                                                              strncmp(model_name, "MX", strlen("MX")) == 0)))
          {
            // In this case, convertVelocity2Value returns a value with the wrong sign, so we cannot use this method
            const ModelInfo* model_info = NULL;
            model_info = dxl_wb_->getModelInfo(id);
            int32_t value = 0;
            double velocity = actr_cmd_vel_[id_cnt];
            const float RPM2RADPERSEC = 0.104719755f;
            if (velocity == 0)
            {
              value = 0;
            }
            else if (velocity < 0)
            {
              value = ((velocity * -1) / (model_info->rpm * RPM2RADPERSEC)) + 1023;
            }
            else if (velocity > 0)
            {
              value = (velocity / (model_info->rpm * RPM2RADPERSEC));
            }
            dxl_vel_vec.push_back(value);
          }
          else
          {
            dxl_vel_vec.push_back(0);
          }
        }
        id_cnt++;
      }

      // Write velocity command to dynamixel
      if (vel_id_vec.size() > 0)
      {
        bool result = false;
        const char* log = NULL;
        uint8_t vel_id[vel_id_vec.size()];
        int32_t dxl_vel[dxl_vel_vec.size()];
        std::copy(vel_id_vec.begin(), vel_id_vec.end(), vel_id);
        std::copy(dxl_vel_vec.begin(), dxl_vel_vec.end(), dxl_vel);
        result =
            dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, vel_id, vel_id_vec.size(), dxl_vel, 1, &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
        }
        last_write_tm_ = ros::Time::now();
      }
    }
    if (is_calc_effort_ && is_current_ctrl_ &&
        robot_transmissions_.get<transmission_interface::JointToActuatorEffortInterface>())
    {
      // Update actuator effort commands only when is_hold_pos_ is false.
      // This keeps which actuator is valid (having non-NaN command) while is_hold_pos_ is true
      if (!is_hold_pos_)
      {
        // Just after is_hold_pos_ becomes false, we must restore normal Operating Mode for normal execution
        if (prev_is_hold_pos_)
        {
          for (const std::pair<std::string, std::map<std::string, int32_t>>& mode : normal_modes_)
          {
            dxl_wb_->torqueOff(dynamixel_[mode.first]);  // We cannot change Operating Mode while torque on
            bool result = true;
            const char* log = NULL;
            for (const std::pair<std::string, int32_t>& addr_val : mode.second)
            {
              if (!dxl_wb_->writeRegister(dynamixel_[mode.first], addr_val.first.c_str(), addr_val.second, &log))
              {
                result = false;
                break;
              }
            }
            if (result == false)
            {
              ROS_ERROR("%s", log);
            }
            dxl_wb_->torqueOn(dynamixel_[mode.first]);
          }
          normal_modes_.clear();
        }

        // Enforce joint effort limits
        eff_jnt_sat_interface_.enforceLimits(period);
        eff_jnt_soft_interface_.enforceLimits(period);

        // Propagate joint effort commands to actuators
        robot_transmissions_.get<transmission_interface::JointToActuatorEffortInterface>()->propagate();
      }

      // Convert ros_control actuator effort command to dynamixel command
      std::vector<uint8_t> eff_id_vec;
      std::vector<int32_t> dxl_eff_vec;
      uint8_t id_cnt = 0;
      for (const std::pair<std::string, uint32_t>& dxl : dynamixel_)
      {
        double torque_const = torque_consts_[dxl.first];
        if (torque_const > 0)
        {
          if (std::isnan(actr_cmd_eff_[id_cnt]))
          {
            ROS_DEBUG_STREAM_DELAYED_THROTTLE(10, "Skipping effort command to " << dxl.first
                                                                                << " because it is NaN. Its "
                                                                                   "controller may not work");
          }
          else
          {
            if (!is_hold_pos_)
            {
              eff_id_vec.push_back((uint8_t)dxl.second);
              dxl_eff_vec.push_back(
                  dxl_wb_->convertCurrent2Value((uint8_t)dxl.second, (actr_cmd_eff_[id_cnt] / torque_const) * 1000));
            }
            else
            {
              // Just after is_hold_pos_ becomes true, save normal Operating Mode of valid actuators and switch to Velocity Control.
              // While is_hold_pos_ is true, leave their velocity commands initial values (zero).
              // This holds current positions of actuators while is_hold_pos_ is true.
              // We avoid keeping Current Control because it requires PID loop to hold current positions.
              // We do not select Position Control because it resets Present Position within one full rotation.
              // We do not select Extended Position Control because some actuators do not have this mode
              if (!prev_is_hold_pos_)
              {
                bool result = false;
                const char* log = NULL;
                int32_t data[2];
                if (dxl_wb_->readRegister((uint8_t)dxl.second, "Operating_Mode", &data[0], &log))
                {
                  result = true;
                  normal_modes_[dxl.first]["Operating_Mode"] = data[0];
                }
                else if (dxl_wb_->readRegister((uint8_t)dxl.second, "CW_Angle_Limit", &data[0], &log) &&
                         dxl_wb_->readRegister((uint8_t)dxl.second, "CCW_Angle_Limit", &data[1], &log))
                {
                  // On some actuators, Operating Mode is represented by CW/CCW Angle Limit
                  result = true;
                  normal_modes_[dxl.first]["CW_Angle_Limit"] = data[0];
                  normal_modes_[dxl.first]["CCW_Angle_Limit"] = data[1];
                }
                if (result == false)
                {
                  ROS_ERROR("%s", log);
                }
                dxl_wb_->torqueOff((uint8_t)dxl.second);  // We cannot change Operating Mode while torque on
                result = dxl_wb_->setVelocityControlMode((uint8_t)dxl.second, &log);
                if (result == false)
                {
                  ROS_ERROR("%s", log);
                }
                dxl_wb_->torqueOn((uint8_t)dxl.second);
                last_write_tm_ = ros::Time::now();
              }
            }
          }
        }
        id_cnt++;
      }

      // Write effort command to dynamixel
      if (eff_id_vec.size() > 0)
      {
        bool result = false;
        const char* log = NULL;
        uint8_t eff_id[eff_id_vec.size()];
        int32_t dxl_eff[dxl_eff_vec.size()];
        std::copy(eff_id_vec.begin(), eff_id_vec.end(), eff_id);
        std::copy(dxl_eff_vec.begin(), dxl_eff_vec.end(), dxl_eff);
        result =
            dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, eff_id, eff_id_vec.size(), dxl_eff, 1, &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
        }
        last_write_tm_ = ros::Time::now();
      }
    }
  }
  else
  {
    // Servo off
    for (const std::pair<std::string, uint32_t>& dxl : dynamixel_)
    {
      dxl_wb_->torqueOff((uint8_t)dxl.second);
    }
    last_write_tm_ = ros::Time::now();

    // Servo off resets other special states
    is_hold_pos_raw_ = false;
    is_hold_pos_ = false;
    if (normal_modes_.size() > 0)
    {
      // Restore normal Operating Mode
      for (const std::pair<std::string, std::map<std::string, int32_t>>& mode : normal_modes_)
      {
        bool result = true;
        const char* log = NULL;
        for (const std::pair<std::string, int32_t>& addr_val : mode.second)
        {
          if (!dxl_wb_->writeRegister(dynamixel_[mode.first], addr_val.first.c_str(), addr_val.second, &log))
          {
            result = false;
            break;
          }
        }
        if (result == false)
        {
          ROS_ERROR("%s", log);
        }
        last_write_tm_ = ros::Time::now();
      }
      normal_modes_.clear();
    }
  }
  if (isJntCmdIgnored())
  {
    // Reset joint limit interfaces to prevent runaway when going back to normal state
    pos_jnt_sat_interface_.reset();
    pos_jnt_soft_interface_.reset();
  }
  prev_is_servo_ = is_servo_;
  prev_is_hold_pos_ = is_hold_pos_;
}

bool DynamixelGeneralHw::isJntCmdIgnored(void)
{
  return (!is_servo_ || is_hold_pos_);
}

void DynamixelGeneralHw::servoCallback(const std_msgs::BoolConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mtx_);

  is_servo_raw_ = msg->data;
}

void DynamixelGeneralHw::holdPosCallback(const std_msgs::BoolConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mtx_);

  is_hold_pos_raw_ = msg->data;
}

bool DynamixelGeneralHw::dynamixelCmdCallback(dynamixel_workbench_msgs::DynamixelCommand::Request& req,
                                              dynamixel_workbench_msgs::DynamixelCommand::Response& res)
{
  std::lock_guard<std::mutex> lock(mtx_);

  bool result = false;
  const char* log;

  uint8_t id = req.id;
  std::string item_name = req.addr_name;
  int32_t value = req.value;

  result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
  }

  res.comm_result = result;

  return true;
}

};  // end namespace dynamixel_general_hw
