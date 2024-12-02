#include "dynamixel_general_hw/dynamixel_general_hw.h"

namespace dynamixel_general_hw
{

DynamixelGeneralHw::DynamixelGeneralHw()
  : pnh_("~")
  , is_servo_raw_(true)
  , is_servo_(is_servo_raw_)
  , prev_is_servo_(is_servo_)
  , is_hold_pos_raw_(false)
  , is_hold_pos_(is_hold_pos_raw_)
  , is_current_eq_load_(false)
{
  is_calc_effort_ = pnh_.param<bool>("calculate_effort", true);
  is_pub_temp_ = pnh_.param<bool>("publish_temperature", true);
  is_pub_volt_ = pnh_.param<bool>("publish_input_voltage", true);

  dxl_wb_.reset(new DynamixelWorkbench);
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

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
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
    uint16_t start_address = *min_addr;
    uint16_t read_length = (*max_addr - *min_addr) + control_items_[target_items[max_addr_idx]]->data_length;

    result = dxl_wb_->addSyncReadHandler(start_address, read_length, &log);
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
  for (int i = 0; i < dynamixel_.size(); i++)
  {
    hardware_interface::ActuatorStateHandle state_handle(actr_names_[i], &actr_curr_pos_[i], &actr_curr_vel_[i], &actr_curr_eff_[i]);
    actr_state_interface_.registerHandle(state_handle);

    hardware_interface::ActuatorHandle position_handle(state_handle, &actr_cmd_pos_[i]);
    pos_actr_interface_.registerHandle(position_handle);
  }
  registerInterface(&actr_state_interface_);
  registerInterface(&pos_actr_interface_);

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
    ROS_INFO_STREAM_ONCE("Waiting for robot_description");
    nh_.getParam("robot_description", urdf_string);
    ros::Duration(0.1).sleep();
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
      else
      {
        ROS_INFO_STREAM("Loaded transmission: " << info.name_);
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
    uint16_t length_of_data = control_items_["Present_Position"]->data_length +
                              control_items_["Present_Velocity"]->data_length +
                              control_items_["Present_Current"]->data_length;
    uint32_t get_all_data[length_of_data];
    uint32_t get_optional_data[2];
    uint8_t dxl_cnt = 0;
    for (auto const& dxl:dynamixel_)
    {
      result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                     control_items_["Present_Position"]->address,
                                     length_of_data,
                                     get_all_data,
                                     &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      dynamixel_state[dxl_cnt].present_current = DXL_MAKEWORD(get_all_data[4], get_all_data[5]);
      dynamixel_state[dxl_cnt].present_velocity = DXL_MAKEWORD(get_all_data[2], get_all_data[3]);
      dynamixel_state[dxl_cnt].present_position = DXL_MAKEWORD(get_all_data[0], get_all_data[1]);

      if (is_pub_temp_)
      {
        result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                       control_items_["Present_Temperature"]->address,
                                       control_items_["Present_Temperature"]->data_length,
                                       get_optional_data,
                                       &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
        }

        if (control_items_["Present_Temperature"]->data_length == 1)
        {
          dynamixel_state[dxl_cnt].present_temperature = get_optional_data[0];
        }
        else if (control_items_["Present_Temperature"]->data_length == 2)
        {
          dynamixel_state[dxl_cnt].present_temperature = DXL_MAKEWORD(get_optional_data[0], get_optional_data[1]);
        }
      }

      if (is_pub_volt_)
      {
        result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                       control_items_["Present_Input_Voltage"]->address,
                                       control_items_["Present_Input_Voltage"]->data_length,
                                       get_optional_data,
                                       &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
        }

        if (control_items_["Present_Input_Voltage"]->data_length == 1)
        {
          dynamixel_state[dxl_cnt].present_input_voltage = get_optional_data[0];
        }
        else if (control_items_["Present_Input_Voltage"]->data_length == 2)
        {
          dynamixel_state[dxl_cnt].present_input_voltage = DXL_MAKEWORD(get_optional_data[0], get_optional_data[1]);
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

void DynamixelGeneralHw::write(void)
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

    if (!is_hold_pos_)
    {
      // Propagate joint commands to actuators
      if (robot_transmissions_.get<transmission_interface::JointToActuatorPositionInterface>())
      {
        robot_transmissions_.get<transmission_interface::JointToActuatorPositionInterface>()->propagate();
      }

      // Convert ros_control actuator command to dynamixel command
      std::vector<uint8_t> pos_id_vec;
      uint8_t id_cnt = 0;
      std::vector<int32_t> dxl_pos_vec;
      for (const std::pair<std::string, uint32_t>& dxl : dynamixel_)
      {
        if (std::isnan(actr_cmd_pos_[id_cnt]))
        {
          ROS_DEBUG_STREAM_DELAYED_THROTTLE(10, "Skipping position command to "
                                                    << dxl.first << " because it is NaN. Its controller may not work");
        }
        else
        {
          pos_id_vec.push_back((uint8_t)dxl.second);
          dxl_pos_vec.push_back(dxl_wb_->convertRadian2Value((uint8_t)dxl.second, actr_cmd_pos_[id_cnt]));
        }
        id_cnt++;
      }

      // Write command to dynamixel
      if (pos_id_vec.size() > 0)
      {
        bool result = false;
        const char* log = NULL;
        uint8_t pos_id[pos_id_vec.size()];
        int32_t dxl_pos[dxl_pos_vec.size()];
        std::copy(pos_id_vec.begin(), pos_id_vec.end(), pos_id);
        std::copy(dxl_pos_vec.begin(), dxl_pos_vec.end(), dxl_pos);
        result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, pos_id, pos_id_vec.size(), dxl_pos, 1, &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
        }
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
  }
  prev_is_servo_ = is_servo_;
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
