#include "dynamixel_workbench_monitor/dynamixel_workbench_monitor_node.h"

using namespace dynamixel_workbench_monitor;

DynamixelWorkbenchMonitor::DynamixelWorkbenchMonitor()
    :nh_priv_("~"),
     is_debug_(false),
     device_name_(DEVICENAME),
     baud_rate_(BAUDRATE),
     protocol_version_(PROTOCOL_VERSION),
     control_mode_set_(CONTROL_MODE)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name_", device_name_);
  nh_priv_.getParam("baud_rate_", baud_rate_);
  nh_priv_.getParam("protocol_version_",protocol_version_);
  nh_priv_.getParam("control_mode_set_", control_mode_set_);

  // Init target name
  ROS_ASSERT(initDynamixelController());

  // Init ROS publish
  dxl_position_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelResponseList>("/dxl_motor_state",10);

  // Init ROS Service server
  dynamixel_workbench_monitor_server_ = nh_.advertiseService("dynamixel_workbench_monitor_remote", &DynamixelWorkbenchMonitor::dynamixelCommandServer, this);

  // Init dynamixel tool
  if (strcmp(control_mode_set_.c_str(), "position") == 0)
  {
    dxl_position_control_tool_ = new dynamixel_position_control::DynamixelPositionControl(device_name_, baud_rate_, protocol_version_);
  }
  else
  {
    ROS_ERROR("Invalidate mode");
    shutdownDynamixelWorkbenchMonitor();
  }

  // Scan Dynamixel
  dxl_position_control_tool_->scanDynamixelId(&dxl_);

  // Enable Dynamixel Torque
  dxl_position_control_tool_->setTorque(dxl_, true);

  ROS_INFO("Press ESC to exit dynamixel workbench monitor node");
}

DynamixelWorkbenchMonitor::~DynamixelWorkbenchMonitor()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchMonitor());
}

void DynamixelWorkbenchMonitor::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  dxl_position_control_tool_->setTorque(dxl_, false);

  dxl_position_control_tool_->portHandler_->closePort();

  ros::shutdown();
}

int DynamixelWorkbenchMonitor::getch()
{
  struct termios oldt, newt;
  int ch;

  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 1;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

  return ch;
}

int DynamixelWorkbenchMonitor::kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

bool DynamixelWorkbenchMonitor::initDynamixelController()
{
  ROS_INFO("dynamixel_workbench_monitor_node : Init OK!");
  return true;
}

bool DynamixelWorkbenchMonitor::shutdownDynamixelWorkbenchMonitor()
{
  dxl_position_control_tool_->portHandler_->closePort();
  return true;
}

//bool DynamixelWorkbenchMonitor::writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int32_t value)
//{
//  uint8_t dxl_error = 0;
//  int dxl_comm_result = COMM_TX_FAIL;

//  if (length == 1)
//  {
//    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (int8_t)value, &dxl_error);
//  }
//  else if (length == 2)
//  {
//    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, addr, (int16_t)value, &dxl_error);
//  }
//  else if (length == 4)
//  {
//    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, addr, (int32_t)value, &dxl_error);
//  }

//  if (dxl_comm_result == COMM_SUCCESS)
//  {
//    if (dxl_error != 0)
//    {
//      packetHandler_->printRxPacketError(dxl_error);
//    }
//    return true;
//  }
//  else
//  {
//    packetHandler_->printTxRxResult(dxl_comm_result);
//    ROS_ERROR("[ID] %u, Fail to write!", id);
//    return false;
//  }

//}

//bool DynamixelWorkbenchMonitor::readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, uint32_t *value)
//{
//  uint8_t dxl_error = 0;
//  int dxl_comm_result = COMM_RX_FAIL;

//  int8_t value_8_bit = 0;
//  int16_t value_16_bit = 0;
//  int32_t value_32_bit = 0;

//  if (length == 1)
//  {
//    dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value_8_bit, &dxl_error);
//  }
//  else if (length == 2)
//  {
//    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value_16_bit, &dxl_error);
//  }
//  else if (length == 4)
//  {
//    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value_32_bit, &dxl_error);
//  }

//  if (dxl_comm_result == COMM_SUCCESS)
//  {
//    if (dxl_error != 0)
//    {
//      packetHandler_->printRxPacketError(dxl_error);
//    }

//    if (length == 1)
//    {
//      *value = value_8_bit;
//      //ROS_INFO("[ID] %u, [Present Value] %d", id, value_8_bit);
//      return true;

//    }
//    else if (length == 2)
//    {
//      *value = value_16_bit;
//      //ROS_INFO("[ID] %u, [Present Value] %d", id, value_16_bit);
//      return true;
//    }
//    else if (length == 4)
//    {
//      *value = value_32_bit;
//      //ROS_INFO("[ID] %u, [Present Value] %d", id, value_32_bit);
//      return true;
//    }
//  }
//  else
//  {
//    packetHandler_->printTxRxResult(dxl_comm_result);
//    ROS_ERROR("[ID] %u, Fail to read!", id);
//    return false;
//  }
//}

bool DynamixelWorkbenchMonitor::dynamixelCommandServer(dynamixel_workbench_monitor::MonitorCommand::Request &req,
                                                       dynamixel_workbench_monitor::MonitorCommand::Response &res)
{
  if (strcmp(req.cmd.c_str(), "push") == 0)
  {
    ROS_INFO("Push DXL ID : %d", req.dxl_id);
    dxl_control_.push_back(new dxl_motor::DxlMotor(req.dxl_id));
  }
  else if (strcmp(req.cmd.c_str(), "pop") == 0)
  {
    ROS_INFO("Pop DXL ID : %d", req.dxl_id);
    dxl_control_.pop_back();
  }
  else if (strcmp(req.cmd.c_str(), "check") == 0)
  {
    for (int i = 0; i < dxl_control_.size(); i++)
    {
      ROS_INFO("Registered DXL : %d", dxl_control_[i]->id_);
    }
  }
  else if (strcmp(req.cmd.c_str(), "torque") == 0)
  {
    ROS_INFO("DXL torque statue : %d", req.onoff);
    for (int i = 0; i < dxl_.size(); i++)
    {
      ROS_INFO("request: x=%ld, y=%ld",(long int)dxl_[i]->id_, (long int)req.onoff);
    }
    dxl_position_control_tool_->setTorque(dxl_, req.onoff);
  }
  else if (strcmp(req.cmd.c_str(), "pos") == 0)
  {
    for (int i = 0; i < dxl_.size(); i++)
    {
      ROS_INFO("request: x=%ld, y=%ld",(long int)dxl_[i]->id_, (long int)req.pos_value);
    }
    dxl_position_control_tool_->writeGoalPosition(dxl_, req.pos_value);
  }
  else if (strcmp(req.cmd.c_str(), "vel") == 0)
  {
    for (int i = 0; i< dxl_.size(); i++)
    {
      ROS_INFO("request: x=%ld, y=%ld",(long int)dxl_[i]->id_, (long int)req.vel_value);
    }
    dxl_position_control_tool_->writeProfileVelocity(dxl_, req.vel_value);
  }
  return true;
}

void DynamixelWorkbenchMonitor::dynamixelMonitorLoop(void)
{
  if (kbhit())
  {
    char c = getch();
    if (c == ESC_ASCII_VALUE)
    {
      closeDynamixel();
    }
  }

  dynamixel_workbench_msgs::DynamixelResponse response[dxl_.size()];
  dynamixel_workbench_msgs::DynamixelResponseList response_list;

  dxl_position_control_tool_->readRealtimeTick(dxl_, &dxl_realtime_tick_read_data_);
//  dxl_position_control_tool_->readOperatingMode(dxl_id_vec_, &dxl_operating_mode_read_data_);
//  dxl_position_control_tool_->readTorque(dxl_id_vec_, &dxl_torque_read_data_);
//  dxl_position_control_tool_->readGoalPosition(dxl_id_vec_, &dxl_goal_position_read_data_);
//  dxl_position_control_tool_->readPresentPosition(dxl_id_vec_, &dxl_present_position_read_data_);
//  dxl_position_control_tool_->readProfileVelocity(dxl_id_vec_, &dxl_profile_velocity_read_data_);
//  dxl_position_control_tool_->readPresentVelocity(dxl_id_vec_, &dxl_present_velocity_read_data_);
//  dxl_position_control_tool_->readVoltage(dxl_id_vec_, &dxl_voltage_);
//  dxl_position_control_tool_->readTemperature(dxl_id_vec_, &dxl_temperature_);
//  dxl_position_control_tool_->readIsMoving(dxl_id_vec_, &dxl_is_moving_);

  for (int i = 0; i < dxl_.size(); i++)
  {
    response[i].timestamps = dxl_realtime_tick_read_data_.at(i);
    response[i].id = dxl_[i]->id_;
    response[i].model_name = dxl_[i]->model_name_;
//    response[i].operating_mode = dxl_operating_mode_read_data_.at(i);
//    response[i].torque = dxl_torque_read_data_.at(i);
//    response[i].goal_position = dxl_goal_position_read_data_.at(i);
//    response[i].present_position = dxl_present_position_read_data_.at(i);
//    response[i].profile_velocity = dxl_profile_velocity_read_data_.at(i);
//    response[i].present_velocity = dxl_present_velocity_read_data_.at(i);
//    response[i].voltage = dxl_voltage_.at(i);
//    response[i].temperature = dxl_temperature_.at(i);
//    response[i].is_moving = dxl_is_moving_.at(i);
  }

  for(int i = 0; i < dxl_.size(); i++)
  {
    response_list.dynamixel_responses.push_back(response[i]);
  }
  dxl_position_pub_.publish(response_list);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_monitor_node");
  DynamixelWorkbenchMonitor dwm;
  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    dwm.dynamixelMonitorLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  dwm.closeDynamixel();

  return 0;
}
