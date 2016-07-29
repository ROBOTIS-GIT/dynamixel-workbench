#include "dynamixel_workbench_monitor/dynamixel_workbench_monitor_node.h"

using namespace dynamixel_workbench_monitor;

DynamixelWorkbenchMonitor::DynamixelWorkbenchMonitor()
    :nh_priv_("~"),
     is_debug_(false),
     device_name_(DEVICENAME),
     dxl_id_(1),
     baud_rate_(BAUDRATE),
     protocol_version_(PROTOCOL_VERSION)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name_", device_name_);
  nh_priv_.getParam("baud_rate_", baud_rate_);
  nh_priv_.getParam("protocol_version_",protocol_version_);

  // Init target name
  ROS_ASSERT(initDynamixelController());

  // Init ROS publish
  dxl_position_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelResponse>("/dxl_motor_state",10);

  // Init ROS Service server
  dynamixel_workbench_monitor_server_ = nh_.advertiseService("dynamixel_workbench_monitor_remote", &DynamixelWorkbenchMonitor::dynamixelCommandServer, this);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1.0 PacketHandler or Protocol 2.0 PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // Open port
  if (portHandler_->openPort())
  {
    ROS_INFO("Succeeded to open the port(%s)!", device_name_.c_str());
  }
  else
  {
    ROS_ERROR("Failed to open the port!");
    shutdownDynamixelWorkbenchMonitor();
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baud_rate_))
  {
    ROS_INFO("Succeeded to change the baudrate(%d)!", portHandler_->getBaudRate());
  }
  else
  {
    ROS_ERROR("Failed to change the baudrate!");
    shutdownDynamixelWorkbenchMonitor();
  }

  // Scan Dynamixel
  dxl_id_ = scanDynamixelId(portHandler_, packetHandler_);

  // Enable Dynamixel Torque
  setTorque(dxl_id_, true);

  ROS_INFO("Press ESC to exit dynamixel workbench monitor node");
}

DynamixelWorkbenchMonitor::~DynamixelWorkbenchMonitor()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchMonitor());
}

void DynamixelWorkbenchMonitor::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  setTorque(dxl_id_, false);

  // Close port
  portHandler_->closePort();

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
  portHandler_->closePort();
  return true;
}

uint8_t DynamixelWorkbenchMonitor::scanDynamixelId(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
  uint8_t dxl_error = 0;
  uint16_t dxl_model_num = 0;
  uint8_t dxl_numbers = 0;
  int id = 1;

  ROS_INFO("Scan Dynamixel Using Protocol 2.0");

  for (id = 1; id < 253; id++)
  {
    if (packetHandler->ping(portHandler, id, &dxl_model_num, &dxl_error) == COMM_SUCCESS)
    {
      dxl_numbers++;
      break;
    }
    else
    {
      ROS_INFO(".");
    }
  }

  if (dxl_numbers != 0)
  {
    ROS_INFO("");
    ROS_INFO("...Succeeded to find dynamixel");
    ROS_INFO("[ID] %u, [Model No.] %d", id, dxl_model_num);
    return id;
  }
  else
  {
    ROS_INFO("");
    ROS_ERROR("...Failed to find dynamixel!");
    closeDynamixel();
  }
}

bool DynamixelWorkbenchMonitor::writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (int8_t)value, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, addr, (int16_t)value, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, addr, (int32_t)value, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0)
    {
      packetHandler_->printRxPacketError(dxl_error);
    }
    return true;
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to write!", id);
    return false;
  }

}

bool DynamixelWorkbenchMonitor::readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, uint32_t *value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_RX_FAIL;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  if (length == 1)
  {
    dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value_8_bit, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value_16_bit, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value_32_bit, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0)
    {
      packetHandler_->printRxPacketError(dxl_error);
    }

    if (length == 1)
    {
      *value = value_8_bit;
      //ROS_INFO("[ID] %u, [Present Value] %d", id, value_8_bit);
      return true;

    }
    else if (length == 2)
    {
      *value = value_16_bit;
      //ROS_INFO("[ID] %u, [Present Value] %d", id, value_16_bit);
      return true;
    }
    else if (length == 4)
    {
      *value = value_32_bit;
      //ROS_INFO("[ID] %u, [Present Value] %d", id, value_32_bit);
      return true;
    }
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to read!", id);
    return false;
  }
}

bool DynamixelWorkbenchMonitor::setTorque(uint8_t id, bool onoff)
{
  bool set_torque_error = false;

  set_torque_error = writeDynamixelRegister(id, ADDR_XM_TORQUE_ENABLE, 1, onoff);

  if (set_torque_error == false)
  {
    ROS_ERROR("[ID] %u, Fail to set dxl torque!", id);
    return false;
  }
  else
  {
    return true;
  }
}

bool DynamixelWorkbenchMonitor::readOperatingMode(uint8_t id, int8_t *operating_mode)
{
  bool read_operating_mode_error = false;
  int8_t dxl_operating_mode = 0;

  read_operating_mode_error = readDynamixelRegister(id, ADDR_XM_OPERATIING_MODE, 1, (uint32_t*)&dxl_operating_mode);

  if (read_operating_mode_error == false)
  {
    ROS_ERROR("[ID] %u, Fail to read dxl operating mode!", id);
    return false;
  }
  else
  {
    *operating_mode = dxl_operating_mode;
    return true;
  }
}

bool DynamixelWorkbenchMonitor::readTorque(uint8_t id, int8_t *torque)
{
  bool read_torque_error = false;
  int8_t dxl_torque = false;

  read_torque_error = readDynamixelRegister(id, ADDR_XM_TORQUE_ENABLE, 1, (uint32_t*)&dxl_torque);

  if (read_torque_error == false)
  {
    ROS_ERROR("[ID] %u, Fail to read dxl torque!", id);
    return false;
  }
  else
  {
    *torque = dxl_torque;
    return true;
  }
}

bool DynamixelWorkbenchMonitor::readPresentPosition(uint8_t id, int32_t *position)
{
  bool read_position_error = false;
  int32_t dxl_present_position = 0;

  read_position_error = readDynamixelRegister(id, ADDR_XM_PRESENT_POSITION, 4, (uint32_t*)&dxl_present_position);

  if (read_position_error == false)
  {
    ROS_ERROR("[ID] %u, Fail to read dxl position!", id);
    return false;
  }
  else
  {
    *position = dxl_present_position;
    return true;
  }
}

bool DynamixelWorkbenchMonitor::readGoalPosition(uint8_t id, int32_t *goal_position)
{
  bool read_goal_position_error = false;
  int32_t dxl_goal_position = 0;

  read_goal_position_error = readDynamixelRegister(id, ADDR_XM_GOAL_POSITION, 4, (uint32_t*)&dxl_goal_position);

  if (read_goal_position_error == false)
  {
    ROS_ERROR("[ID] %u, Fail to read dxl goal position!", id);
    return false;
  }
  else
  {
    *goal_position = dxl_goal_position;
    return true;
  }
}

bool DynamixelWorkbenchMonitor::readRealtimeTick(uint8_t id, int16_t *realtime_tick)
{
  bool read_realtime_tick_error = false;
  int16_t dxl_realtime_tick = 0;

  read_realtime_tick_error = readDynamixelRegister(id, ADDR_XM_REALTIME_TICK, 2, (uint32_t*)&dxl_realtime_tick);

  if (read_realtime_tick_error == false)
  {
    ROS_ERROR("[ID] %u, Fail to read dxl realtime tick!", id);
    return false;
  }
  else
  {
    *realtime_tick = dxl_realtime_tick;
    return true;
  }
}

bool DynamixelWorkbenchMonitor::readPresentVelocity(uint8_t id, int32_t *velocity)
{
  bool read_present_velocity_error = false;
  int32_t dxl_present_velocity = 0;

  read_present_velocity_error = readDynamixelRegister(id, ADDR_XM_PRESENT_VELOCITY, 4, (uint32_t*)&dxl_present_velocity);

  if (read_present_velocity_error == false)
  {
    ROS_ERROR("[ID] %u, Fail to read present velocity!", id);
    return false;
  }
  else
  {
    *velocity = dxl_present_velocity;
    return true;
  }
}

bool DynamixelWorkbenchMonitor::readGoalVelocity(uint8_t id, int32_t *velocity)
{
  bool read_goal_velocity_error = false;
  int32_t dxl_goal_velocity = 0;

  read_goal_velocity_error = readDynamixelRegister(id, ADDR_XM_GOAL_VELOCITY, 4, (uint32_t*)&dxl_goal_velocity);

  if (read_goal_velocity_error == false)
  {
    ROS_ERROR("[ID] %u, Fail to read goal velocity!", id);
    return false;
  }
  else
  {
    *velocity = dxl_goal_velocity;
    return true;
  }
}

bool DynamixelWorkbenchMonitor::readPresentVoltage(uint8_t id, int16_t *voltage)
{
  bool read_present_voltage_error = false;
  int16_t dxl_present_voltage = 0.0;

  read_present_voltage_error = readDynamixelRegister(id, ADDR_XM_PRESENT_VOLTAGE, 2, (uint32_t*)&dxl_present_voltage);

  if (read_present_voltage_error == false)
  {
    ROS_ERROR("[ID] %u, Fail to read present voltage!", id);
    return false;
  }
  else
  {
    *voltage = dxl_present_voltage;
    return true;
  }
}

bool DynamixelWorkbenchMonitor::readPresentTemperature(uint8_t id, int8_t *temperature)
{
  bool read_present_temperature = false;
  int8_t dxl_present_temperature = 0;

  read_present_temperature = readDynamixelRegister(id, ADDR_XM_PRESENT_TEMPERATURE, 1, (uint32_t*)&dxl_present_temperature);

  if (read_present_temperature == false)
  {
    ROS_ERROR("[ID] %u, Fail to read present temperature!", id);
    return false;
  }
  else
  {
    *temperature = dxl_present_temperature;
    return true;
  }
}

bool DynamixelWorkbenchMonitor::readIsMoving(uint8_t id, int8_t *is_moving)
{
  bool read_moving = false;
  int8_t dxl_moving = 0;

  read_moving = readDynamixelRegister(id, ADDR_XM_MOVING, 1, (uint32_t*)&dxl_moving);

  if (read_moving == false)
  {
    ROS_ERROR("[ID] %u, Fail to read moving!", id);
    return false;
  }
  else
  {
    *is_moving = dxl_moving;
    return true;
  }
}

bool DynamixelWorkbenchMonitor::dynamixelCommandServer(dynamixel_workbench_monitor::MonitorCommand::Request &req,
                                                       dynamixel_workbench_monitor::MonitorCommand::Response &res)
{
  if (strcmp(req.cmd.c_str(), "pos") == 0)
  {
    ROS_INFO("request: x=%ld, y=%ld",(long int)req.dxl_id, (long int)req.pos_value);
    writeDynamixelRegister(req.dxl_id, ADDR_XM_GOAL_POSITION, 4, req.pos_value);
  }
  else if (strcmp(req.cmd.c_str(), "vel") == 0)
  {
    ROS_INFO("request: x=%ld, y=%ld",(long int)req.dxl_id, (long int)req.vel_value);
    writeDynamixelRegister(req.dxl_id, ADDR_XM_PROFILE_VELOCITY, 4, req.vel_value);
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

  dynamixel_workbench_msgs::DynamixelResponse response;

  readRealtimeTick(dxl_id_, &response.timestamps);
  response.id = dxl_id_;
  readOperatingMode(dxl_id_, &response.operating_mode);
  readTorque(dxl_id_, &response.torque);
  readGoalPosition(dxl_id_, &response.goal_position);
  readPresentPosition(dxl_id_, &response.present_position);
  readGoalVelocity(dxl_id_, &response.goal_velocity);
  readPresentVelocity(dxl_id_, &response.present_velocity);
  readPresentVoltage(dxl_id_, &response.voltage);
  readPresentTemperature(dxl_id_, &response.temperature);
  readIsMoving(dxl_id_, &response.is_moving);

  dxl_position_pub_.publish(response);
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
