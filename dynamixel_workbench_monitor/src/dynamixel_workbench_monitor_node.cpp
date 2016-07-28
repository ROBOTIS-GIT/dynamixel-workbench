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
  ROS_INFO("Got device_name_ baud_rate_ protocol_version_ : %s, %d, %.1f", device_name_.c_str(), baud_rate_, protocol_version_);


  // Init target name
  ROS_ASSERT(initDynamixelController());
  dxl_position_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelResponse>("/dxl_motor_position",10);

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
    ROS_INFO("Succeeded to open the port!");
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

  // Enable Dynamixel Torque
  setTorque(dxl_id_, true);
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
}

bool DynamixelWorkbenchMonitor::initDynamixelController()
{
  ROS_INFO("dynamixel_controller_node : Init OK!");
  return true;
}

bool DynamixelWorkbenchMonitor::shutdownDynamixelWorkbenchMonitor()
{
  return true;
}

void DynamixelWorkbenchMonitor::writeDynamixelRegister(uint8_t id, uint16_t addr, uint16_t length, int32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (int8_t)value, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (int16_t)value, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (int32_t)value, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0)
    {
      packetHandler_->printRxPacketError(dxl_error);
    }
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to write!", id);
  }

}

void DynamixelWorkbenchMonitor::readDynamixelRegister(uint8_t id, uint16_t addr, uint16_t length)
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
      ROS_INFO("[ID] %u, [Present Value] %d", id, value_8_bit);
    }
    else if (length == 2)
    {
      ROS_INFO("[ID] %u, [Present Value] %d", id, value_16_bit);
    }
    else if (length == 4)
    {
      ROS_INFO("[ID] %u, [Present Value] %d", id, value_32_bit);
    }
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to read!", id);
  }
}

bool DynamixelWorkbenchMonitor::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_XM_TORQUE_ENABLE, onoff, &dxl_error);
  if(dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
  }
  else if(dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
  }
}

bool DynamixelWorkbenchMonitor::readPosition(uint8_t id, int32_t &position, int32_t &realtime_tick)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_RX_FAIL;
  int32_t dxl_present_position = 0;
  int16_t dxl_realtime_tick = 0;

  dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, ADDR_XM_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    return false;
  }
  else if (dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
    return false;
  }

  dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, ADDR_XM_REALTIRM_TICK, (uint16_t*)&dxl_realtime_tick, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    return false;
  }
  else if (dxl_error != 0)
  {
    packetHandler_->printRxPacketError(dxl_error);
    return false;
  }

  position = dxl_present_position;
  realtime_tick = dxl_realtime_tick;

  return true;
}

void DynamixelWorkbenchMonitor::dynamixelMonitorLoop(void)
{
  dynamixel_workbench_msgs::DynamixelResponse response;

  //readDynamixelRegister(dxl_id_, ADDR_XM_PRESENT_VELOCITY, 4);

  readPosition(dxl_id_, response.position, response.realtime_tick);
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
