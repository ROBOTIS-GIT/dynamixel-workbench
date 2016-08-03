#include "dynamixel_workbench_tool/dynamixel_tool.h"

using namespace dynamixel_workbench_tool;

DynamixelWorkbenchTool::DynamixelWorkbenchTool(std::string device_name, float baud_rate, float protocol_version)
{
  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchTool());

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  ROS_INFO("device_name: %s", device_name.c_str());
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1.0 PacketHandler or Protocol 2.0 PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  // Open port
  if (portHandler_->openPort())
  {
    ROS_INFO("Succeeded to open the port(%s)!", device_name.c_str());
  }
  else
  {
    ROS_ERROR("Failed to open the port!");
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baud_rate))
  {
    ROS_INFO("Succeeded to change the baudrate(%d)!", portHandler_->getBaudRate());
  }
  else
  {
    ROS_ERROR("Failed to change the baudrate!");
  }

  dxl_addparam_result_ = false;
  dxl_comm_result_ = COMM_TX_FAIL;
}

DynamixelWorkbenchTool::~DynamixelWorkbenchTool()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchTool());
}

bool DynamixelWorkbenchTool::initDynamixelWorkbenchTool()
{
  ROS_INFO("dynamixel_workbench_tool : Init OK!");
  return true;
}

bool DynamixelWorkbenchTool::shutdownDynamixelWorkbenchTool()
{
  return true;
}


bool DynamixelWorkbenchTool::scanDynamixelId(std::vector<uint8_t> *dxl_id_vec, std::vector<uint16_t> *dxl_model_vec)
{
  uint8_t dxl_error = 0;
  uint8_t dxl_id = 1;
  uint16_t dxl_model = 0;

  ROS_INFO("Scan Dynamixel Using Protocol 2.0");

  for (dxl_id = 1; dxl_id < 253; dxl_id++)
  {
    if (packetHandler_->ping(portHandler_, dxl_id, &dxl_model, &dxl_error) == COMM_SUCCESS)
    {
      dxl_id_vec->push_back(dxl_id);
      dxl_model_vec->push_back(dxl_model);
    }
    else
    {
      fprintf(stderr,".");
    }
  }

  if (dxl_id_vec->empty() == true)
  {
    ROS_INFO("");
    ROS_ERROR("...Failed to find dynamixel!");
    return false;
  }
  else
  {
    ROS_INFO("");
    ROS_INFO("...Succeeded to find dynamixel");
    for (int i = 0; i < dxl_id_vec->size(); i++)
    {
      ROS_INFO("[ID] %u, [Model No.] %d", dxl_id_vec->at(i), dxl_model_vec->at(i));
    }
    return true;
  }
}

bool DynamixelWorkbenchTool::setTorque(std::vector<uint8_t> dxl_id_vec, bool onoff)
{
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_XM_TORQUE_ENABLE, 1);

  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_addparam_result_ = groupSyncWrite.addParam(dxl_id_vec.at(i), (uint8_t*)&onoff);
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dxl_id_vec.at(i));
      return false;
    }
  }

  dxl_comm_result_ = groupSyncWrite.txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWrite.clearParam();
  return true;
}

bool DynamixelWorkbenchTool::writeGoalPosition(std::vector<uint8_t> dxl_id_array, uint32_t pos_value)
{
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_XM_GOAL_POSITION, 4);

  for (int i = 0; i < dxl_id_array.size(); i++)
  {
    dxl_addparam_result_ = groupSyncWrite.addParam(dxl_id_array.at(i), (uint8_t*)&pos_value);
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dxl_id_array.at(i));
      return false;
    }
  }

  dxl_comm_result_ = groupSyncWrite.txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWrite.clearParam();
  return true;
}

bool DynamixelWorkbenchTool::writeProfileVelocity(std::vector<uint8_t> dxl_id_array, uint32_t vel_value)
{
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_XM_PROFILE_VELOCITY, 4);

  for (int i = 0; i < dxl_id_array.size(); i++)
  {
    dxl_addparam_result_ = groupSyncWrite.addParam(dxl_id_array[i], (uint8_t*)&vel_value);
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dxl_id_array[i]);
      return false;
    }
  }

  dxl_comm_result_ = groupSyncWrite.txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWrite.clearParam();
  return true;
}

bool DynamixelWorkbenchTool::readRealtimeTick(std::vector<uint8_t> dxl_id_vec, std::vector<uint16_t> *dxl_read_data)
{
 dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_XM_REALTIME_TICK, 2);

 for (int i = 0; i < dxl_id_vec.size(); i++)
 {
   dxl_addparam_result_ = groupSyncRead.addParam(dxl_id_vec.at(i));
   if (dxl_addparam_result_ != true)
   {
     ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dxl_id_vec.at(i));
     return false;
   }
 }

 // Syncread present position
 dxl_comm_result_ = groupSyncRead.txRxPacket();
 if (dxl_comm_result_ != COMM_SUCCESS)
 {
   packetHandler_->printTxRxResult(dxl_comm_result_);
 }

 // Check if groupsyncread data is available
 for (int i = 0; i < dxl_id_vec.size(); i++)
 {
   dxl_getdata_result_ = groupSyncRead.isAvailable(dxl_id_vec.at(i), ADDR_XM_REALTIME_TICK, 2);
   if (dxl_getdata_result_ != true)
   {
     fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id_vec.at(i));
     return false;
   }
 }

 dxl_read_data->clear();

 // Get present position value
 for (int i = 0; i < dxl_id_vec.size(); i++)
 {
   dxl_read_data->push_back(groupSyncRead.getData(dxl_id_vec.at(i), ADDR_XM_REALTIME_TICK, 2));
 }

 groupSyncRead.clearParam();
 return true;
}

bool DynamixelWorkbenchTool::readOperatingMode(std::vector<uint8_t> dxl_id_vec, std::vector<uint16_t> *dxl_read_data)
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_XM_OPERATIING_MODE, 1);

  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dxl_id_vec.at(i));
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dxl_id_vec.at(i));
      return false;
    }
  }

  // Syncread present position
  dxl_comm_result_ = groupSyncRead.txRxPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
  }

  // Check if groupsyncread data is available
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dxl_id_vec.at(i), ADDR_XM_OPERATIING_MODE, 1);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id_vec.at(i));
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dxl_id_vec.at(i), ADDR_XM_OPERATIING_MODE, 1));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchTool::readTorque(std::vector<uint8_t> dxl_id_vec, std::vector<bool> *dxl_read_data)
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_XM_TORQUE_ENABLE, 1);

  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dxl_id_vec.at(i));
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dxl_id_vec.at(i));
      return false;
    }
  }

  // Syncread present position
  dxl_comm_result_ = groupSyncRead.txRxPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
  }

  // Check if groupsyncread data is available
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dxl_id_vec.at(i), ADDR_XM_TORQUE_ENABLE, 1);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id_vec.at(i));
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dxl_id_vec.at(i), ADDR_XM_TORQUE_ENABLE, 1));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchTool::readGoalPosition(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data)
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_XM_GOAL_POSITION, 4);

  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dxl_id_vec.at(i));
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dxl_id_vec.at(i));
      return false;
    }
  }

  // Syncread present position
  dxl_comm_result_ = groupSyncRead.txRxPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
  }

  // Check if groupsyncread data is available
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dxl_id_vec.at(i), ADDR_XM_GOAL_POSITION, 4);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id_vec.at(i));
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dxl_id_vec.at(i), ADDR_XM_GOAL_POSITION, 4));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchTool::readPresentPosition(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data)
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_XM_PRESENT_POSITION, 4);

  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dxl_id_vec.at(i));
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dxl_id_vec.at(i));
      return false;
    }
  }

  // Syncread present position
  dxl_comm_result_ = groupSyncRead.txRxPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
  }

  // Check if groupsyncread data is available
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dxl_id_vec.at(i), ADDR_XM_PRESENT_POSITION, 4);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id_vec.at(i));
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dxl_id_vec.at(i), ADDR_XM_PRESENT_POSITION, 4));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchTool::readProfileVelocity(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data)
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_XM_PROFILE_VELOCITY, 4);

  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dxl_id_vec.at(i));
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dxl_id_vec.at(i));
      return false;
    }
  }

  // Syncread present position
  dxl_comm_result_ = groupSyncRead.txRxPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
  }

  // Check if groupsyncread data is available
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dxl_id_vec.at(i), ADDR_XM_PROFILE_VELOCITY, 4);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id_vec.at(i));
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dxl_id_vec.at(i), ADDR_XM_PROFILE_VELOCITY, 4));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchTool::readPresentVelocity(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data)
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_XM_PRESENT_VELOCITY, 4);

  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dxl_id_vec.at(i));
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dxl_id_vec.at(i));
      return false;
    }
  }

  // Syncread present position
  dxl_comm_result_ = groupSyncRead.txRxPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
  }

  // Check if groupsyncread data is available
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dxl_id_vec.at(i), ADDR_XM_PRESENT_VELOCITY, 4);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id_vec.at(i));
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dxl_id_vec.at(i), ADDR_XM_PRESENT_VELOCITY, 4));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchTool::readVoltage(std::vector<uint8_t> dxl_id_vec, std::vector<uint16_t> *dxl_read_data)
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_XM_VOLTAGE, 2);

  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dxl_id_vec.at(i));
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dxl_id_vec.at(i));
      return false;
    }
  }

  // Syncread present position
  dxl_comm_result_ = groupSyncRead.txRxPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
  }

  // Check if groupsyncread data is available
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dxl_id_vec.at(i), ADDR_XM_VOLTAGE, 2);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id_vec.at(i));
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dxl_id_vec.at(i), ADDR_XM_VOLTAGE, 2));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchTool::readTemperature(std::vector<uint8_t> dxl_id_vec, std::vector<uint8_t> *dxl_read_data)
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_XM_TEMPERATURE, 1);

  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dxl_id_vec.at(i));
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dxl_id_vec.at(i));
      return false;
    }
  }

  // Syncread present position
  dxl_comm_result_ = groupSyncRead.txRxPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
  }

  // Check if groupsyncread data is available
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dxl_id_vec.at(i), ADDR_XM_TEMPERATURE, 1);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id_vec.at(i));
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dxl_id_vec.at(i), ADDR_XM_TEMPERATURE, 1));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchTool::readIsMoving(std::vector<uint8_t> dxl_id_vec, std::vector<bool> *dxl_read_data)
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_XM_IS_MOVING, 1);

  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dxl_id_vec.at(i));
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dxl_id_vec.at(i));
      return false;
    }
  }

  // Syncread present position
  dxl_comm_result_ = groupSyncRead.txRxPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
  }

  // Check if groupsyncread data is available
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dxl_id_vec.at(i), ADDR_XM_IS_MOVING, 1);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id_vec.at(i));
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dxl_id_vec.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dxl_id_vec.at(i), ADDR_XM_IS_MOVING, 1));
  }

  groupSyncRead.clearParam();
  return true;
}
