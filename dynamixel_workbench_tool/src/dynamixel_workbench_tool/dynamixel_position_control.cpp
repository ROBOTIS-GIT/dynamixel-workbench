#include "dynamixel_workbench_tool/dynamixel_position_control.h"

using namespace dynamixel_position_control;

DynamixelPositionControl::DynamixelPositionControl(std::string device_name, float baud_rate, float protocol_version)
{
  // Init target name
  ROS_ASSERT(initDynamixelPositionControl());

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
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

DynamixelPositionControl::~DynamixelPositionControl()
{
  ROS_ASSERT(shutdownDynamixelPositionControl());
}

bool DynamixelPositionControl::initDynamixelPositionControl()
{
  ROS_INFO("dynamixel_workbench_tool : Init OK!");
  return true;
}

bool DynamixelPositionControl::shutdownDynamixelPositionControl()
{
  return true;
}


bool DynamixelPositionControl::scanDynamixelId(std::vector<dxl_motor::DxlMotor *> *dynamixel)
{
  uint8_t dxl_error = 0;
  uint8_t dxl_id = 0;
  uint16_t dxl_model = 0;

  if (packetHandler_->getProtocolVersion() == 1.0)
  {
    ROS_INFO("Scan Dynamixel Using Protocol 1.0");
    for (dxl_id = 1; dxl_id < 253; dxl_id++)
    {
      if (packetHandler_->ping(portHandler_, dxl_id, &dxl_model, &dxl_error) == COMM_SUCCESS)
      {
        dynamixel->push_back(new dxl_motor::DxlMotor(dxl_id, dxl_model, 1.0));
      }
      else
      {
        fprintf(stderr,".");
      }
    }

    if (dynamixel->size() == 0)
    {
      ROS_INFO("");
      ROS_ERROR("...Failed to find dynamixel!");
      return false;
    }
    else
    {
      ROS_INFO("");
      ROS_INFO("...Succeeded to find dynamixel");
      for (int i = 0; i < dynamixel->size(); i++)
      {
        ROS_INFO("[ID] %u, [Model No.] %s", dynamixel->at(i)->id_, dynamixel->at(i)->model_name_.c_str());
      }
      return true;
    }
  }
  else
  {
    ROS_INFO("Scan Dynamixel Using Protocol 2.0");
    for (dxl_id = 1; dxl_id < 253; dxl_id++)
    {
      if (packetHandler_->ping(portHandler_, dxl_id, &dxl_model, &dxl_error) == COMM_SUCCESS)
      {        
        dynamixel->push_back(new dxl_motor::DxlMotor(dxl_id, dxl_model, 2.0));
      }
      else
      {
        fprintf(stderr,".");
      }
    }

    if (dynamixel->size() == 0)
    {
      ROS_INFO("");
      ROS_ERROR("...Failed to find dynamixel!");
      return false;
    }
    else
    {
      ROS_INFO("");
      ROS_INFO("...Succeeded to find dynamixel");

      for (int i = 0; i < dynamixel->size(); i++)
      {
        ROS_INFO("[ID] %u, [Model No.] %s", dynamixel->at(i)->id_,dynamixel->at(i)->model_name_.c_str());
      }
      return true;
    }
  }
}

bool DynamixelPositionControl::setTorque(std::vector<dxl_motor::DxlMotor *> dynamixel, bool onoff)
{
  std::vector<dxl_motor::DxlMotor *> dynamixel_protocol1;
  std::vector<dxl_motor::DxlMotor *> dynamixel_protocol2;

  for (int i = 0; i < dynamixel.size(); i++)
  {
    if (dynamixel[i]->protocol_version_ == 1.0)
    {
      dynamixel_protocol1.push_back(dynamixel[i]);
    }
    else
    {
      dynamixel_protocol2.push_back(dynamixel[i]);
    }
  }

  if (packetHandler_->getProtocolVersion() == 1.0)
  {
//    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_XM_TORQUE_ENABLE, 1);

//    for (int i = 0; i < dxl_id_vec.size(); i++)
//    {
//      dxl_addparam_result_ = groupSyncWrite.addParam(dxl_id_vec.at(i), (uint8_t*)&onoff);
//      if (dxl_addparam_result_ != true)
//      {
//        ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dxl_id_vec.at(i));
//        return false;
//      }
//    }

//    dxl_comm_result_ = groupSyncWrite.txPacket();
//    if (dxl_comm_result_ != COMM_SUCCESS)
//    {
//      packetHandler_->printTxRxResult(dxl_comm_result_);
//      return false;
//    }

//    groupSyncWrite.clearParam();
//    return true;
  }
  else
  {
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_,
                                             packetHandler_,
                                             dynamixel_protocol2[0]->torque_enable_item->address,
                                             dynamixel_protocol2[0]->torque_enable_item->data_length);

    for (int i = 0; i < dynamixel_protocol2.size(); i++)
    {
      dxl_addparam_result_ = groupSyncWrite.addParam(dynamixel_protocol2[i]->id_, (uint8_t*)&onoff);
      if (dxl_addparam_result_ != true)
      {
        ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_protocol2[i]->id_);
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
}

bool DynamixelPositionControl::writeGoalPosition(std::vector<dxl_motor::DxlMotor *> dynamixel, uint32_t pos_value)
{
  std::vector<dxl_motor::DxlMotor *> dynamixel_protocol1;
  std::vector<dxl_motor::DxlMotor *> dynamixel_protocol2;

  for (int i = 0; i < dynamixel.size(); i++)
  {
    if (dynamixel[i]->protocol_version_ == 1.0)
    {
      dynamixel_protocol1.push_back(dynamixel[i]);
    }
    else
    {
      dynamixel_protocol2.push_back(dynamixel[i]);
    }
  }

  if (packetHandler_->getProtocolVersion() == 1.0)
  {
//    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_XM_GOAL_POSITION, 4);

//    for (int i = 0; i < dxl_id_array.size(); i++)
//    {
//      dxl_addparam_result_ = groupSyncWrite.addParam(dxl_id_array.at(i), (uint8_t*)&pos_value);
//      if (dxl_addparam_result_ != true)
//      {
//        ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dxl_id_array.at(i));
//        return false;
//      }
//    }

//    dxl_comm_result_ = groupSyncWrite.txPacket();
//    if (dxl_comm_result_ != COMM_SUCCESS)
//    {
//      packetHandler_->printTxRxResult(dxl_comm_result_);
//      return false;
//    }

//    groupSyncWrite.clearParam();
//    return true;
  }
  else
  {
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_,
                                             packetHandler_,
                                             dynamixel_protocol2[0]->goal_position_item->address,
                                             dynamixel_protocol2[0]->goal_position_item->data_length);

    for (int i = 0; i < dynamixel_protocol2.size(); i++)
    {
      dxl_addparam_result_ = groupSyncWrite.addParam(dynamixel_protocol2[i]->id_, (uint8_t*)&pos_value);
      if (dxl_addparam_result_ != true)
      {
        ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_protocol2[i]->id_);
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
}

bool DynamixelPositionControl::writeProfileVelocity(std::vector<dxl_motor::DxlMotor *> dynamixel, uint32_t vel_value)
{
  std::vector<dxl_motor::DxlMotor *> dynamixel_protocol1;
  std::vector<dxl_motor::DxlMotor *> dynamixel_protocol2;

  for (int i = 0; i < dynamixel.size(); i++)
  {
    if (dynamixel[i]->protocol_version_ == 1.0)
    {
      dynamixel_protocol1.push_back(dynamixel[i]);
    }
    else
    {
      dynamixel_protocol2.push_back(dynamixel[i]);
    }
  }

  if (packetHandler_->getProtocolVersion() == 1.0)
  {
//    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_XM_GOAL_POSITION, 4);

//    for (int i = 0; i < dxl_id_array.size(); i++)
//    {
//      dxl_addparam_result_ = groupSyncWrite.addParam(dxl_id_array.at(i), (uint8_t*)&pos_value);
//      if (dxl_addparam_result_ != true)
//      {
//        ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dxl_id_array.at(i));
//        return false;
//      }
//    }

//    dxl_comm_result_ = groupSyncWrite.txPacket();
//    if (dxl_comm_result_ != COMM_SUCCESS)
//    {
//      packetHandler_->printTxRxResult(dxl_comm_result_);
//      return false;
//    }

//    groupSyncWrite.clearParam();
//    return true;
  }
  else
  {
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_,
                                             packetHandler_,
                                             dynamixel_protocol2[0]->profile_velocity_item->address,
                                             dynamixel_protocol2[0]->profile_velocity_item->data_length);

    for (int i = 0; i < dynamixel_protocol2.size(); i++)
    {
      dxl_addparam_result_ = groupSyncWrite.addParam(dynamixel_protocol2[i]->id_, (uint8_t*)&vel_value);
      if (dxl_addparam_result_ != true)
      {
        ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_protocol2[i]->id_);
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
}

bool DynamixelPositionControl::readRealtimeTick(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<uint16_t> *dxl_read_data)
{
  std::vector<dxl_motor::DxlMotor *> dynamixel_protocol1;
  std::vector<dxl_motor::DxlMotor *> dynamixel_protocol2;

  for (int i = 0; i < dynamixel.size(); i++)
  {
    if (dynamixel[i]->protocol_version_ == 1.0)
    {
      dynamixel_protocol1.push_back(dynamixel[i]);
    }
    else
    {
      dynamixel_protocol2.push_back(dynamixel[i]);
    }

  }

  if(packetHandler_->getProtocolVersion() == 1.0)
  {
// dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, ADDR_XM_REALTIME_TICK, 2);

// for (int i = 0; i < dxl_id_vec.size(); i++)
// {
//   dxl_addparam_result_ = groupSyncRead.addParam(dxl_id_vec.at(i));
//   if (dxl_addparam_result_ != true)
//   {
//     ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dxl_id_vec.at(i));
//     return false;
//   }
// }

// // Syncread present position
// dxl_comm_result_ = groupSyncRead.txRxPacket();
// if (dxl_comm_result_ != COMM_SUCCESS)
// {
//   packetHandler_->printTxRxResult(dxl_comm_result_);
// }

// // Check if groupsyncread data is available
// for (int i = 0; i < dxl_id_vec.size(); i++)
// {
//   dxl_getdata_result_ = groupSyncRead.isAvailable(dxl_id_vec.at(i), ADDR_XM_REALTIME_TICK, 2);
//   if (dxl_getdata_result_ != true)
//   {
//     fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_id_vec.at(i));
//     return false;
//   }
// }

// dxl_read_data->clear();

// // Get present position value
// for (int i = 0; i < dxl_id_vec.size(); i++)
// {
//   dxl_read_data->push_back(groupSyncRead.getData(dxl_id_vec.at(i), ADDR_XM_REALTIME_TICK, 2));
// }

// groupSyncRead.clearParam();
// return true;
  }
  else
  {
    dynamixel::GroupSyncRead groupSyncRead(portHandler_,
                                           packetHandler_,
                                           dynamixel_protocol2[0]->realtime_tick_item->address,
                                           dynamixel_protocol2[0]->realtime_tick_item->data_length);

    for (int i = 0; i < dynamixel_protocol2.size(); i++)
    {
      dxl_addparam_result_ = groupSyncRead.addParam(dynamixel_protocol2[i]->id_);
      if (dxl_addparam_result_ != true)
      {
        ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dynamixel_protocol2[i]->id_);
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
    for (int i = 0; i < dynamixel_protocol2.size(); i++)
    {
      dxl_getdata_result_ = groupSyncRead.isAvailable(dynamixel_protocol2[i]->id_,
                                                      dynamixel_protocol2[0]->realtime_tick_item->address,
                                                      dynamixel_protocol2[0]->realtime_tick_item->data_length);

      if (dxl_getdata_result_ != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dynamixel_protocol2[i]->id_);
        return false;
      }
    }

    dxl_read_data->clear();

    // Get present position value
    for (int i = 0; i < dynamixel_protocol2.size(); i++)
    {
      dxl_read_data->push_back(groupSyncRead.getData(dynamixel_protocol2[i]->id_,
                                                     dynamixel_protocol2[0]->realtime_tick_item->address,
                                                     dynamixel_protocol2[0]->realtime_tick_item->data_length));
    }

    groupSyncRead.clearParam();
    return true;
  }
}

bool DynamixelPositionControl::readOperatingMode(std::vector<uint8_t> dxl_id_vec, std::vector<uint16_t> *dxl_read_data)
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

bool DynamixelPositionControl::readTorque(std::vector<uint8_t> dxl_id_vec, std::vector<bool> *dxl_read_data)
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

bool DynamixelPositionControl::readGoalPosition(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data)
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

bool DynamixelPositionControl::readPresentPosition(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data)
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

bool DynamixelPositionControl::readProfileVelocity(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data)
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

bool DynamixelPositionControl::readPresentVelocity(std::vector<uint8_t> dxl_id_vec, std::vector<uint32_t> *dxl_read_data)
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

bool DynamixelPositionControl::readVoltage(std::vector<uint8_t> dxl_id_vec, std::vector<uint16_t> *dxl_read_data)
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

bool DynamixelPositionControl::readTemperature(std::vector<uint8_t> dxl_id_vec, std::vector<uint8_t> *dxl_read_data)
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

bool DynamixelPositionControl::readIsMoving(std::vector<uint8_t> dxl_id_vec, std::vector<bool> *dxl_read_data)
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
