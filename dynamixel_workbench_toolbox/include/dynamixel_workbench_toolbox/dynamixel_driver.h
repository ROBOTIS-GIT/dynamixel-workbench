/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_DRIVER_H

#include "dynamixel_tool.h"

#if defined(__OPENCR__) || defined(__OPENCM904__)
  #include <Arduino.h>
  #include <DynamixelSDK.h>
#elif defined(__linux__) || defined(__APPLE__)
  #include "unistd.h"
  #include "dynamixel_sdk/dynamixel_sdk.h"
#endif

#define MAX_DXL_SERIES_NUM 5
#define MAX_HANDLER_NUM 5

typedef struct 
{
  const ControlItem *control_item; 
  dynamixel::GroupSyncWrite *groupSyncWrite;    
} SyncWriteHandler;

typedef struct 
{
  const ControlItem *control_item;
  dynamixel::GroupSyncRead  *groupSyncRead;     
} SyncReadHandler;

typedef struct
{
  int dxl_comm_result;
  bool dxl_addparam_result;
  bool dxl_getdata_result;
  uint8_t dxl_error;
} ErrorFromSDK;

class DynamixelDriver
{
 private:
  dynamixel::PortHandler   *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  SyncWriteHandler syncWriteHandler_[MAX_HANDLER_NUM];
  SyncReadHandler  syncReadHandler_[MAX_HANDLER_NUM];

  dynamixel::GroupBulkRead  *groupBulkRead_;  
  dynamixel::GroupBulkWrite *groupBulkWrite_;  
 
  DynamixelTool tools_[MAX_DXL_SERIES_NUM];

  uint8_t tools_cnt_;
  uint8_t sync_write_handler_cnt_;
  uint8_t sync_read_handler_cnt_;

 public:
  DynamixelDriver();
  ~DynamixelDriver();

  bool init(const char* device_name = "/dev/ttyUSB0", 
            uint32_t baud_rate = 57600, 
            const char **log = NULL);

  bool setPortHandler(const char *device_name, const char **log = NULL);
  bool setBaudrate(uint32_t baud_rate, const char **log = NULL);
  bool setPacketHandler(float protocol_version, const char **log = NULL);

  float getProtocolVersion(void);
  uint32_t getBaudrate(void);

  const char * getModelName(uint8_t id, const char **log = NULL);
  uint16_t getModelNumber(uint8_t id, const char **log = NULL);
  const ControlItem *getControlTable(uint8_t id, const char **log = NULL);
  uint8_t getTheNumberOfControlItem(uint8_t id, const char **log = NULL);

  bool scan(uint8_t *get_id,
            uint8_t *get_the_number_of_id, 
            uint8_t range = 253,
            const char **log = NULL);

  bool scan(uint8_t *get_id,
            uint8_t *get_the_number_of_id, 
            uint8_t start_number,
            uint8_t end_number = 253,
            const char **log = NULL);

  bool ping(uint8_t id, 
            uint16_t *get_model_number = NULL,
            const char **log = NULL);

  bool reboot(uint8_t id, const char **log = NULL);
  bool reset(uint8_t id, const char **log = NULL);

  bool writeRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t* data, const char **log = NULL);

  bool writeRegister(uint8_t id, const char *item_name, uint8_t data, const char **log = NULL);
  bool writeRegister(uint8_t id, const char *item_name, uint16_t data, const char **log = NULL);
  bool writeRegister(uint8_t id, const char *item_name, uint32_t data, const char **log = NULL);

  bool writeOnlyRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, const char **log = NULL);

  bool writeOnlyRegister(uint8_t id, const char *item_name, uint8_t data, const char **log = NULL);
  bool writeOnlyRegister(uint8_t id, const char *item_name, uint16_t data, const char **log = NULL);
  bool writeOnlyRegister(uint8_t id, const char *item_name, uint32_t data, const char **log = NULL);  

  bool readRegister(uint8_t id, uint16_t address, uint16_t length, uint32_t *data, const char **log = NULL);

  bool readRegister(uint8_t id, const char *item_name, uint8_t *data, const char **log = NULL);
  bool readRegister(uint8_t id, const char *item_name, uint16_t *data, const char **log = NULL);
  bool readRegister(uint8_t id, const char *item_name, uint32_t *data, const char **log = NULL);

  bool addSyncWriteHandler(uint8_t id, uint16_t address, uint16_t length, const char **log = NULL);
  bool addSyncWriteHandler(uint8_t id, const char *item_name, const char **log = NULL);

  bool getParam(uint16_t *data, uint8_t *param);
  bool getParam(uint32_t *data, uint8_t *param);

  bool syncWrite(uint8_t index, uint8_t *data, const char **log = NULL);
  bool syncWrite(uint8_t index, uint8_t *id, uint8_t id_num, uint8_t *data, const char **log = NULL);

  bool addSyncReadHandler(uint8_t id, uint16_t address, uint16_t length, const char **log = NULL);
  bool addSyncReadHandler(uint8_t id, const char *item_name, const char **log = NULL);

  bool syncRead(uint8_t index, uint32_t *data, const char **log = NULL);

  // void initBulkWrite();
  // bool addBulkWriteParam(uint8_t id, const char *item_name, int32_t data);
  // bool bulkWrite();

  // void initBulkRead();
  // bool addBulkReadParam(uint8_t id, const char *item_name);
  // bool sendBulkReadPacket();
  // bool bulkRead(uint8_t id, const char *item_name, int32_t *data);

  // int32_t convertRadian2Value(uint8_t id, float radian);
  // float convertValue2Radian(uint8_t id, int32_t value);

  // int32_t convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian = 3.14, float min_radian = -3.14);
  // float convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian = 3.14, float min_radian = -3.14);

  // int32_t convertVelocity2Value(uint8_t id, float velocity);
  // float convertValue2Velocity(uint8_t id, int32_t value);

  // int16_t convertTorque2Value(uint8_t id, float torque);
  // float convertValue2Torque(uint8_t id, int16_t value);

 private:
  void initTools(void);
  bool setTool(uint16_t model_number, uint8_t id, const char **log = NULL);
  uint8_t getTool(uint8_t id, const char **log = NULL);

  void wait(uint16_t msec);
};

#endif //DYNAMIXEL_DRIVER_H
