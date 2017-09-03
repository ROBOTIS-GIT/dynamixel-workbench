/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Taehoon Lim (Darby) */

#include <DynamixelWorkbench.h>

#define DEVICENAME       "3"
#define BAUDRATE         1000000

DynamixelDriver driver;

uint8_t model_cnt = 0;
uint8_t dxl[] = {0, };

void setup() 
{
  Serial.begin(57600);
  while(!Serial);


  driver.begin("XM", DEVICENAME, BAUDRATE);
  model_cnt = driver.scan(dxl);

  // driver.writeRegister(id[0], "Operating Mode", 3);
  // driver.writeRegister(id[1], "Operating Mode", 3);

  // driver.writeRegister(dxl[0], "Torque Enable", 1);
  // driver.writeRegister(dxl[1], "Torque Enable", 1);

  // driver.writeRegister(dxl[0], "Profile Velocity", 40);
  // driver.writeRegister(dxl[1], "Profile Velocity", 40);

  // driver.setSyncWrite(dxl[0], "Goal Position");

  // driver.setSyncRead(dxl[0], "Present Position");

  // driver.initBulkWrite();

  driver.initBulkRead();
  driver.addBulkReadParam(dxl[0], "Present Position");
  driver.addBulkReadParam(dxl[1], "Present Velocity");
  
  // DynamixelTool tool_;
  // tool_.begin("XM430-W350");

  // for (int i=0; i<tool_.control_table_size_; i++)
  // {
  //   uint16_t    address = tool_.item_[i].address;
  //   char*       item_name = tool_.item_[i].item_name;  
  //   uint8_t     data_length = tool_.item_[i].data_length;
  //   uint8_t     access_type = tool_.item_[i].access_type;
  //   uint8_t     memory_type = tool_.item_[i].memory_type;
    
  //   Serial.print("address : "); 
  //   Serial.print(address);
  //   Serial.print(" item_name : ");
  //   Serial.print(item_name);
  //   Serial.print(" data_length : ");
  //   Serial.print(data_length);
  //   Serial.print(" access_type : ");
  //   Serial.print(access_type);
  //   Serial.print(" memory_type : ");
  //   Serial.println(memory_type);
  // }
}

void loop() 
{
  // driver.writeRegister(dxl[0], "Goal Position", 2048);
  // driver.writeRegister(dxl[1], "Goal Position", 2048);
  
  // delay(1000);

  // driver.writeRegister(dxl[0], "Goal Position", 2548);  
  // driver.writeRegister(dxl[1], "Goal Position", 2548);

  // uint32_t value[2];
  // value[0] = 2048;
  // value[1] = 2048;

  // driver.syncWrite(value);
  
  // delay(1000);

  // value[0] = 2748;
  // value[1] = 2748;
  // driver.syncWrite(value);

  // delay(1000);  

  // delay(1000);  
  // int32_t data = 0;

  // driver.readRegister(dxl[0], "Present Position", &data);
  // Serial.println(data);

  
  // driver.syncRead("Present Position", value);
  // Serial.print("value 1 : ");
  // Serial.print(value[0]);
  // Serial.print("value 2 : ");
  // Serial.println(value[1]);

  // driver.addBulkWriteParam(dxl[1], "Goal Position", 1548);
  // driver.addBulkWriteParam(dxl[0], "LED", 1);

  // driver.bulkWrite();

  // delay(1000);

  // driver.addBulkWriteParam(dxl[1], "Goal Position", 2548);
  // driver.addBulkWriteParam(dxl[0], "LED", 0);

  // driver.bulkWrite();

  // delay(1000);

  int32_t data[2];
  driver.sendBulkReadPacket();
  driver.bulkRead(dxl[0], "Present Position", &data[0]);
  driver.bulkRead(dxl[1], "Present Velocity", &data[1]);

  Serial.print("data 1 : ");
  Serial.print(data[0]);
  Serial.print(" data 2 : ");
  Serial.println(data[1]);
}
