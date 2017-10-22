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

/* Authors: Taehun Lim (Darby) */

#include <DynamixelWorkbench.h>

#define DXL_BUS_SERIAL1 "1"            //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 "2"            //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 "3"            //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#define DXL_BUS_SERIAL4 "/dev/ttyUSB0" //Dynamixel on Serial3(USART3)  <-OpenCR

#define BAUDRATE  57600
#define DXL_ID    1

DynamixelWorkbench dxl_wb;

void setup() 
{
  Serial.begin(57600);
  while(!Serial);

  dxl_wb.begin(DXL_BUS_SERIAL3, BAUDRATE);
  dxl_wb.ping(DXL_ID);

  // dxl_wb.jointMode(DXL_ID);
  dxl_wb.driver_.packetHandler_2->write1ByteTxRx(dxl_wb.driver_.portHandler_, 1, 64, 1);
  // dxl_wb.driver_.packetHandler_1->write1ByteTxRx(dxl_wb.driver_.portHandler_, 1, 13, 2);
  
  Serial.println("ok");
}

void loop() 
{
  static int index = 0;
  int32_t present_position = 0;
  uint16_t goal_position[2] = {1, 3000};
  
  // dxl_wb.write(DXL_ID, "Goal Position", goal_position[index]);
  
  dxl_wb.driver_.packetHandler_1->write4ByteTxRx(dxl_wb.driver_.portHandler_, 1, 116, goal_position[index]);

  do
  {
    dxl_wb.driver_.packetHandler_1->read4ByteTxRx(dxl_wb.driver_.portHandler_, 1, 132, (uint32_t *)&present_position); 
    //present_position = dxl_wb.read(DXL_ID, "Present Position");
    Serial.print("[ID:");      Serial.print(DXL_ID);
    Serial.print(" GoalPos:"); Serial.print(goal_position[index]);
    Serial.print(" PresPos:");  Serial.print(present_position);
    Serial.println(" ");
  }while(abs(goal_position[index] - present_position) > 20);

  if (index == 0)
  {
    index = 1;
  }
  else
  {
    index = 0;
  }
}

// DynamixelDriver drv;
// int32_t data;
// int32_t goal_position[2] = {1000, 0};

// void setup()
// {
//   Serial.begin(57600);
//   while(!Serial);

//   drv.begin(DXL_BUS_SERIAL3, BAUDRATE);
//   drv.ping(DXL_ID);

//   drv.packetHandler_->write1ByteTxRx(drv.portHandler_, 1, 64, 0);

//   drv.packetHandler_->write1ByteTxRx(drv.portHandler_, 1, 11, 3);

//   drv.packetHandler_->write1ByteTxRx(drv.portHandler_, 1, 64, 1);

//   drv.packetHandler_->write4ByteTxRx(drv.portHandler_, 1, 108, 0);
//   drv.packetHandler_->write4ByteTxRx(drv.portHandler_, 1, 112, 0);
// }

// void loop()
// {
//   static int index = 0;

//   drv.packetHandler_->write4ByteTxRx(drv.portHandler_, 1, 116, goal_position[index]);
  
//   do
//   {
//     drv.packetHandler_->read4ByteTxRx(drv.portHandler_, 1, 132, (uint32_t *)&data); 
//     Serial.print("[ID:");      Serial.print(DXL_ID);
//     Serial.print(" GoalPos:"); Serial.print(goal_position[index]);
//     Serial.print(" PresPos:");  Serial.print(data);
//     Serial.println(" ");
//   }while(abs(goal_position[index] - data) > 20);

//   if (index == 0)
//   {
//     index = 1;
//   }
//   else
//   {
//     index = 0;
//   }
// }