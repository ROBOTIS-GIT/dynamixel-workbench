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
int present_position = 0;
int goal_position = 4000;

uint32_t tTime = 0;

void setup() 
{
  Serial.begin(57600);
  while(!Serial);

  dxl_wb.begin(DXL_BUS_SERIAL3, BAUDRATE);
  dxl_wb.ping(DXL_ID);

  dxl_wb.jointMode(DXL_ID);
}

void loop() 
{
  // uint32_t t = millis();
  // if ((t-tTime) >= (1000 / 30))
  // {
  //   dxl_wb.write(DXL_ID, "Goal Position", 0);
    
  //   present_position = dxl_wb.read(DXL_ID, "Present Position");
  //   Serial.print("Present Position : ");
  //   Serial.println(present_position);

  //   tTime = t;
  // }
  // goal_position = (-1) * goal_position;

  dxl_wb.write(DXL_ID, "Goal Position", goal_position);

  // do
  // {

  // }while(abs(present_position - goal_position) < 10);
}