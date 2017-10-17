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

#define BAUDRATE  1000000
#define DXL_1   1
#define DXL_2   2

DynamixelWorkbench dxl_wb;

int goal_position[2] = {1000, 0};
int error = 0;

void setup() 
{
  Serial.begin(57600);
  while(!Serial);

  dxl_wb.begin(DXL_BUS_SERIAL3, BAUDRATE);
  dxl_wb.ping(DXL_1);
  dxl_wb.ping(DXL_2);

  dxl_wb.jointMode(DXL_1);
  dxl_wb.jointMode(DXL_2);

  dxl_wb.initSyncWrite(DXL_1, "Goal Position");
  dxl_wb.initSyncRead(DXL_1, "Present Position");
}

void loop() 
{
  int read_position[2];
  dxl_wb.read("Present Position", read_position);
  Serial.print("DXL_1 : "); Serial.println(read_position[0]);
  Serial.print("DXL_2 : "); Serial.println(read_position[1]);

  if ((abs(goal_position[0] - read_position[0]) < 10) && 
      (abs(goal_position[1] - read_position[1]) < 10))
  {
    int tmp = goal_position[0];
    goal_position[0] = goal_position[1];
    goal_position[1] = tmp;

    dxl_wb.write(goal_position);
  }

}