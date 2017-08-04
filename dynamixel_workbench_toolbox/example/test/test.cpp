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

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_toolbox.h"

#define DEVICE_NAME      "/dev/ttyUSB0"
#define BAUD_RATE        57600
#define PROTOCOL_VERSION 2.0

#define ID 1

#define ENABLE  1
#define DISABLE 0

dynamixel_driver::DynamixelDriver *dynamixel_driver_;

void setup()
{
  dynamixel_driver_ = new dynamixel_driver::DynamixelDriver(DEVICE_NAME,
                                                            BAUD_RATE,
                                                            PROTOCOL_VERSION);

  dynamixel_driver_->ping(ID);

  dynamixel_driver_->writeRegister("torque_enable", ENABLE);
}

int main()
{
  setup();

  while(1)
  {
    dynamixel_driver_->writeRegister("goal_position", 0);

    sleep(1.5);

    dynamixel_driver_->writeRegister("goal_position", 1000);

    sleep(1.5);
  }
  
  return 0;
}
