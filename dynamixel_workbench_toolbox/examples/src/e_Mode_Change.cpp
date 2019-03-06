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

#include <DynamixelWorkbench.h>

int main(int argc, char *argv[]) 
{
  const char* port_name = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int dxl_id = 1;
  int mode = 0;

  if (argc < 4)
  {
    printf("Please set '-port_name', '-baud_rate', '-dynamixel id' arguments for connected Dynamixels\n");
    return 0;
  }
  else
  {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
    dxl_id = atoi(argv[3]);
  }

  DynamixelWorkbench dxl_wb;

  const char *log;
  bool result = false;

  uint16_t model_number = 0;

  result = dxl_wb.init(port_name, baud_rate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init\n");

    return 0;
  }
  else
    printf("Succeeded to init(%d)\n", baud_rate);  

  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to ping\n");
  }
  else
  {
    printf("Succeeded to ping\n");
    printf("id : %d, model_number : %d\n", dxl_id, model_number);
  }

  printf("---> \n");
  printf("Please insert the right number from 0 to 5 to set control mode \n");
  printf("0 - current control mode\n");
  printf("1 - velocity control mode\n");
  printf("2 - position control mode\n");
  printf("3 - extended position control mode\n");
  printf("4 - current based position control mode\n");
  printf("5 - pwm control mode\n");
  printf("<--- \n\n");

  scanf("%d", &mode);

  switch (mode)
  {
    case 0:
      result = dxl_wb.setCurrentControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
        printf("Failed to set mode\n");
      }
      else
      {
        printf("Succeeded to set mode\n");
      }
     break;

    case 1:
      result = dxl_wb.setVelocityControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
        printf("Failed to set mode\n");
      }
      else
      {
        printf("Succeeded to set mode\n");
      }
     break;

    case 2:
      result = dxl_wb.setPositionControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
        printf("Failed to set mode\n");
      }
      else
      {
        printf("Succeeded to set mode\n");
      }
     break;

    case 3:
      result = dxl_wb.setExtendedPositionControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
        printf("Failed to set mode\n");
      }
      else
      {
        printf("Succeeded to set mode\n");
      }
     break;

    case 4:
      result = dxl_wb.setCurrentBasedPositionControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
        printf("Failed to set mode\n");
      }
      else
      {
        printf("Succeeded to set mode\n");
      }
     break;

    case 5:
      result = dxl_wb.setPWMControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
        printf("Failed to set mode\n");
      }
      else
      {
        printf("Succeeded to set mode\n");
      }
     break;

    default:
      result = dxl_wb.setPositionControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
        printf("Failed to set mode\n");
      }
      else
      {
        printf("Succeeded to set mode\n");
      }
     break;
  }

  return 0;
}
