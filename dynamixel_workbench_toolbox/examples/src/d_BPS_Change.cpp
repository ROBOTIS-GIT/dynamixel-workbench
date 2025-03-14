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
  int dxl_id = 1;
  int baud_rate = 57600;
  int new_baud_rate = 1000000;

  if (argc < 5)
  {
    printf("Please set '-port_name', '-baud_rate', '-dynamixel_id', '-new_baud_rate' arguments for connected Dynamixels\n");
    return 0;
  }
  else
  {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
    dxl_id = atoi(argv[3]);
    new_baud_rate = atoi(argv[4]);
  }

  DynamixelWorkbench dxl_wb;

  const char *log;
  bool result = false;

  result = dxl_wb.init(port_name, baud_rate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init\n");

    return 0;
  }
  else
    printf("Succeeded to init(%d)\n", baud_rate);

  uint16_t model_number = 0;
  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to ping\n");

    return 0;
  }
  else
  {
    printf("Succeeded to ping\n");
    printf("id : %d, model_number : %d\n", dxl_id, model_number);
  }

  result = dxl_wb.changeBaudrate(dxl_id, new_baud_rate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    return 0;
  }
  else
  {
    printf("%s\n", log);
  }

  return 0;
}
