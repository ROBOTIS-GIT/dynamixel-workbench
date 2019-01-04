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

int main(int argc, char *argv[]) 
{
  const char* port_name = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int dxl_id = 1;
  int new_dxl_id = 2;

  if (argc < 5)
  {
    printf("Please set '-port_name', '-baud_rate', '-dynamixel id', '-new_dynamixel_id' arguments for connected Dynamixels\n");
    return 0;
  }
  else
  {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
    dxl_id = atoi(argv[3]);
    new_dxl_id = atoi(argv[4]);
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

  result = dxl_wb.changeID(dxl_id, new_dxl_id, &log);
  if (result == false)
  {
    printf("%s\n", log);
    return 0;
  }
  else
  {
    printf("%s\n", log);
  }

  uint8_t scanned_id[16];
  uint8_t dxl_cnt = 0;
  uint8_t range = 100;

  printf("Wait for scan...\n");
  result = dxl_wb.scan(scanned_id, &dxl_cnt, range, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to scan\n");
  }
  else
  {
    printf("Find %d Dynamixels\n", dxl_cnt);

    for (int cnt = 0; cnt < dxl_cnt; cnt++)
      printf("id : %d, model name : %s\n", scanned_id[cnt], dxl_wb.getModelName(scanned_id[cnt]));
  }
}