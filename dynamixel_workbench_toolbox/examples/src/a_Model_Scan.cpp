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

#define DEVICE_NAME "/dev/ttyUSB0"
#define BAUDRATE  1000000

int main(int argc, char *argv[]) 
{
  DynamixelWorkbench dxl_wb;

  uint8_t scanned_id[16];
  uint8_t dxl_cnt = 0;
  uint8_t range = 100;

  dxl_wb.begin(DEVICE_NAME, BAUDRATE);
  dxl_wb.scan(scanned_id, &dxl_cnt, range);

  if (dxl_cnt == 0)
    printf("Can't find Dynamixels");
  for (int index = 0; index < dxl_cnt; index++)
    printf("ID : %d, Model Name : %s\n", scanned_id[index], dxl_wb.getModelName(scanned_id[index]));

  return 0;
}