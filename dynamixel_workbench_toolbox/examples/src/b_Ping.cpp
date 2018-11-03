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

#define DEVICE_NAME "/dev/tty.usbserial-FT1CTA16"
// #define DEVICE_NAME "/dev/ttyUSB0"
#define BAUDRATE  57600

#define DXL_ID  1

#define ADDR_PRESENT_CURRENT_2 126
#define ADDR_PRESENT_VELOCITY_2 128
#define ADDR_PRESENT_POSITION_2 132

#define LENGTH_PRESENT_CURRENT_2 2
#define LENGTH_PRESENT_VELOCITY_2 4
#define LENGTH_PRESENT_POSITION_2 4

int main(int argc, char *argv[]) 
{
  DynamixelWorkbench dxl_wb;

  const char *log;
  bool result = false;

  uint8_t id = DXL_ID;
  uint16_t model_number = 0;

  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init\n");
  }
  else
    printf("Succeed to init(%d)\n", BAUDRATE);  

  result = dxl_wb.ping(DXL_ID, &model_number, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to ping\n");
  }
  else
  {
    printf("id : %d, model_number : %d\n", DXL_ID, model_number);
  }


  result = dxl_wb.addSyncReadHandler(ADDR_PRESENT_CURRENT_2, 
                                                    (LENGTH_PRESENT_CURRENT_2 + LENGTH_PRESENT_VELOCITY_2 + LENGTH_PRESENT_POSITION_2), 
                                                    &log);
  if (result == false)
  {
    printf("%s\n", log);

  }
  else
  {
    printf("%s\n", log);
  }

  result = dxl_wb.syncRead(0, 
                                          &id, 
                                          1, 
                                          &log);
  if (result == false)
  {
    printf("%s\n", log);

  }
  else
  {
    printf("%s\n", log);
  }

  int32_t get_value = 0;
  result = dxl_wb.getSyncReadData(0, 
                                            &id, 
                                            1,
                                            ADDR_PRESENT_CURRENT_2,
                                            LENGTH_PRESENT_CURRENT_2 + LENGTH_PRESENT_VELOCITY_2 + LENGTH_PRESENT_POSITION_2,
                                            &get_value, 
                                            &log);
  if (result == false)
  {
    printf("%s\n", log);
  } 
  else
  {
    printf("%s\n", log);
    printf("%d\n", get_value);
  }



}