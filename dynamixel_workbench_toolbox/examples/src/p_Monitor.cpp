/*******************************************************************************
  Copyright 2016 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include <DynamixelWorkbench.h>
#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <iostream>

using namespace std;

#define ESC_ASCII_VALUE             0x1b
#define SPACEBAR_ASCII_VALUE        0x20
#define ENTER_ASCII_VALUE           0x0a

uint8_t get_id[16];
uint8_t scan_cnt = 0;
uint8_t ping_cnt = 0;

int getch(void);
int kbhit(void);
bool isAvailableID(uint8_t id);
void printInst();
bool monitoring(const char* port_name);

DynamixelWorkbench dxl_wb;

int main(int argc, char *argv[]) 
{
  const char* port_name = "/dev/ttyUSB0";

  if (argc < 2)
  {
    printf("Please set '-port_name' arguments for connected Dynamixels\n");
    return 0;
  }
  else
  {
    port_name = argv[1];
  }

  printInst();

  while(true)
  {
    monitoring(port_name);
  }

  return 0;
}

bool monitoring(const char* port_name)
{
  char input[128];
  char cmd[80];
  char param[20][30];
  int num_param = 0;
  char *token;
  bool valid_cmd = false;

  bool wb_result = false;
  const char *log;

  if (kbhit())
  {
    if (getchar() == ENTER_ASCII_VALUE)
    {
      printf("[CMD]");
      fgets(input, sizeof(input), stdin);

      char *p;
      if ((p = strchr(input, '\n'))!= NULL) *p = '\0';
      fflush(stdin);

      if (strlen(input) == 0) return false;

      token = strtok(input, " ");

      if (token == 0) return false;

      strcpy(cmd, token);
      token = strtok(0, " ");
      num_param = 0;

      while (token != 0)
      {
        strcpy(param[num_param++], token);
        token = strtok(0, " ");
      }

      if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0)
      {
        printInst();
      }
      else if (strcmp(cmd, "begin") == 0)
      {
        uint32_t baud = 57600;
        
        baud = atoi(param[0]);
        wb_result = dxl_wb.init(port_name, baud, &log);
        if (wb_result == false)
        {
          printf("%s\n", log);
          printf("Failed to init\n");
        }
        else
          printf("Succeed to init(%d)\n", baud);          
      }
      else if (strcmp(cmd, "end") == 0)
      {        
        exit(0);
      }
      else if (strcmp(cmd, "scan") == 0)
      {
        uint8_t range = 253;  // default
        
        range = atoi(param[0]);
        wb_result = dxl_wb.scan(get_id, &scan_cnt, range, &log);
        if (wb_result == false)
        {
          printf("%s\n", log);
          printf("Failed to scan\n");
        }
        else
        {
          printf("Find %d Dynamixels\n", scan_cnt);

          for (int cnt = 0; cnt < scan_cnt; cnt++)
            printf("id : %d\n", get_id[cnt]);
        }
      }
      else if (strcmp(cmd, "ping") == 0)
      {
        get_id[ping_cnt] = 1; // default to 1
        
        get_id[ping_cnt] = atoi(param[0]);

        uint16_t model_number = 0;

        wb_result = dxl_wb.ping(get_id[ping_cnt], &model_number, &log);
        if (wb_result == false)
        {
          printf("%s\n", log);
          printf("Failed to ping\n");
        }
        else
        {
          printf("id : %d, model_number : %d\n", get_id[ping_cnt], model_number);
          ping_cnt++;
        }
      }
      else if (isAvailableID(atoi(param[0])))
      {
        if (strcmp(cmd, "control_table") == 0)
        {
          uint8_t id = atoi(param[0]);

          const ControlItem *control_item =  dxl_wb.getControlTable(id);
          uint8_t the_number_of_control_item = dxl_wb.getTheNumberOfControlItem(id);

          uint16_t last_register_addr = control_item[the_number_of_control_item-1].address;
          uint16_t last_register_addr_length = control_item[the_number_of_control_item-1].data_length;

          uint32_t getAllRegisteredData[last_register_addr+last_register_addr_length];

          if (control_item != NULL)
          {
            wb_result = dxl_wb.readRegister(id, (uint16_t)0, last_register_addr+last_register_addr_length, getAllRegisteredData, &log);
            if (wb_result == false)
            {
              printf("%s\n", log);
              return 0;
            }
            else
            {
              for (int index = 0; index < the_number_of_control_item; index++)
              {
                uint32_t data = 0;

                if (dxl_wb.getProtocolVersion() == 2.0f)
                {
                  data = getAllRegisteredData[control_item[index].address];
                  printf("\t%s : %d\n", control_item[index].item_name, data);
                }
                else if (dxl_wb.getProtocolVersion() == 1.0f)
                {
                  switch (control_item[index].data_length)
                  {
                    case BYTE:
                      data = getAllRegisteredData[control_item[index].address];
                      printf("\t%s : %d\n", control_item[index].item_name, data);
                    break;

                    case WORD:
                      data = DXL_MAKEWORD(getAllRegisteredData[control_item[index].address], getAllRegisteredData[control_item[index].address+1]);
                      printf("\t%s : %d\n", control_item[index].item_name, data);
                    break;

                    case DWORD:
                      data = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[control_item[index].address],   getAllRegisteredData[control_item[index].address+1]),
                                          DXL_MAKEWORD(getAllRegisteredData[control_item[index].address+2], getAllRegisteredData[control_item[index].address+3]));
                      printf("\t%s : %d\n", control_item[index].item_name, data);
                    break;

                    default:
                      data = getAllRegisteredData[control_item[index].address];
                    break;
                  } 
                }
              }
            }
          }
        }
        else if (strcmp(cmd, "sync_write_handler") == 0)
        {
          static uint8_t sync_write_handler_index = 0;
          uint8_t id = atoi(param[0]);
          wb_result = dxl_wb.addSyncWriteHandler(id, param[1], &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to add sync write handler\n");
            return 0;
          }
          else
            printf("%s, sync_write_handler_index = %d\n", log, sync_write_handler_index++);
        }
        else if (strcmp(cmd, "sync_read_handler") == 0)
        {
          static uint8_t sync_read_handler_index = 0;
          uint8_t id = atoi(param[0]);
          wb_result = dxl_wb.addSyncReadHandler(id, param[1], &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to add sync read handler\n");
            return 0;
          }
          else
            printf("%s, sync_read_handler_index = %d\n", log, sync_read_handler_index++);
        }
        else if (strcmp(cmd, "bulk_write_handler") == 0)
        {
          wb_result = dxl_wb.initBulkWrite(&log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to init bulk write handler\n");
            return 0;
          }
          else
            printf("%s\n", log);
        }
        else if (strcmp(cmd, "bulk_write_param") == 0)
        {
          uint8_t id = atoi(param[0]);
          wb_result = dxl_wb.addBulkWriteParam(id, param[1], atoi(param[2]), &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to add param for bulk write\n");
            return 0;
          }
          else
            printf("%s\n", log);
        }
        else if (strcmp(cmd, "bulk_write") == 0)
        {
          wb_result = dxl_wb.bulkWrite(&log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to bulk write\n");
            return 0;
          }
          else
            printf("%s\n", log);
        }
        else if (strcmp(cmd, "bulk_read_handler") == 0)
        {
          wb_result = dxl_wb.initBulkRead(&log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to init bulk read handler\n");
            return 0;
          }
          else
            printf("%s\n", log);
        }
        else if (strcmp(cmd, "bulk_read_param") == 0)
        {
          uint8_t id = atoi(param[0]);
          wb_result = dxl_wb.addBulkReadParam(id, param[1], &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to add param for bulk read\n");
            return 0;
          }
          else
            printf("%s\n", log);
        }
        else if (strcmp(cmd, "bulk_read") == 0)
        {
          wb_result = dxl_wb.bulkRead(&log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to bulk read\n");
            return 0;
          }
          else
            printf("%s\n", log);

          int32_t get_data[dxl_wb.getTheNumberOfBulkReadParam()];
          wb_result = dxl_wb.getBulkReadData(&get_data[0], &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to get bulk read data\n");
            return 0;
          }
          else
          {
            printf("%s\n", log);
            for (uint8_t index = 0; index < dxl_wb.getTheNumberOfBulkReadParam(); index++)
              printf("data[%d] : %d  ", index, get_data[index]);

            printf("\n");
          }

          dxl_wb.clearBulkReadParam();
        }
        else if (isAvailableID(atoi(param[0])) && isAvailableID(atoi(param[1])))
        {
          if (strcmp(cmd, "sync_write") == 0)
          {
            uint8_t id_1 = atoi(param[0]);
            uint8_t id_2 = atoi(param[1]);
            uint8_t id[2] = {id_1, id_2};
            uint8_t id_num = 2;    

            int32_t data[2] = {0, 0};
            data[0] = atoi(param[3]);
            data[1] = atoi(param[4]);

            uint8_t handler_index = atoi(param[2]);

            wb_result = dxl_wb.syncWrite(handler_index, id, id_num, (int32_t *)data, 1, &log);
            if (wb_result == false)
            {
              printf("%s\n", log);
              return 0;
            }
            else
              printf("%s\n", log);
          }
          else if (strcmp(cmd, "sync_read") == 0)
          {
            uint8_t id_1 = atoi(param[0]);
            uint8_t id_2 = atoi(param[1]);
            uint8_t id[2] = {id_1, id_2};
            uint8_t id_num = 2;

            int32_t data[2] = {0, 0};
            uint8_t handler_index = atoi(param[2]);

            wb_result = dxl_wb.syncRead(handler_index, id, id_num, &log);
            if (wb_result == false)
            {
              printf("%s\n", log);
              return 0;
            }
            else
            {
              printf("%s\n", log);
            }

            wb_result = dxl_wb.getSyncReadData(handler_index, id, id_num, data, &log);
            if (wb_result == false)
            {
              printf("%s\n", log);
              return 0;
            }
            else
            {
              printf("%s\n", log);
              printf("id[%d] : %d, id[%d] : %d\n", id_1, data[0], id_2, data[1]);
            }
          }
        }
        else if (strcmp(cmd, "id") == 0)
        {
          uint8_t id     = atoi(param[0]);
          uint8_t new_id = atoi(param[1]);

          wb_result = dxl_wb.changeID(id, new_id, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            return 0;
          }
          else
          {
            printf("%s\n", log);
          }
        }
        else if (strcmp(cmd, "baud") == 0)
        {
          uint8_t  id        = atoi(param[0]);
          uint32_t new_baud  = atoi(param[1]);

          wb_result = dxl_wb.changeBaudrate(id, new_baud, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            return 0;
          }
          else
          {
            wb_result = dxl_wb.setBaudrate(new_baud, &log);
            printf("%s\n", log);
          }
        }
        else if (strcmp(cmd, "torque_on") == 0)
        {
          uint8_t id    = atoi(param[0]);

          wb_result = dxl_wb.torqueOn(id, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            return 0;
          }
          else
          {
            printf("%s\n", log);
          }
        }
        else if (strcmp(cmd, "torque_off") == 0)
        {
          uint8_t id    = atoi(param[0]);

          wb_result = dxl_wb.torqueOff(id, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            return 0;
          }
          else
          {
            printf("%s\n", log);
          }
        }
        else if (strcmp(cmd, "joint") == 0)
        {
          uint8_t id    = atoi(param[0]);
          uint32_t goal = atoi(param[1]);

          wb_result = dxl_wb.jointMode(id, 0, 0, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            return 0;
          }
          else
          {
            printf("%s\n", log);
          }

          wb_result = dxl_wb.goalPosition(id, (int32_t)goal, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            return 0;
          }
          else
          {
            printf("%s\n", log);
          }
        }
        else if (strcmp(cmd, "wheel") == 0)
        {
          uint8_t id    = atoi(param[0]);
          uint32_t goal = atoi(param[1]);

          wb_result = dxl_wb.wheelMode(id, 0, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            return 0;
          }
          else
          {
            printf("%s\n", log);
          }

          wb_result = dxl_wb.goalVelocity(id, (int32_t)goal, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            return 0;
          }
          else
          {
            printf("%s\n", log);
          }
        }
        else if (strcmp(cmd, "write") == 0)
        {
          uint8_t id = atoi(param[0]);
          int32_t value = atoi(param[2]);

          wb_result = dxl_wb.writeRegister(id, param[1], value, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to write\n");
            return 0;
          }
          else
          {
            printf("%s\n", log);
          }
        }
        else if (strcmp(cmd, "read") == 0)
        {
          uint8_t id = atoi(param[0]);

          int32_t data = 0;
          
          wb_result = dxl_wb.readRegister(id, param[1], &data, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to read\n");
            return 0;
          }
          else
          {
            printf("%s\n", log);
            printf("read data : %d\n", data);
          }
        }
        else if (strcmp(cmd, "reboot") == 0)
        {
          uint8_t id = atoi(param[0]);

          wb_result = dxl_wb.reboot(id, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to reboot\n");
            return 0;
          }
          else
          {
            printf("%s\n", log);
          }
        }
        else if (strcmp(cmd, "reset") == 0)
        {
          uint8_t id = atoi(param[0]);

          wb_result = dxl_wb.reset(id, &log);
          if (wb_result == false)
          {
            printf("%s\n", log);
            printf("Failed to reset\n");
            return 0;
          }
          else
          {
            printf("%s\n", log);
          }
        }
        else
        {
          printf("Wrong command\n");
        }
      }
      else
      {
        printf("Please check ID\n");
      }
    }
  }

  return 0;
}

bool isAvailableID(uint8_t id)
{
  for (int dxl_cnt = 0; dxl_cnt < (scan_cnt + ping_cnt); dxl_cnt++)
  {
    if (get_id[dxl_cnt] == id)
      return true;
  }

  return false;
}

void printInst(void)
{
  printf("-------------------------------------\n");
  printf("Set begin before scan or ping\n");
  printf("-------------------------------------\n");
  printf("help\n");
  printf("begin  (BAUD)\n");
  printf("scan   (RANGE)\n");
  printf("ping   (ID)\n");
  printf("control_table (ID)\n");
  printf("id     (ID) (NEW_ID)\n");
  printf("baud   (ID) (NEW_BAUD)\n");
  printf("torque_on (ID)\n");
  printf("torque_off (ID)\n");
  printf("joint  (ID) (GOAL_POSITION)\n");
  printf("wheel  (ID) (GOAL_VELOCITY)\n");
  printf("write  (ID) (ADDRESS_NAME) (DATA)\n");
  printf("read   (ID) (ADDRESS_NAME)\n");
  printf("sync_write_handler (Ref_ID) (ADDRESS_NAME)\n");
  printf("sync_write (ID_1) (ID_2) (HANDLER_INDEX) (PARAM_1) (PARAM_2)\n");
  printf("sync_read_handler (Ref_ID) (ADDRESS_NAME)\n");
  printf("sync_read (ID_1) (ID_2) (HANDLER_INDEX)\n");
  printf("bulk_write_handler\n");
  printf("bulk_write_param (ID) (ADDRESS_NAME) (PARAM)\n");
  printf("bulk_write\n");
  printf("bulk_read_handler\n");
  printf("bulk_read_param (ID) (ADDRESS_NAME)\n");
  printf("bulk_read\n");
  printf("reboot (ID) \n");
  printf("reset  (ID) \n");
  printf("end\n");
  printf("-------------------------------------\n");
  printf("Press Enter Key\n");
}

int getch()
{
  struct termios oldt, newt;
  int ch;

  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 1;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

  return ch;
}

int kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}
