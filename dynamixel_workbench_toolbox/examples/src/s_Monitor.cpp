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

#define DEVICE_NAME "/dev/ttyUSB0"

uint8_t get_id[16];
uint8_t scan_cnt = 0;
uint8_t ping_cnt = 0;

int getch(void);
int kbhit(void);
bool isAvailableID(uint8_t id);
void printInst();
bool monitoring();

DynamixelWorkbench dxl_wb;

int main(int argc, char *argv[]) 
{
  printInst();

  while(true)
  {
    monitoring();
  }

  return 0;
}

bool monitoring()
{
  char input[128];
  char cmd[80];
  char param[20][30];
  int num_param = 0;
  char *token;
  bool valid_cmd = false;

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
        if (dxl_wb.begin(DEVICE_NAME, baud))
          printf("Succeed to begin(%d)\n", baud);
        else
          printf("Failed to begin\n");
      }
      else if (strcmp(cmd, "scan") == 0)
      {
        uint8_t range = 100;  // default
        
        range = atoi(param[0]);
        dxl_wb.scan(get_id, &scan_cnt, range);

        if (scan_cnt == 0)
          printf("Can't find Dynamixel\n");
        else
        {
          printf("Find %d Dynamixels\n", scan_cnt);

          for (int cnt = 0; cnt < scan_cnt; cnt++)
            printf("ID : %d\n", get_id[cnt]);
        }
      }
      else if (strcmp(cmd, "ping") == 0)
      {
        get_id[ping_cnt] = 1; // default to 1
        
        get_id[ping_cnt] = atoi(param[0]);

        uint16_t model_number = 0;

        if (dxl_wb.ping(get_id[ping_cnt], &model_number))
        {
          printf("id : %d, model_number : %d\n", get_id[ping_cnt], model_number);
          ping_cnt++;
        }
        else
          printf("Failed to ping\n");
      }
      else if (isAvailableID(atoi(param[0])))
      {
        if (strcmp(cmd, "info") == 0)
        {
          // Lets loop through all of the information for this servo...
          uint8_t id     = atoi(param[0]);

          // first print model number stuff
          uint16_t model_number = 0;
          const ControlTableItem *cti =  dxl_wb.getControlItemPtr(id);
          uint8_t cti_count = dxl_wb.getControlItemCount(id);

          if (cti)
          {
            if (dxl_wb.ping(id, &model_number))
            {
              printf("id : %d, model_number : %d (%s)\n", id, model_number, dxl_wb.getModelName(id));
            }
            while (cti_count--)
            {
              printf("\t%s : %d\n", cti->item_name, dxl_wb.itemRead(id, cti->item_name));
              cti++;
            }
          }
        }

        else if (strcmp(cmd, "id") == 0)
        {
          uint8_t id     = atoi(param[0]);
          uint8_t new_id = atoi(param[1]);

          if (dxl_wb.setID(id, new_id))
            printf("Succeed to change ID");
          else
            printf("Failed");

        }
        else if (strcmp(cmd, "baud") == 0)
        {
          uint8_t  id       = atoi(param[0]);
          uint32_t  new_baud  = atoi(param[1]);

          if (dxl_wb.setBaud(id, new_baud))
            printf("Succeed to change BaudRate\n");
          else
            printf("Failed\n");

        }
        else if (strcmp(cmd, "torque") == 0)
        {
          uint8_t id       = atoi(param[0]);
          uint8_t onoff    = atoi(param[1]);

          if (dxl_wb.itemWrite(id, "Torque_Enable", onoff))
            printf("Succeed to torque command!!\n");
          else
            printf("Failed\n");

        }
        else if (strcmp(cmd, "joint") == 0)
        {
          uint8_t id    = atoi(param[0]);
          uint16_t goal = atoi(param[1]);

          dxl_wb.jointMode(id);
          if (dxl_wb.goalPosition(id, goal))
            printf("Succeed to joint command!!\n");
          else
            printf("Failed\n");

        }
        else if (strcmp(cmd, "wheel") == 0)
        {
          uint8_t id    = atoi(param[0]);
          int32_t goal  = atoi(param[1]);

          dxl_wb.wheelMode(id);
          if (dxl_wb.goalSpeed(id, goal))
            printf("Succeed to wheel command!!\n");
          else
            printf("Failed\n");
        }
        else if (strcmp(cmd, "write") == 0)
        {
          uint8_t id = atoi(param[0]);
          int32_t value = atoi(param[2]);

          if (dxl_wb.itemWrite(id, param[1], value))
          {
            printf("Succeed to write command!!\n");
          }
          else
            printf("Failed\n");
        }
        else if (strcmp(cmd, "read") == 0)
        {
          uint8_t id = atoi(param[0]);

          int32_t value = dxl_wb.itemRead(id, param[1]);

          printf("read data : %d\n", value);
        }
        else if (strcmp(cmd, "reboot") == 0)
        {
          uint8_t id = atoi(param[0]);

          if (dxl_wb.reboot(id))
            printf("Succeed to reboot\n");
          else
            printf("Failed to reboot\n");
        }
        else if (strcmp(cmd, "reset") == 0)
        {
          uint8_t id = atoi(param[0]);

          if (dxl_wb.reset(id))
            printf("Succeed to reset\n");
          else
            printf("Failed to reset\n");
        }
        else
        {
          printf("Wrong command\n");
        }
      }
      else if (strcmp(cmd, "exit") == 0)
      {
        exit(0);
      }
      else
      {
        printf("Please check ID\n");
      }
    }
  }

  return true;
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
  printf("Set portHandler Before scan or ping\n");
  printf("-------------------------------------\n");
  printf("help\n");
  printf("begin  (BAUD)\n");
  printf("scan   (RANGE)\n");
  printf("ping   (ID)\n");
  printf("info   (ID)\n");
  printf("id     (ID) (NEW_ID)\n");
  printf("baud   (ID) (NEW_BAUD)\n");
  printf("torque (ID) (VALUE)\n");
  printf("joint  (ID) (GOAL_POSITION)\n");
  printf("wheel  (ID) (GOAL_VELOCITY)\n");
  printf("write  (ID) (ADDRESS_NAME) (VALUE)\n");
  printf("read   (ID) (ADDRESS_NAME)\n");
  printf("reboot (ID) \n");
  printf("reset  (ID) \n");
  printf("exit\n");
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
