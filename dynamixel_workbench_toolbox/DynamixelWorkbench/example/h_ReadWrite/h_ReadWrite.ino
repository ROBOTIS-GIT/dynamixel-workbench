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

// DynamixelWorkbench dxl_wb;
// int32_t present_position = 0;
// int32_t goal_position[2] = {1000, 1};

// void setup() 
// {
//   Serial.begin(57600);
//   while(!Serial);

//   dxl_wb.begin(DXL_BUS_SERIAL3, BAUDRATE);
//   dxl_wb.ping(DXL_ID);

//   dxl_wb.jointMode(DXL_ID);

// }

// void loop() 
// {
//   static int index = 0;
  
//   dxl_wb.write(DXL_ID, "Goal Position", goal_position[index]);
  
//   do
//   {
//     present_position = dxl_wb.read(DXL_ID, "Present Position");
//   }while(abs(goal_position[index] - present_position) > 20);

//   if (index == 0)
//   {
//     index = 1;
//   }
//   else
//   {
//     index = 0;
//   }
// }

DynamixelDriver drv;
int32_t data;
int32_t goal_position[2] = {1000, 0};

void setup()
{
  Serial.begin(57600);
  while(!Serial);

  drv.begin(DXL_BUS_SERIAL3, BAUDRATE);
  drv.ping(DXL_ID);

  // drv.packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  drv.packetHandler_->write1ByteTxRx(drv.portHandler_, 1, 64, 1);
}

void loop()
{
  static int index = 0;

  drv.packetHandler_->write4ByteTxRx(drv.portHandler_, 1, 116, goal_position[index]);
  
  do
  {
    drv.packetHandler_->read4ByteTxRx(drv.portHandler_, 1, 132, (uint32_t *)&data); 
    Serial.print("[ID:");      Serial.print(DXL_ID);
    Serial.print(" GoalPos:"); Serial.print(goal_position[index]);
    Serial.print(" PresPos:");  Serial.print(data);
    Serial.println(" ");
  }while(abs(goal_position[index] - data) > 20);

  if (index == 0)
  {
    index = 1;
  }
  else
  {
    index = 0;
  }
}

/*
  Motor : XL320

  DEVICENAME "1" -> Serial1
  DEVICENAME "2" -> Serial2
  DEVICENAME "3" -> Serial3
*/

// #include <DynamixelSDK.h>


// // Control table address (XL320)
// #define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
// #define ADDR_PRO_GOAL_POSITION          116
// #define ADDR_PRO_PRESENT_POSITION       132

// // Protocol version
// #define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// // Default setting
// #define DXL_ID                          1                   // Dynamixel ID: 1
// #define BAUDRATE                        57600
// #define DEVICENAME                      "3"                 // Check which port is being used on your controller
//                                                             // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

// #define TORQUE_ENABLE                   1                   // Value for enabling the torque
// #define TORQUE_DISABLE                  0                   // Value for disabling the torque
// #define DXL_MINIMUM_POSITION_VALUE      1                 // Dynamixel will rotate between this value
// #define DXL_MAXIMUM_POSITION_VALUE      900                 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
// #define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

// #define ESC_ASCII_VALUE                 0x1b



// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
//   while(!Serial);


//   Serial.println("Start..");


//   // Initialize PortHandler instance
//   // Set the port path
//   // Get methods and members of PortHandlerLinux or PortHandlerWindows
//   dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

//   // Initialize PacketHandler instance
//   // Set the protocol version
//   // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
//   dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

//   int index = 0;
//   int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//   int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

//   uint8_t dxl_error = 0;                          // Dynamixel error
//   int16_t dxl_present_position = 0;               // Present position

//   // Open port
//   if (portHandler->openPort())
//   {
//     Serial.print("Succeeded to open the port!\n");
//   }
//   else
//   {
//     Serial.print("Failed to open the port!\n");
//     Serial.print("Press any key to terminate...\n");
//     return;
//   }

//   // Set port baudrate
//   if (portHandler->setBaudRate(BAUDRATE))
//   {
//     Serial.print("Succeeded to change the baudrate!\n");
//   }
//   else
//   {
//     Serial.print("Failed to change the baudrate!\n");
//     Serial.print("Press any key to terminate...\n");
//     return;
//   }

//   // Enable Dynamixel Torque
//   dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     packetHandler->printTxRxResult(dxl_comm_result);
//   }
//   else if (dxl_error != 0)
//   {
//     packetHandler->printRxPacketError(dxl_error);
//   }
//   else
//   {
//     Serial.print("Dynamixel has been successfully connected \n");
//   }


//   while(1)
//   {
//     // Serial.print("Press any key to continue! (or press q to quit!)\n");


//     // while(Serial.available()==0);

//     // int ch;

//     // ch = Serial.read();
//     // if (ch == 'q')
//     //   break;

//     // Write goal position
//     dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index], &dxl_error);

//     do
//     {
//       // Read present position
//       dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
//       Serial.print("[ID:");      Serial.print(DXL_ID);
//       Serial.print(" GoalPos:"); Serial.print(dxl_goal_position[index]);
//       Serial.print(" PresPos:");  Serial.print(dxl_present_position);
//       Serial.println(" ");


//     }while((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

//     // Change goal position
//     if (index == 0)
//     {
//       index = 1;
//     }
//     else
//     {
//       index = 0;
//     }
//   }

//   // Close port
//   portHandler->closePort();

// }

// void loop() {
//   // put your main code here, to run repeatedly:

// }