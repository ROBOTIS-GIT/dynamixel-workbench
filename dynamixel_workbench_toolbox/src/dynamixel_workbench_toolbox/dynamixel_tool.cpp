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

/* Authors: zerom, Taehoon Lim (Darby) */

#include "../../include/dynamixel_workbench_toolbox/dynamixel_tool.h"

using namespace dynamixel_tool;

static inline std::string &ltrim(std::string &s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
  return s;
}
static inline std::string &rtrim(std::string &s)
{
  s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
  return s;
}
static inline std::string &trim(std::string &s)
{
  return ltrim(rtrim(s));
}
static inline std::vector<std::string> split(const std::string &text, char sep)
{
  std::vector<std::string> tokens;
  std::size_t start = 0, end = 0;

  while((end = text.find(sep, start)) != (std::string::npos))
  {
    tokens.push_back(text.substr(start, end - start));
    trim(tokens.back());
    start = end + 1;
  }

  tokens.push_back(text.substr(start));
  trim(tokens.back());
  return tokens;
}

DynamixelTool::DynamixelTool(uint8_t id, uint16_t model_number)
    :id_(0),
     model_number_(0),
     model_name_(""),
     model_path_(""),
     name_path_("")
{
  id_           = id;
  model_number_ = model_number;

  getNameFilePath();
  // getModelFilePath();
}

DynamixelTool::DynamixelTool(uint8_t id, std::string model_name)
    :id_(0),
     model_number_(0),
     model_name_(""),
     model_path_(""),
     name_path_("")
{
  id_         = id;
  model_name_ = model_name;

  getModelFilePath();
}

DynamixelTool::~DynamixelTool(){}

bool DynamixelTool::getNameFilePath()
{
#if defined(__linux__)
  name_path_ = "../../../dynamixel/model_info.list";
  getModelName();

#elif defined(__OPENCR__) || defined(__OPENCM904__)
  char* model_info = 
  #include "../../dynamixel/model_info.list"
  ;
  getModelName(model_info);
#endif

  return true;
}

bool DynamixelTool::getModelFilePath()
{
  std::string dynamixel_series = "";

  dynamixel_series = model_name_.substr(0,3);

  if (dynamixel_series.find("_") != std::string::npos ||
      dynamixel_series.find("4") != std::string::npos)
    dynamixel_series.erase(2,3);

#if defined(__linux__)
  model_path_ = "../../../dynamixel/models/";
  model_path_ = model_path_ + dynamixel_series + "/" + model_name_ + ".device";

  getModelItem();

#elif defined(__OPENCR__) || defined(__OPENCM904__)
  char* device;
  if (dynamixel_series == "AX")
  {
    device = 
    #include "../../dynamixel/models/AX/AX.device"
    ;
  }
  else if (dynamixel_series == "RX")
  {
    device = 
    #include "../../dynamixel/models/RX/RX.device"
    ;
  }
  else if (dynamixel_series == "EX")
  {
    device = 
    #include "../../dynamixel/models/EX/EX.device"
    ;
  }
  else if (dynamixel_series == "MX")
  {
    if (model_name_ == "MX_12W" || model_name_ == "MX_28")
    {
      device = 
      #include "../../dynamixel/models/MX/MX_S.device"
      ;
    }
    else
    {
      device = 
      #include "../../dynamixel/models/MX/MX_L.device"
      ;
    }
  }
  else if (dynamixel_series == "XL")
  {
    if (model_name_ == "XL_320")
    {
      device = 
      #include "../../dynamixel/models/XL/XL_320.device"
      ;
    }
    else
    {
      device = 
      #include "../../dynamixel/models/XL/XL.device"
      ;     
    }
  }
  else if (dynamixel_series == "XM")
  {
    device = 
    #include "../../dynamixel/models/XM/XM.device"
    ;
  }
  else if (dynamixel_series == "XH")
  {
    device = 
    #include "../../dynamixel/models/XH/XH.device"
    ;
  }
  else if (dynamixel_series == "PRO")
  {
    if (model_name_ == "PRO_L42_10_S300_R")
    {
      device = 
      #include "../../dynamixel/models/PRO/PRO_L42_10_S300_R.device"
      ;
    }
    else
    {
      device = 
      #include "../../dynamixel/models/PRO/PRO.device"
      ;
    }
  }

  // getModelItem(device);
#endif

  return true;
}

bool DynamixelTool::getModelName()
{
  std::ifstream file(name_path_.c_str());

  if (file.is_open())
  {
    std::string input_str;

    while (!file.eof())
    {
      std::getline(file, input_str);

      // remove comment ( # )
      std::size_t pos = input_str.find("#");
      if (pos != std::string::npos)
      {
        input_str = input_str.substr(0,pos);
      }

      std::vector<std::string> tokens = split(input_str, '|');
      if (tokens.size() != 2)
        continue;

      if (model_number_ == std::atoi(tokens[0].c_str()))
      {
        model_name_ = tokens[1];
      }
    }
    file.close();    
    return true;
  }
  else
  {
    printf("Unable to read model_info.list file\n");
    exit(1);
    return false;
  }
}

bool DynamixelTool::getModelName(char* info)
{
  // std::istringstream iss;

  // std::string line;
  // while (std::getline(iss, line))
  // {
  //   // remove comment ( # )
  //   std::size_t pos = line.find("#");
  //   if (pos != std::string::npos)
  //   {
  //     line = line.substr(0,pos);
  //   }

  //   std::vector<std::string> tokens = split(line, '|');
  //   if (tokens.size() != 2)
  //     continue;

  //   if (model_number_ == std::atoi(tokens[0].c_str()))
  //   {
  //     model_name_ = tokens[1];
  //   }
  // }

  char* single_line;
  single_line = strtok(info, "\n");
  std::string input_str(single_line);

  while (single_line != NULL)
  {
    // remove comment ( # )
    std::size_t pos = input_str.find("#");
    if (pos != std::string::npos)
    {
      input_str = input_str.substr(0,pos);
    }
    Serial.println(input_str.c_str());

    std::vector<std::string> tokens = split(input_str, '|');
    if (tokens.size() != 2)
      continue;

    // if (model_number_ == std::atoi(tokens[0].c_str()))
    // {
    //   model_name_ = tokens[1];
    // }

    // Serial.println(model_name_.c_str());

    single_line = strtok(NULL, "\n");
  }

  return true;
}

bool DynamixelTool::getModelItem()
{
  std::ifstream file(model_path_.c_str());

  if (file.is_open())
  {
    std::string session = "";
    std::string input_str;

    while (!file.eof())
    {
      std::getline(file, input_str);

      // remove comment ( # )
      std::size_t pos = input_str.find("#");
      if (pos != std::string::npos)
      {
        input_str = input_str.substr(0,pos);
      }

      // trim
      input_str = trim(input_str);
      if (input_str == "")
        continue;

      // find session;
      if (!input_str.compare(0, 1, "[") && !input_str.compare(input_str.size()-1, 1, "]"))
      {
        input_str = input_str.substr(1, input_str.size()-2);
        std::transform(input_str.begin(), input_str.end(), input_str.begin(), ::tolower);
        session = trim(input_str);
        continue;
      }

      if (session == "type info")
      {
        std::vector<std::string> tokens = split(input_str, '=');
        if (tokens.size() != 2)
          continue;

        if (tokens[0] == "torque_to_current_value_ratio")
          torque_to_current_value_ratio_ = std::atof(tokens[1].c_str());
        else if (tokens[0] == "velocity_to_value_ratio")
          velocity_to_value_ratio_ = std::atof(tokens[1].c_str());
        else if (tokens[0] == "value_of_0_radian_position")
          value_of_0_radian_position_ = std::atoi(tokens[1].c_str());
        else if (tokens[0] == "value_of_min_radian_position")
          value_of_min_radian_position_ = std::atoi(tokens[1].c_str());
        else if (tokens[0] == "value_of_max_radian_position")
          value_of_max_radian_position_ = std::atoi(tokens[1].c_str());
        else if (tokens[0] == "min_radian")
          min_radian_ = std::atof(tokens[1].c_str());
        else if (tokens[0] == "max_radian")
          max_radian_ = std::atof(tokens[1].c_str());
      }
      else if (session == "baud rate")
      {
        std::vector<std::string> tokens = split(input_str, '|');
        if(tokens.size() != 2)
          continue;

        baud_rate_table_[std::atoi(tokens[0].c_str())] = std::atoi(tokens[1].c_str());
      }
      else if (session == "control table")
      {
        std::vector<std::string> tokens = split(input_str, '|');
        if(tokens.size() != 5)
          continue;

        ControlTableItem *item = new ControlTableItem;
        item->item_name = tokens[1];
        item->address = std::atoi(tokens[0].c_str());
        item->data_length = std::atoi(tokens[2].c_str());
        if(tokens[3] == "R")
            item->access_type = READ;
        else if(tokens[3] == "RW")
            item->access_type = READ_WRITE;
        if(tokens[4] == "EEPROM")
            item->memory_type = EEPROM;
        else if(tokens[4] == "RAM")
            item->memory_type = RAM;

        ctrl_table_[item->item_name] = item;
      }
    }
    file.close();
  }
  else
  {
    printf("Unable to read .device file\n");
    exit(1);
    return false;
  }

  return true;
}

// bool DynamixelTool::getModelItem(char* device)
// {
//   std::istringstream iss(device);

//   std::string line;
//   std::string session = "";

//   while (std::getline(iss, line))
//   {
//     // remove comment ( # )
//     std::size_t pos = line.find("#");
//     if (pos != std::string::npos)
//     {
//       line = line.substr(0,pos);
//     }

//     // trim
//     line = trim(line);
//     if (line == "")
//       continue;

//     // find session;
//     if (!line.compare(0, 1, "[") && !line.compare(line.size()-1, 1, "]"))
//     {
//       line = line.substr(1, line.size()-2);
//       std::transform(line.begin(), line.end(), line.begin(), ::tolower);
//       session = trim(line);
//       continue;
//     }

//     if (session == "type info")
//     {
//       std::vector<std::string> tokens = split(line, '=');
//       if (tokens.size() != 2)
//         continue;

//       if (tokens[0] == "torque_to_current_value_ratio")
//         torque_to_current_value_ratio_ = std::atof(tokens[1].c_str());
//       else if (tokens[0] == "velocity_to_value_ratio")
//         velocity_to_value_ratio_ = std::atof(tokens[1].c_str());
//       else if (tokens[0] == "value_of_0_radian_position")
//         value_of_0_radian_position_ = std::atoi(tokens[1].c_str());
//       else if (tokens[0] == "value_of_min_radian_position")
//         value_of_min_radian_position_ = std::atoi(tokens[1].c_str());
//       else if (tokens[0] == "value_of_max_radian_position")
//         value_of_max_radian_position_ = std::atoi(tokens[1].c_str());
//       else if (tokens[0] == "min_radian")
//         min_radian_ = std::atof(tokens[1].c_str());
//       else if (tokens[0] == "max_radian")
//         max_radian_ = std::atof(tokens[1].c_str());
//     }
//     // else if (session == "baud rate")
//     // {
//     //   std::vector<std::string> tokens = split(line, '|');
//     //   if(tokens.size() != 2)
//     //     continue;

//     //   baud_rate_table_[std::atoi(tokens[0].c_str())] = std::atoi(tokens[1].c_str());
//     // }
//     else if (session == "control table")
//     {
//       std::vector<std::string> tokens = split(line, '|');
//       if(tokens.size() != 5)
//         continue;

//       ControlTableItem *item = new ControlTableItem;
//       item->item_name = tokens[1];
//       item->address = std::atoi(tokens[0].c_str());
//       item->data_length = std::atoi(tokens[2].c_str());
//       // if(tokens[3] == "R")
//       //     item->access_type = READ;
//       // else if(tokens[3] == "RW")
//       //     item->access_type = READ_WRITE;
//       // if(tokens[4] == "EEPROM")
//       //     item->memory_type = EEPROM;
//       // else if(tokens[4] == "RAM")
//       //     item->memory_type = RAM;

//       ctrl_table_[item->item_name] = item;
//     }
//   }

//   return true;
// }

