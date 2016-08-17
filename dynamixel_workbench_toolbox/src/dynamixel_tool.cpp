#include "dynamixel_workbench_toolbox/dynamixel_tool.h"

using namespace dynamixel_tool;

static inline std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}
static inline std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}
static inline std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}
static inline std::vector<std::string> split(const std::string &text, char sep) {
    std::vector<std::string> tokens;
    std::size_t start = 0, end = 0;
    while((end = text.find(sep, start)) != (std::string::npos)) {
        tokens.push_back(text.substr(start, end - start));
        trim(tokens.back());
        start = end + 1;
    }
    tokens.push_back(text.substr(start));
    trim(tokens.back());
    return tokens;
}

DynamixelTool::DynamixelTool(uint8_t id, uint16_t model_number, float protocol_version)
    :id_(0),
     model_number_(0),
     protocol_version_(0.0),
     model_name_(""),
     dynamixel_item_path_(""),
     dynamixel_name_path_("")
{
  id_ = id;
  protocol_version_ = protocol_version;

  getModelName(model_number);
  getModelItem();
}

DynamixelTool::DynamixelTool(uint8_t id, std::string model_name, float protocol_version)
    :id_(0),
     model_number_(0),
     protocol_version_(0.0),
     model_name_(""),
     dynamixel_item_path_(""),
     dynamixel_name_path_("")
{
  id_ = id;
  model_name_ = model_name;
  protocol_version_ = protocol_version;

  getModelItem();
}

DynamixelTool::~DynamixelTool(){}

bool DynamixelTool::getModelName(uint16_t model_number)
{
  dynamixel_name_path_  = ros::package::getPath("dynamixel_workbench_toolbox") + "/dynamixel/model_info.list";
  ROS_INFO("Get file path : %s", dynamixel_name_path_.c_str());

  std::ifstream file(dynamixel_name_path_.c_str());
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

      if (model_number == std::atoi(tokens[0].c_str()))
      {
        model_number_ = model_number;
        model_name_ = tokens[1];
      }
    }
    file.close();
  }
  else
  {
    ROS_ERROR("Unable to open file : %s", dynamixel_name_path_.c_str());
  }
}

bool DynamixelTool::getModelPath()
{
  std::string dxl_series = "";
  dxl_series = model_name_.substr(0,2);

  dynamixel_item_path_  = ros::package::getPath("dynamixel_workbench_toolbox");

  dynamixel_item_path_ = dynamixel_item_path_ + "/dynamixel" + "/" + dxl_series + "/" + model_name_ + ".device";
  ROS_INFO("Get file path : %s", dynamixel_item_path_.c_str());
}

bool DynamixelTool::getModelItem()
{
  getModelPath();

  std::ifstream file(dynamixel_item_path_.c_str());
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

      if (session == "baud rate")
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
    ROS_ERROR("Unable to open file : %s", dynamixel_item_path_.c_str());
  }
}
