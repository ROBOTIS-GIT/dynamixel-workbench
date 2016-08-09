#include "dynamixel_workbench_tool/dxl_motor.h"

using namespace dxl_motor;

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

DxlMotor::DxlMotor(uint8_t id, uint16_t model_number, float protocol_version)
{
  id_ = id;
  protocol_version_ = protocol_version;
  getModelName(model_number);
  getPath();
  getModelItem();
}

DxlMotor::~DxlMotor()
{

}

bool DxlMotor::getPath()
{
  dynamixel_item_path_  = ros::package::getPath("dynamixel_workbench_tool") + "/dynamixel";

  if(dynamixel_item_path_.compare(dynamixel_item_path_.size()-1, 1, "/") != 0)
  {
    dynamixel_item_path_ += "/";
  }

  dynamixel_item_path_ = dynamixel_item_path_ + model_name_ + ".device";
}

bool DxlMotor::getModelName(uint16_t model_number)
{
  if (model_number == 1030)
  {
    model_name_ = "XM430-W210";
  }
  else if(model_number == 51200)
  {
    model_name_ = "H42-20-S300-R";
  }
}

bool DxlMotor::getModelItem()
{
  std::ifstream file(dynamixel_item_path_.c_str());
  if(file.is_open())
  {
    std::string session = "";
    std::string input_str;

    while(!file.eof())
    {
      std::getline(file, input_str);

      // remove commnet ( # )
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

      if (session == "control table")
      {
        std::vector<std::string> tokens = split(input_str, '|');
        if(tokens.size() != 8)
          continue;

        ControlTableItem *item = new ControlTableItem;
        item->item_name = tokens[1];
        item->address = std::atoi(tokens[0].c_str());
        item->data_length = std::atoi(tokens[2].c_str());

        if (item->item_name == "realtime_tick")
        {
          realtime_tick_item = item;
        }
        else if (item->item_name == "operating_mode")
        {
          operating_mode_item = item;
        }
        else if (item->item_name == "torque_enable")
        {
          torque_enable_item = item;
        }
        else if (item->item_name == "goal_position")
        {
          ROS_INFO("%d",item->address);
          goal_position_item = item;
        }
        else if (item->item_name == "goal_velocity")
        {
          goal_velocity_item = item;
        }
        else if (item->item_name == "present_position")
        {
          present_position_item = item;
        }
        else if (item->item_name == "profile_velocity")
        {
          profile_velocity_item = item;
        }
        else if (item->item_name == "present_velocity")
        {
          present_velocity_item = item;
        }
        else if (item->item_name == "present_input_voltage")
        {
          present_input_voltage_item = item;
        }
        else if (item->item_name == "present_temperature")
        {
          present_temperature_item = item;
        }
        else if (item->item_name == "moving")
        {
          is_moving_item = item;
        }
      }
    }
    file.close();
  }
  else
  {
    ROS_ERROR("Unable to open file : %s", dynamixel_item_path_.c_str());
  }
}
