/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Taehoon Lim (Darby) */

#include "dynamixel_workbench_tutorials/dynamixel_workbench_pan_tilt.h"

using namespace dynamixel_workbench_pan_tilt;

DynamixelWorkbenchPanTilt::DynamixelWorkbenchPanTilt()
    :nh_priv_("~"),
     is_debug_(false)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchPanTilt());

  // init ROS Client
  position_control_client_ = nh_.serviceClient<dynamixel_workbench_msgs::SetPosition>("/dynamixel_workbench_tutorials/pan_tilt");
}

DynamixelWorkbenchPanTilt::~DynamixelWorkbenchPanTilt()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchPanTilt());
}

bool DynamixelWorkbenchPanTilt::initDynamixelWorkbenchPanTilt(void)
{
  ROS_INFO("dynamixel_workbench_pan_tilt : Init OK!");
  return true;
}

bool DynamixelWorkbenchPanTilt::shutdownDynamixelWorkbenchPanTilt(void)
{
  ros::shutdown();
  return true;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_pan_tilt");

  DynamixelWorkbenchPanTilt dynamixel_pan_tilt;
  dynamixel_workbench_msgs::SetPosition srv;

  if (argc != 4)
  {
    ROS_ERROR("rosrun dynamixel_workbench_tutorials dynamixel_workbench_pan_tilt [mode] [pan_pos(radian)] [tilt_pos(radian)]");
    return 1;
  }

  srv.request.unit = argv[1];
  srv.request.pan_pos = atof(argv[2]);
  srv.request.tilt_pos = atof(argv[3]);

  if (dynamixel_pan_tilt.position_control_client_.call(srv))
  {
    ROS_INFO("send messages: [pan_pos: %.2f (value)] [tilt_pos: %.2f (value)]", srv.response.pan_pos, srv.response.tilt_pos);
  }
  else
  {
    ROS_ERROR("Failed to call service /pan_tilt or /multi_port");
  }

  return 0;
}
