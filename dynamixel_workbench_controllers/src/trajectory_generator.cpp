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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "dynamixel_workbench_controllers/trajectory_generator.h"

MinimumJerk::MinimumJerk()
{
  coefficient_ = Eigen::VectorXd::Zero(6);
}

MinimumJerk::~MinimumJerk() {}

void MinimumJerk::calcCoefficient(WayPoint start,
                                  WayPoint goal,
                                  double move_time,
                                  double control_time)
{
  uint16_t step_time = uint16_t(floor(move_time / control_time) + 1.0);
  move_time = double(step_time - 1) * control_time;

  Eigen::Matrix3d A = Eigen::Matrix3d::Identity(3, 3);
  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();

  A << pow(move_time, 3), pow(move_time, 4), pow(move_time, 5),
      3 * pow(move_time, 2), 4 * pow(move_time, 3), 5 * pow(move_time, 4),
      6 * pow(move_time, 1), 12 * pow(move_time, 2), 20 * pow(move_time, 3);

  coefficient_(0) = start.position;
  coefficient_(1) = start.velocity;
  coefficient_(2) = 0.5 * start.acceleration;

  b << (goal.position - start.position - (start.velocity * move_time + 0.5 * start.acceleration * pow(move_time, 2))),
      (goal.velocity - start.velocity - (start.acceleration * move_time)),
      (goal.acceleration - start.acceleration);

  Eigen::ColPivHouseholderQR<Eigen::Matrix3d> dec(A);
  x = dec.solve(b);

  coefficient_(3) = x(0);
  coefficient_(4) = x(1);
  coefficient_(5) = x(2);
}

Eigen::VectorXd MinimumJerk::getCoefficient()
{
  return coefficient_;
}

//-------------------- Joint trajectory --------------------//

JointTrajectory::JointTrajectory(){}

JointTrajectory::~JointTrajectory() {}

void JointTrajectory::init(double move_time,
                           double control_time,
                           std::vector<WayPoint> start,
                           std::vector<WayPoint> goal)
{
  for (uint8_t index = 0; index < start.size(); index++)
  {
    trajectory_generator_.calcCoefficient(start.at(index),
                                    goal.at(index),
                                    move_time,
                                    control_time);

    coefficient_.col(index) = trajectory_generator_.getCoefficient();
  }
}

void JointTrajectory::setJointNum(uint8_t joint_num)
{
  joint_num_ = joint_num;
  coefficient_ = Eigen::MatrixXd::Identity(6, joint_num);
}

std::vector<WayPoint> JointTrajectory::getJointWayPoint(double tick)
{
  joint_way_point_.clear();
  for (uint8_t index = 0; index < joint_num_; index++)
  {
    WayPoint single_joint_way_point;
    single_joint_way_point.position = 0.0;
    single_joint_way_point.velocity = 0.0;
    single_joint_way_point.acceleration = 0.0;

    single_joint_way_point.position = coefficient_(0, index) +
             coefficient_(1, index) * pow(tick, 1) +
             coefficient_(2, index) * pow(tick, 2) +
             coefficient_(3, index) * pow(tick, 3) +
             coefficient_(4, index) * pow(tick, 4) +
             coefficient_(5, index) * pow(tick, 5);

    single_joint_way_point.velocity = coefficient_(1, index) +
             2 * coefficient_(2, index) * pow(tick, 1) +
             3 * coefficient_(3, index) * pow(tick, 2) +
             4 * coefficient_(4, index) * pow(tick, 3) +
             5 * coefficient_(5, index) * pow(tick, 4);

    single_joint_way_point.acceleration = 2 * coefficient_(2, index) +
             6 * coefficient_(3, index) * pow(tick, 1) +
             12 * coefficient_(4, index) * pow(tick, 2) +
             20 * coefficient_(5, index) * pow(tick, 3);

    joint_way_point_.push_back(single_joint_way_point);
  }

  return joint_way_point_;
}

Eigen::MatrixXd JointTrajectory::getCoefficient()
{
  return coefficient_;
}
