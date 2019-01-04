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

#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <math.h>
#include <vector>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>

typedef struct _Point
{
  double position;
  double velocity;
  double acceleration;
} WayPoint;

class MinimumJerk
{
private:
  Eigen::VectorXd coefficient_;

public:
  MinimumJerk();
  virtual ~MinimumJerk();

  void calcCoefficient(WayPoint start,
                       WayPoint goal,
                       double move_time,
                       double control_time);

  Eigen::VectorXd getCoefficient();
};

class JointTrajectory
{
private:
  MinimumJerk trajectory_generator_;

  uint8_t joint_num_;
  Eigen::MatrixXd coefficient_;
  std::vector<WayPoint> joint_way_point_;

public:
  JointTrajectory();
  virtual ~JointTrajectory();

  void setJointNum(uint8_t joint_num);
  void init(double move_time,
            double control_time,
            std::vector<WayPoint> start,
            std::vector<WayPoint> goal
            );

  std::vector<WayPoint> getJointWayPoint(double tick);

  Eigen::MatrixXd getCoefficient();
};

#endif  // TRAJECTORY_GENERATOR_H_
