// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/move_observation_action.hpp"

#include <cmath>

namespace rwh_beyond_bt
{
MoveObservationAction::~MoveObservationAction() {stop();}

BT::PortsList MoveObservationAction::providedPorts()
{
  return motion_ports({"home_joint_positions", "x", "y", "z", "tilt_deg", "settle_time"});
}

double MoveObservationAction::default_z()
{return context_->number("observation_z_offset", .2, -.3, .3);}

void MoveObservationAction::perform()
{
  auto & ctx = *context_;
  ctx.prepare_motion(cancel_);
  auto pose = ctx.fk(home_state(ctx.state(cancel_)), cancel_);
  auto transform = pose_matrix(pose);
  transform.translation() += Eigen::Vector3d(x_, y_, z_);
  transform.rotate(Eigen::AngleAxisd(tilt_[2] * M_PI / 180, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(tilt_[1] * M_PI / 180, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(tilt_[0] * M_PI / 180, Eigen::Vector3d::UnitX()));
  ctx.execute_cartesian(matrix_pose(transform), Constraints(), cancel_);
  auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(settle_);
  ctx.wait([&]() {return std::chrono::steady_clock::now() >= until;}, settle_ + 1, cancel_);
}
}  // namespace rwh_beyond_bt
