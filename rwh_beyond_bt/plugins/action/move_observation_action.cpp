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
  return motion_ports({"home_joint_positions", "x", "y", "z", "tilt_deg", "yaw_deg",
             "settle_time"});
}

double MoveObservationAction::default_z()
{return context_->number("observation_z_offset", .2, -.3, .3);}

void MoveObservationAction::perform()
{
  auto & ctx = *context_;
  const auto mode = ctx.parameter<std::string>("observation_motion_mode", "ompl");
  if (mode != "ompl" && mode != "cartesian") {
    throw std::invalid_argument("observation_motion_mode must be ompl or cartesian");
  }
  ctx.prepare_motion(cancel_);
  auto start = ctx.state(cancel_);
  auto pose = ctx.fk(home_state(start), cancel_);
  auto transform = pose_matrix(pose);
  transform.translation() += Eigen::Vector3d(x_, y_, z_);
  transform.rotate(Eigen::AngleAxisd(tilt_[2] * M_PI / 180, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(tilt_[1] * M_PI / 180, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(tilt_[0] * M_PI / 180, Eigen::Vector3d::UnitX()));
  // Rotate only orientation about base Z; offsets and elevation are unchanged.
  transform.linear() = Eigen::AngleAxisd(yaw_ * M_PI / 180, Eigen::Vector3d::UnitZ()) *
    transform.linear();
  const auto target = matrix_pose(transform);
  RCLCPP_INFO(ctx.node->get_logger(),
    "MoveObservation mode=%s frame=%s target_xyz=[%.4f, %.4f, %.4f] "
    "quaternion=[%.4f, %.4f, %.4f, %.4f] offset=[%.3f, %.3f, %.3f] tilt=%.1f yaw=%.1f",
    mode.c_str(), ctx.base.c_str(), target.position.x, target.position.y, target.position.z,
    target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w,
    x_, y_, z_, tilt_[1], yaw_);
  if (mode == "cartesian") {
    // Keep the complete-path requirement; never execute partial Cartesian results.
    ctx.execute_cartesian(target, Constraints(), cancel_);
  } else {
    try {
      const auto goal = ctx.ik(target, start, cancel_, false, PlanningProfile::OBSERVATION);
      ctx.execute(ctx.plan(start, goal, Constraints(), cancel_, false,
          PlanningProfile::OBSERVATION),
        cancel_);
    } catch (const ActionFailure & error) {
      if (error.source == "/compute_ik") {
        throw ActionFailure(error.code, error.domain, error.name, error.source,
          std::string(error.what()) + "; observation target may be unreachable or in collision; "
          "check target_xyz/tilt/yaw and moveit_observation_ik_timeout (OMPL has not started)");
      }
      throw;
    }
  }
  auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(settle_);
  ctx.wait([&]() {return std::chrono::steady_clock::now() >= until;}, settle_ + 1, cancel_);
}
}  // namespace rwh_beyond_bt
