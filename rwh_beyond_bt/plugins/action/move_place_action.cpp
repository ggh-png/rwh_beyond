// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/move_place_action.hpp"

#include <cmath>

namespace rwh_beyond_bt
{
MovePlaceAction::~MovePlaceAction() {stop();}

BT::PortsList MovePlaceAction::providedPorts()
{return motion_ports({"home_joint_positions", "x", "z"});}

double MovePlaceAction::minimum_z()
{return 0;}

void MovePlaceAction::perform()
{
  auto & ctx = *context_;
  ctx.release_ready = false;
  ctx.prepare_motion(cancel_);
  if (!ctx.attached) {
    throw ActionFailure(ErrorCode::NO_PAYLOAD, "bt", "NO_PAYLOAD", "MovePlace",
            "Place requires an attached target");
  }
  if (x_ < 0) {throw std::invalid_argument("Place requires nonnegative X offset");}
  auto start = ctx.state(cancel_);
  auto pose = ctx.fk(home_state(start), cancel_);
  pose.position.x += x_;
  pose.position.z += z_;
  auto path = ctx.cartesian(pose, start, cancel_);
  if (path->error_code.val != 1) {
    throw ActionFailure(
      path->error_code.val == 0 ? ErrorCode::INVALID_RESPONSE : path->error_code.val,
      "moveit", "MOVEIT_ERROR_" + std::to_string(path->error_code.val),
      "/compute_cartesian_path", path->error_code.message.empty() ?
      "Place Cartesian planning failed" : path->error_code.message);
  }
  if (!std::isfinite(path->fraction) || path->fraction < 0 || path->fraction > 1) {
    throw ActionFailure(ErrorCode::INVALID_RESPONSE, "moveit", "INVALID_RESPONSE",
            "/compute_cartesian_path", "Invalid Cartesian fraction");
  }
  if (path->fraction >= .999) {
    ctx.execute(path->solution, cancel_);
  } else {
    auto goal = ctx.ik(pose, start, cancel_);
    if (goal.joint_state.name.empty()) {throw std::runtime_error("Place target IK failed");}
    ctx.execute(ctx.plan(start, goal, Constraints(), cancel_), cancel_);
  }
  ctx.release_ready = true;
}
}  // namespace rwh_beyond_bt
