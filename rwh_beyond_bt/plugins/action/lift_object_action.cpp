// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/lift_object_action.hpp"

namespace rwh_beyond_bt
{
LiftObjectAction::~LiftObjectAction() {stop();}

BT::PortsList LiftObjectAction::providedPorts()
{return motion_ports({"plan"});}

void LiftObjectAction::prepare()
{prepare_plan();}

void LiftObjectAction::perform()
{
  auto & ctx = *context_;
  phase(GraspPlan::Phase::ATTACHED);
  if (!ctx.simulation && !ctx.attached) {
    throw ActionFailure(ErrorCode::NO_PAYLOAD, "bt", "NO_PAYLOAD", registrationName(),
            "No payload attachment for lift");
  }
  auto pose = ctx.fk(ctx.state(cancel_), cancel_);
  pose.position.z += input_plan_->lift_distance;
  ctx.execute_cartesian(pose, input_plan_->wrist, cancel_);
  input_plan_->phase = GraspPlan::Phase::LIFTED;
}
}  // namespace rwh_beyond_bt
