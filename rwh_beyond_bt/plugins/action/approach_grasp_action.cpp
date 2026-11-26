// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/approach_grasp_action.hpp"

namespace rwh_beyond_bt
{
ApproachGraspAction::~ApproachGraspAction() {stop();}

BT::PortsList ApproachGraspAction::providedPorts()
{return motion_ports({"plan"});}

void ApproachGraspAction::prepare()
{prepare_plan();}

void ApproachGraspAction::perform()
{
  phase(GraspPlan::Phase::PREGRASP);
  if (input_plan_->has_approach) {
    context_->execute_cartesian(input_plan_->grasp, input_plan_->wrist, cancel_);
  }
  input_plan_->phase = GraspPlan::Phase::APPROACHED;
}
}  // namespace rwh_beyond_bt
