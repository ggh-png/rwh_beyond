// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/move_to_pregrasp_action.hpp"

namespace rwh_beyond_bt
{
MoveToPregraspAction::~MoveToPregraspAction() {stop();}

BT::PortsList MoveToPregraspAction::providedPorts()
{return motion_ports({"plan"});}

void MoveToPregraspAction::prepare()
{prepare_plan();}

void MoveToPregraspAction::perform()
{
  phase(GraspPlan::Phase::SELECTED);
  context_->validate_capture(input_plan_->capture);
  context_->execute(input_plan_->trajectory, cancel_);
  input_plan_->phase = GraspPlan::Phase::PREGRASP;
}
}  // namespace rwh_beyond_bt
