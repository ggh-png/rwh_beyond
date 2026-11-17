// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/check_grasp_ready_action.hpp"

namespace rwh_beyond_bt
{
CheckGraspReadyAction::~CheckGraspReadyAction() {stop();}

BT::PortsList CheckGraspReadyAction::providedPorts()
{return motion_ports({"release_previous"});}

void CheckGraspReadyAction::prepare()
{repeat_ = required<bool>("release_previous");}

void CheckGraspReadyAction::perform()
{context_->check_ready(repeat_, cancel_);}
}  // namespace rwh_beyond_bt
