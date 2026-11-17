// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/move_home_action.hpp"

namespace rwh_beyond_bt
{
MoveHomeAction::~MoveHomeAction() {stop();}

BT::PortsList MoveHomeAction::providedPorts()
{return motion_ports({"home_joint_positions"});}

void MoveHomeAction::perform()
{
  auto & ctx = *context_;
  ctx.prepare_motion(cancel_);
  if (at_home()) {return;}
  auto start = ctx.state(cancel_);
  auto goal = home_state(start);
  ctx.execute(ctx.plan(start, goal, Constraints(), cancel_), cancel_);
  ctx.wait([&]() {return at_home();}, 20, cancel_);
}
}  // namespace rwh_beyond_bt
