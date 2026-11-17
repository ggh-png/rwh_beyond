// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/attach_object_action.hpp"

namespace rwh_beyond_bt
{
AttachObjectAction::~AttachObjectAction() {stop();}

BT::PortsList AttachObjectAction::providedPorts()
{return motion_ports({"plan"});}

void AttachObjectAction::prepare()
{prepare_plan();}

void AttachObjectAction::perform()
{
  auto & ctx = *context_;
  phase(GraspPlan::Phase::APPROACHED);
  // An interrupted close may still hold an object; retain attachment state for recovery.
  ctx.attached = true;
  ctx.gripper(input_plan_->close_width, ctx.number("gripper_close_duration", 2, .001, 30),
    true, cancel_);
  moveit_msgs::msg::PlanningScene update;
  moveit_msgs::msg::AttachedCollisionObject attachment;
  attachment.link_name = ctx.tcp;
  attachment.object = ctx.object;
  attachment.touch_links = {"gripper_link1", "gripper_link2", ctx.tcp};
  update.robot_state.attached_collision_objects = {attachment};
  ctx.apply(update, cancel_);
  ctx.release_ready = false;
  input_plan_->phase = GraspPlan::Phase::ATTACHED;
}
}  // namespace rwh_beyond_bt
