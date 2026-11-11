// Copyright 2026 rwh_beyond contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rwh_beyond_bt/plugins/action/move_home_action.hpp"

#include <memory>

#include "moveit_msgs/msg/joint_constraint.hpp"


namespace rwh_beyond_bt
{

MoveHomeAction::MoveHomeAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: MoveGroupNode(xml_tag_name, action_name, conf)
{}

void MoveHomeAction::on_tick()
{
  set_stage("move_home");
  set_common_request();
  moveit_msgs::msg::Constraints constraints;
  for (const auto * joint : {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}) {
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = joint;
    constraint.position = 0.0;
    constraint.tolerance_above = 0.001;
    constraint.tolerance_below = 0.001;
    constraint.weight = 1.0;
    constraints.joint_constraints.push_back(constraint);
  }
  goal_.request.goal_constraints = {constraints};
}

}  // namespace rwh_beyond_bt

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rwh_beyond_bt::MoveHomeAction>(name, "move_action", config);
    };
  factory.registerBuilder<rwh_beyond_bt::MoveHomeAction>("MoveHome", builder);
}
