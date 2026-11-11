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

#include "rwh_beyond_bt/plugins/action/move_observation_action.hpp"

#include <memory>

#include <array>

#include "moveit_msgs/msg/joint_constraint.hpp"


namespace rwh_beyond_bt
{

MoveObservationAction::MoveObservationAction(
  const std::string & xml_tag_name,
  const std::string & action_name, const BT::NodeConfiguration & conf)
: MoveGroupNode(xml_tag_name, action_name, conf)
{}

void MoveObservationAction::on_tick()
{
  set_stage("move_observation");
  set_common_request();
  // Collision-free simulation observation pose from moma config/robots/piper.yaml.
  constexpr std::array<double, 6> positions{0.22229381216392086, 1.0279353924406198,
    -0.22003795712607355, -1.0347526435042003, 0.041778217520923366, -1.0109176820764199};
  moveit_msgs::msg::Constraints constraints;
  for (size_t i = 0; i < positions.size(); ++i) {
    moveit_msgs::msg::JointConstraint joint;
    joint.joint_name = "joint" + std::to_string(i + 1);
    joint.position = positions[i];
    joint.tolerance_above = 0.001;
    joint.tolerance_below = 0.001;
    joint.weight = 1.0;
    constraints.joint_constraints.push_back(joint);
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
      return std::make_unique<rwh_beyond_bt::MoveObservationAction>(name, "move_action", config);
    };
  factory.registerBuilder<rwh_beyond_bt::MoveObservationAction>("MoveObservation", builder);
}
