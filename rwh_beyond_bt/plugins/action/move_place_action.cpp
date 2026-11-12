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

#include "rwh_beyond_bt/plugins/action/move_place_action.hpp"

#include <memory>
#include <vector>

namespace rwh_beyond_bt
{

MovePlaceAction::MovePlaceAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ScenarioActionNode(xml_tag_name, action_name, conf, "move_place")
{}

BT::PortsList MovePlaceAction::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::vector<double>>("home_joint_positions",
        "Home reference joint angles; omit for YAML"),
      BT::InputPort<double>("x", "Home TCP local +X offset in metres [0, 0.30]"),
      BT::InputPort<double>("z", "Base +Z offset in metres [0, 0.30]")});
}

}  // namespace rwh_beyond_bt

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rwh_beyond_bt::MovePlaceAction>(name, "grasp/move_place", config);
    };
  factory.registerBuilder<rwh_beyond_bt::MovePlaceAction>("MovePlace", builder);
}
