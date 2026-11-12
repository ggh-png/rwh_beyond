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

#include "rwh_beyond_bt/plugins/action/release_object_action.hpp"

#include <memory>

namespace rwh_beyond_bt
{

ReleaseObjectAction::ReleaseObjectAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ScenarioActionNode(xml_tag_name, action_name, conf, "release_object")
{}

BT::PortsList ReleaseObjectAction::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<double>("opening", "Gripper width in metres [0, 0.10]; omit for YAML"),
      BT::InputPort<double>("duration", "Opening duration in seconds (0, 30]; omit for default")});
}

}  // namespace rwh_beyond_bt

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rwh_beyond_bt::ReleaseObjectAction>(
        name, "grasp/release_object", config);
    };
  factory.registerBuilder<rwh_beyond_bt::ReleaseObjectAction>("ReleaseObject", builder);
}
