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

#include "rwh_beyond_bt/plugins/action/build_object_cloud_action.hpp"

#include <memory>

namespace rwh_beyond_bt
{

BuildObjectCloudAction::BuildObjectCloudAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: GraspActionNode(xml_tag_name, action_name, conf)
{
}

BT::PortsList BuildObjectCloudAction::providedPorts()
{
  return providedBasicPorts({BT::InputPort<std::string>("observation_id"),
             BT::InputPort<ObjectMask>("mask"),
             BT::OutputPort<ObjectCloud>("cloud")});
}

void BuildObjectCloudAction::on_tick()
{
  GraspActionNode::on_tick();
  goal_.observation_id = required_input<std::string>("observation_id");
  goal_.object_mask = required_input<ObjectMask>("mask");
}

BT::NodeStatus BuildObjectCloudAction::on_success()
{
  const auto status = GraspActionNode::on_success();
  if (status != BT::NodeStatus::SUCCESS) {
    return status;
  }
  setOutput("cloud", result_.result->object_cloud);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rwh_beyond_bt

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rwh_beyond_bt::BuildObjectCloudAction>(name,
      "grasp/build_object_cloud", config);
    };
  factory.registerBuilder<rwh_beyond_bt::BuildObjectCloudAction>("BuildObjectCloud", builder);
}
