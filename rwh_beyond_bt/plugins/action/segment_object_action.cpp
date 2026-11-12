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

#include "rwh_beyond_bt/plugins/action/segment_object_action.hpp"

#include <memory>

namespace rwh_beyond_bt
{

SegmentObjectAction::SegmentObjectAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: GraspActionNode(xml_tag_name, action_name, conf)
{
}

BT::PortsList SegmentObjectAction::providedPorts()
{
  return providedBasicPorts({BT::InputPort<std::string>("target_text", "", "Optional target description"),
             BT::OutputPort<std::string>("observation_id"),
             BT::OutputPort<ObjectMask>("mask")});
}

void SegmentObjectAction::on_tick()
{
  GraspActionNode::on_tick();
  goal_.target_text = getInput<std::string>("target_text").value();
  std::string requested_target;
  if (config().blackboard->rootBlackboard()->get("target_text", requested_target) &&
    !requested_target.empty())
  {
    goal_.target_text = requested_target;
  }
}

BT::NodeStatus SegmentObjectAction::on_success()
{
  const auto status = GraspActionNode::on_success();
  if (status != BT::NodeStatus::SUCCESS) {
    return status;
  }
  setOutput("observation_id", result_.result->observation_id);
  setOutput("mask", result_.result->object_mask);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rwh_beyond_bt

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rwh_beyond_bt::SegmentObjectAction>(name, "grasp/segment_object",
      config);
    };
  factory.registerBuilder<rwh_beyond_bt::SegmentObjectAction>("SegmentObject", builder);
}
