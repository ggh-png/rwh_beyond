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

#include "rwh_beyond_bt/plugins/action/close_gripper_action.hpp"

#include <memory>

#include "control_msgs/msg/joint_tolerance.hpp"


namespace rwh_beyond_bt
{

CloseGripperAction::CloseGripperAction(
  const std::string & xml_tag_name,
  const std::string & action_name, const BT::NodeConfiguration & conf)
: GripperCommand(xml_tag_name, action_name, conf, 0.0, "close_gripper")
{}

void CloseGripperAction::on_tick()
{
  GripperCommand::on_tick();
    // A grasp is expected to block before aperture zero. Preserve arm tolerances;
    // only gripper path-position/velocity tracking is disabled during this close.
  control_msgs::msg::JointTolerance tolerance;
  tolerance.name = "gripper";
  tolerance.position = -1.0;
  tolerance.velocity = -1.0;
  goal_.path_tolerance = {tolerance};
}

}  // namespace rwh_beyond_bt

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rwh_beyond_bt::CloseGripperAction>(name,
      "gripper_controller/follow_joint_trajectory", config);
    };
  factory.registerBuilder<rwh_beyond_bt::CloseGripperAction>("CloseGripper", builder);
}
