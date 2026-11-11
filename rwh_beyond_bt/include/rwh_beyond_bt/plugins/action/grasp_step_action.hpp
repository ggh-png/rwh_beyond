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

#ifndef RWH_BEYOND_BT__PLUGINS__ACTION__GRASP_STEP_ACTION_HPP_
#define RWH_BEYOND_BT__PLUGINS__ACTION__GRASP_STEP_ACTION_HPP_

#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rwh_beyond_bt/tcp_pose.hpp"
#include "rwh_beyond_msgs/action/run_grasp_step.hpp"

namespace rwh_beyond_bt
{

/** @brief GraspStep behavior tree node. */
class GraspStepAction
  : public nav2_behavior_tree::BtActionNode<rwh_beyond_msgs::action::RunGraspStep>
{
  using Action = rwh_beyond_msgs::action::RunGraspStep;

public:
  GraspStepAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  void on_tick() override;

  void on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback) override;

  BT::NodeStatus on_success() override;

  BT::NodeStatus on_aborted() override;

  BT::NodeStatus on_cancelled() override;
};

}  // namespace rwh_beyond_bt

#endif  // RWH_BEYOND_BT__PLUGINS__ACTION__GRASP_STEP_ACTION_HPP_
