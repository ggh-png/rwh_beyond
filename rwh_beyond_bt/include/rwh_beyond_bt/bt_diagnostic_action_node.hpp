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

#ifndef RWH_BEYOND_BT__BT_DIAGNOSTIC_ACTION_NODE_HPP_
#define RWH_BEYOND_BT__BT_DIAGNOSTIC_ACTION_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rwh_beyond_bt/diagnostics.hpp"

namespace rwh_beyond_bt
{

std::string moveit_error_name(int32_t code);
std::string moveit_error_description(int32_t code);

template<class ActionT>
class DiagnosticActionNode : public nav2_behavior_tree::BtActionNode<ActionT>
{
  using Base = nav2_behavior_tree::BtActionNode<ActionT>;

public:
  DiagnosticActionNode(
    const std::string & name, const std::string & action, const BT::NodeConfiguration & config);

  BT::NodeStatus tick() override;

  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback> feedback) override;

protected:
  void set_stage(const std::string & stage);

  void fail(int32_t code, const std::string & name, const std::string & message);

private:
  BT::NodeStatus finish(bool ros_succeeded);
};

extern template class DiagnosticActionNode<moveit_msgs::action::MoveGroup>;
extern template class DiagnosticActionNode<control_msgs::action::FollowJointTrajectory>;

}  // namespace rwh_beyond_bt

#endif  // RWH_BEYOND_BT__BT_DIAGNOSTIC_ACTION_NODE_HPP_
