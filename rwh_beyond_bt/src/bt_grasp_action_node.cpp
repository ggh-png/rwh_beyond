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

#include "rwh_beyond_bt/bt_grasp_action_node.hpp"

#include <chrono>
#include <exception>

#include "rwh_beyond_bt/diagnostics.hpp"

namespace rwh_beyond_bt
{

GraspActionNode::GraspActionNode(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
  cancel_timeout_ = std::chrono::milliseconds(2000);
}

BT::NodeStatus GraspActionNode::tick()
{
  try {
    auto status = BtActionNode<Action>::tick();
    if (status == BT::NodeStatus::FAILURE && demo_feedback(config().blackboard).error_code == 0) {
      report_failure(config().blackboard, -1202, "GRASP_ACTION_REQUEST_FAILED", action_name_,
        "Stage rejected or acknowledgement timed out");
    }
    return status;
  } catch (const std::exception & error) {
    report_failure(config().blackboard, -1203, "GRASP_BT_EXCEPTION", action_name_, error.what());
    return BT::NodeStatus::FAILURE;
  }
}

void GraspActionNode::on_tick()
{
  begin_stage(config().blackboard, action_name_, name());
  goal_ = Action::Goal();
}

void GraspActionNode::on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback)
{
  if (feedback) {
    auto diagnostic = demo_feedback(config().blackboard);
    diagnostic.current_stage = feedback->stage;
    config().blackboard->rootBlackboard()->set("demo_feedback", diagnostic);
  }
}

BT::NodeStatus GraspActionNode::on_success()
{
  if (!result_.result || !result_.result->success) {
    return on_aborted();
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GraspActionNode::on_aborted()
{
  report_failure(config().blackboard, -1201, "GRASP_STAGE_FAILED", action_name_,
    result_.result ? result_.result->message : "Stage returned no result");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GraspActionNode::on_cancelled()
{
  return BT::NodeStatus::FAILURE;
}

}  // namespace rwh_beyond_bt
