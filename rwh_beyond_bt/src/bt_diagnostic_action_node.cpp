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

#include "rwh_beyond_bt/bt_diagnostic_action_node.hpp"

#include <exception>
#include <type_traits>


namespace rwh_beyond_bt
{

std::string moveit_error_name(int32_t code)
{
  using Codes = moveit_msgs::msg::MoveItErrorCodes;
  switch (code) {
    case Codes::SUCCESS: return "SUCCESS";
    case Codes::UNDEFINED: return "UNDEFINED";
    case Codes::FAILURE: return "FAILURE";
    case Codes::PLANNING_FAILED: return "PLANNING_FAILED";
    case Codes::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case Codes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: return
        "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case Codes::CONTROL_FAILED: return "CONTROL_FAILED";
    case Codes::UNABLE_TO_AQUIRE_SENSOR_DATA: return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case Codes::TIMED_OUT: return "TIMED_OUT";
    case Codes::PREEMPTED: return "PREEMPTED";
    case Codes::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case Codes::START_STATE_VIOLATES_PATH_CONSTRAINTS: return
        "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case Codes::START_STATE_INVALID: return "START_STATE_INVALID";
    case Codes::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case Codes::GOAL_VIOLATES_PATH_CONSTRAINTS: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case Codes::GOAL_CONSTRAINTS_VIOLATED: return "GOAL_CONSTRAINTS_VIOLATED";
    case Codes::GOAL_STATE_INVALID: return "GOAL_STATE_INVALID";
    case Codes::UNRECOGNIZED_GOAL_TYPE: return "UNRECOGNIZED_GOAL_TYPE";
    case Codes::INVALID_GROUP_NAME: return "INVALID_GROUP_NAME";
    case Codes::INVALID_GOAL_CONSTRAINTS: return "INVALID_GOAL_CONSTRAINTS";
    case Codes::INVALID_ROBOT_STATE: return "INVALID_ROBOT_STATE";
    case Codes::INVALID_LINK_NAME: return "INVALID_LINK_NAME";
    case Codes::INVALID_OBJECT_NAME: return "INVALID_OBJECT_NAME";
    case Codes::FRAME_TRANSFORM_FAILURE: return "FRAME_TRANSFORM_FAILURE";
    case Codes::COLLISION_CHECKING_UNAVAILABLE: return "COLLISION_CHECKING_UNAVAILABLE";
    case Codes::ROBOT_STATE_STALE: return "ROBOT_STATE_STALE";
    case Codes::SENSOR_INFO_STALE: return "SENSOR_INFO_STALE";
    case Codes::COMMUNICATION_FAILURE: return "COMMUNICATION_FAILURE";
    case Codes::CRASH: return "CRASH";
    case Codes::ABORT: return "ABORT";
    case Codes::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    default: return "UNKNOWN_MOVEIT_ERROR";
  }
}

std::string moveit_error_description(int32_t code)
{
  using Codes = moveit_msgs::msg::MoveItErrorCodes;
  switch (code) {
    case Codes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "Planning scene changed during execution; the remaining trajectory became invalid. "
             "Possible collision with updated environment geometry; exact contact pairs are not "
             "provided by the MoveGroup action result. Check MoveIt collision logs.";
    case Codes::START_STATE_IN_COLLISION: return "The start state is in collision.";
    case Codes::GOAL_IN_COLLISION: return "The goal state is in collision.";
    case Codes::PLANNING_FAILED: return "MoveIt could not find a motion plan.";
    case Codes::NO_IK_SOLUTION: return "No inverse-kinematics solution was found.";
    case Codes::CONTROL_FAILED: return "Trajectory execution failed in the controller.";
    case Codes::TIMED_OUT: return "MoveIt timed out.";
    case Codes::PREEMPTED: return "MoveIt execution was canceled or preempted.";
    default: return "MoveIt returned " + moveit_error_name(code) + ".";
  }
}


template<class ActionT>
DiagnosticActionNode<ActionT>::DiagnosticActionNode(
  const std::string & name, const std::string & action, const BT::NodeConfiguration & config)
: Base(name, action, config)
{}

template<class ActionT>
BT::NodeStatus DiagnosticActionNode<ActionT>::tick()
{
  if (!BT::isStatusActive(this->status())) {
    begin_stage(this->config().blackboard, this->name(), this->name());
  }
  try {
    const auto status = Base::tick();
    if (status == BT::NodeStatus::FAILURE &&
      demo_feedback(this->config().blackboard).error_code == 0)
    {
      fail(-1001, "ACTION_REQUEST_FAILED",
          "Action goal was rejected, could not be sent, or its acknowledgement timed out: " +
          this->action_name_);
    }
    return status;
  } catch (const std::exception & error) {
    fail(-1003, "BT_EXCEPTION", error.what());
    return BT::NodeStatus::FAILURE;
  }
}

template<class ActionT>
BT::NodeStatus DiagnosticActionNode<ActionT>::on_success()
{return finish(true);}

template<class ActionT>
BT::NodeStatus DiagnosticActionNode<ActionT>::on_aborted()
{return finish(false);}

template<class ActionT>
BT::NodeStatus DiagnosticActionNode<ActionT>::on_cancelled()
{
  if (demo_feedback(this->config().blackboard).error_code == 0) {
    if (this->result_.result) {return finish(false);}
    fail(-1004, "ACTION_CANCELED", "Action was canceled before a result was available.");
  }
  return BT::NodeStatus::FAILURE;
}

template<class ActionT>
void DiagnosticActionNode<ActionT>::on_wait_for_result(
  std::shared_ptr<const typename ActionT::Feedback> feedback)
{
  if constexpr (std::is_same_v<ActionT, moveit_msgs::action::MoveGroup>) {
    if (feedback) {
      auto diagnostic = demo_feedback(this->config().blackboard);
      diagnostic.moveit_state = feedback->state;
      this->config().blackboard->rootBlackboard()->set("demo_feedback", diagnostic);
    }
  }
}

template<class ActionT>
void DiagnosticActionNode<ActionT>::set_stage(const std::string & stage)
{
  begin_stage(this->config().blackboard, stage, this->name());
}

template<class ActionT>
void DiagnosticActionNode<ActionT>::fail(
  int32_t code, const std::string & name,
  const std::string & message)
{
  report_failure(this->config().blackboard, code, name, this->action_name_, message);
  RCLCPP_ERROR(this->node_->get_logger(), "%s: %s (%d): %s",
      this->name().c_str(), name.c_str(), code, message.c_str());
}

template<class ActionT>
BT::NodeStatus DiagnosticActionNode<ActionT>::finish(bool ros_succeeded)
{
  if (!this->result_.result) {
    fail(-1005, "MISSING_ACTION_RESULT", "Action completed without a result payload.");
    return BT::NodeStatus::FAILURE;
  }
  on_wait_for_result(this->feedback_);
  const auto & result = *this->result_.result;
  int32_t code;
  std::string name, source = this->action_name_, message;
  bool application_succeeded;
  if constexpr (std::is_same_v<ActionT, moveit_msgs::action::MoveGroup>) {
    code = result.error_code.val;
    application_succeeded = code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    name = moveit_error_name(code);
    if (!result.error_code.source.empty()) {source = result.error_code.source;}
    message = result.error_code.message.empty() ?
      moveit_error_description(code) : result.error_code.message;
  } else {
    code = result.error_code;
    application_succeeded = code == ActionT::Result::SUCCESSFUL;
    switch (code) {
      case ActionT::Result::SUCCESSFUL: name = "SUCCESSFUL"; break;
      case ActionT::Result::INVALID_GOAL: name = "INVALID_GOAL"; break;
      case ActionT::Result::INVALID_JOINTS: name = "INVALID_JOINTS"; break;
      case ActionT::Result::OLD_HEADER_TIMESTAMP: name = "OLD_HEADER_TIMESTAMP"; break;
      case ActionT::Result::PATH_TOLERANCE_VIOLATED: name = "PATH_TOLERANCE_VIOLATED"; break;
      case ActionT::Result::GOAL_TOLERANCE_VIOLATED: name = "GOAL_TOLERANCE_VIOLATED"; break;
      default: name = "UNKNOWN_CONTROLLER_ERROR";
    }
    message = result.error_string.empty() ? "Controller returned " + name +
      "." : result.error_string;
  }
  if (ros_succeeded && application_succeeded) {return BT::NodeStatus::SUCCESS;}
  if (application_succeeded || code == 0) {
    message = "ROS action did not succeed (or returned an undefined error); native code=" +
      std::to_string(code) + ". " + message;
    code = -1006;
    name = "ACTION_FAILED";
  }
  report_failure(this->config().blackboard, code, name, source, message);
  RCLCPP_ERROR(this->node_->get_logger(), "%s: %s (%d), source=%s: %s",
      this->name().c_str(), name.c_str(), code, source.c_str(), message.c_str());
  return BT::NodeStatus::FAILURE;
}

template class DiagnosticActionNode<moveit_msgs::action::MoveGroup>;
template class DiagnosticActionNode<control_msgs::action::FollowJointTrajectory>;

}  // namespace rwh_beyond_bt
