#pragma once

#include <type_traits>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rwh_beyond_bt/diagnostics.hpp"

namespace rwh_beyond_bt
{
inline std::string moveItErrorName(int32_t code)
{
  using Codes = moveit_msgs::msg::MoveItErrorCodes;
  switch (code) {
    case Codes::SUCCESS: return "SUCCESS";
    case Codes::UNDEFINED: return "UNDEFINED";
    case Codes::FAILURE: return "FAILURE";
    case Codes::PLANNING_FAILED: return "PLANNING_FAILED";
    case Codes::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case Codes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case Codes::CONTROL_FAILED: return "CONTROL_FAILED";
    case Codes::UNABLE_TO_AQUIRE_SENSOR_DATA: return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case Codes::TIMED_OUT: return "TIMED_OUT";
    case Codes::PREEMPTED: return "PREEMPTED";
    case Codes::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case Codes::START_STATE_VIOLATES_PATH_CONSTRAINTS: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
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

inline std::string moveItErrorDescription(int32_t code)
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
    default: return "MoveIt returned " + moveItErrorName(code) + ".";
  }
}

template<class ActionT>
class DiagnosticActionNode : public nav2_behavior_tree::BtActionNode<ActionT>
{
  using Base = nav2_behavior_tree::BtActionNode<ActionT>;
public:
  DiagnosticActionNode(
    const std::string & name, const std::string & action, const BT::NodeConfiguration & config)
  : Base(name, action, config) {}

  BT::NodeStatus tick() override
  {
    if (!BT::isStatusActive(this->status())) {
      beginStage(this->config().blackboard, this->name(), this->name());
    }
    try {
      const auto status = Base::tick();
      if (status == BT::NodeStatus::FAILURE &&
        demoFeedback(this->config().blackboard).error_code == 0)
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

  BT::NodeStatus on_success() override {return finish(true);}
  BT::NodeStatus on_aborted() override {return finish(false);}
  BT::NodeStatus on_cancelled() override
  {
    if (demoFeedback(this->config().blackboard).error_code == 0) {
      if (this->result_.result) {return finish(false);}
      fail(-1004, "ACTION_CANCELED", "Action was canceled before a result was available.");
    }
    return BT::NodeStatus::FAILURE;
  }

  void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback> feedback) override
  {
    if constexpr (std::is_same_v<ActionT, moveit_msgs::action::MoveGroup>) {
      if (feedback) {
        auto diagnostic = demoFeedback(this->config().blackboard);
        diagnostic.moveit_state = feedback->state;
        this->config().blackboard->rootBlackboard()->set("demo_feedback", diagnostic);
      }
    }
  }

protected:
  void setStage(const std::string & stage)
  {
    beginStage(this->config().blackboard, stage, this->name());
  }

  void fail(int32_t code, const std::string & name, const std::string & message)
  {
    reportFailure(this->config().blackboard, code, name, this->action_name_, message);
    RCLCPP_ERROR(this->node_->get_logger(), "%s: %s (%d): %s",
      this->name().c_str(), name.c_str(), code, message.c_str());
  }

private:
  BT::NodeStatus finish(bool ros_succeeded)
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
      name = moveItErrorName(code);
      if (!result.error_code.source.empty()) {source = result.error_code.source;}
      message = result.error_code.message.empty() ?
        moveItErrorDescription(code) : result.error_code.message;
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
      message = result.error_string.empty() ? "Controller returned " + name + "." : result.error_string;
    }
    if (ros_succeeded && application_succeeded) {return BT::NodeStatus::SUCCESS;}
    if (application_succeeded || code == 0) {
      message = "ROS action did not succeed (or returned an undefined error); native code=" +
        std::to_string(code) + ". " + message;
      code = -1006;
      name = "ACTION_FAILED";
    }
    reportFailure(this->config().blackboard, code, name, source, message);
    RCLCPP_ERROR(this->node_->get_logger(), "%s: %s (%d), source=%s: %s",
      this->name().c_str(), name.c_str(), code, source.c_str(), message.c_str());
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace rwh_beyond_bt
