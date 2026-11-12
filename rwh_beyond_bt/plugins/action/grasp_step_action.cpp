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

#include "rwh_beyond_bt/plugins/action/grasp_step_action.hpp"

#include <memory>

#include <chrono>
#include <cmath>
#include <exception>

#include "rwh_beyond_bt/diagnostics.hpp"


namespace rwh_beyond_bt
{

GraspStepAction::GraspStepAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{cancel_timeout_ = std::chrono::milliseconds(2000);}

BT::PortsList GraspStepAction::providedPorts()
{
  return providedBasicPorts({BT::InputPort<std::string>("command", "Grasp coordinator stage"),
             BT::InputPort<double>("rise_m", "Observation rise in base Z (m); set with tilt_deg"),
             BT::InputPort<double>("tilt_deg", "Observation home-local Y tilt (degrees); set with rise_m"),
             BT::OutputPort<TcpPose>("pregrasp", "Collision-validated pregrasp TCP")});
}

BT::NodeStatus GraspStepAction::tick()
{
  try {
    const auto status = BtActionNode<Action>::tick();
    if (status == BT::NodeStatus::FAILURE && demo_feedback(config().blackboard).error_code == 0) {
      report_failure(config().blackboard, -1202, "GRASP_ACTION_REQUEST_FAILED", "grasp_step",
          "Grasp stage was rejected or its acknowledgement timed out");
    }
    return status;
  } catch (const std::exception & error) {
    report_failure(config().blackboard, -1203, "GRASP_BT_EXCEPTION", "grasp_step", error.what());
    return BT::NodeStatus::FAILURE;
  }
}

void GraspStepAction::on_tick()
{
  auto command = getInput<std::string>("command");
  begin_stage(config().blackboard, command ? command.value() : "grasp_step", name());
  if (!command || command.value().empty()) {
    report_failure(config().blackboard, -1200, "INVALID_GRASP_STAGE", "grasp_step",
        "command is required");
    should_send_goal_ = false;
    return;
  }
  goal_.command = command.value();
  goal_.override_observation = false;
  goal_.observation_z_offset = 0.0;
  goal_.observation_tilt_deg = 0.0;
  if (goal_.command == "move_observation" &&
    (config().input_ports.count("rise_m") || config().input_ports.count("tilt_deg")))
  {
    const auto rise = getInput<double>("rise_m");
    const auto tilt = getInput<double>("tilt_deg");
    if (!rise || !tilt || !std::isfinite(rise.value()) || rise.value() <= 0.0 ||
      !std::isfinite(tilt.value()))
    {
      report_failure(config().blackboard, -1200, "INVALID_OBSERVATION_POSE", "grasp_step",
          "move_observation requires finite rise_m > 0 and tilt_deg together");
      should_send_goal_ = false;
      return;
    }
    goal_.override_observation = true;
    goal_.observation_z_offset = rise.value();
    goal_.observation_tilt_deg = tilt.value();
  }
}

void GraspStepAction::on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback)
{
  if (feedback) {
    auto diagnostic = demo_feedback(config().blackboard);
    diagnostic.current_stage = feedback->stage;
    config().blackboard->rootBlackboard()->set("demo_feedback", diagnostic);
  }
}

BT::NodeStatus GraspStepAction::on_success()
{
  if (!result_.result || !result_.result->success) {return on_aborted();}
  if (goal_.command == "prepare") {setOutput("pregrasp", result_.result->pregrasp);}
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GraspStepAction::on_aborted()
{
  report_failure(config().blackboard, -1201, "GRASP_STAGE_FAILED", "grasp_step/" + goal_.command,
      result_.result ? result_.result->message : "Grasp action returned no result");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GraspStepAction::on_cancelled()
{return BT::NodeStatus::FAILURE;}

}  // namespace rwh_beyond_bt

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rwh_beyond_bt::GraspStepAction>(name, "grasp_step", config);
    };
  factory.registerBuilder<rwh_beyond_bt::GraspStepAction>("GraspStep", builder);
}
