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

#include "rwh_beyond_bt/bt_scenario_action_node.hpp"

#include <memory>
#include <algorithm>
#include <stdexcept>
#include <vector>

#include <chrono>
#include <cmath>
#include <exception>

#include "rwh_beyond_bt/diagnostics.hpp"


namespace rwh_beyond_bt
{

ScenarioActionNode::ScenarioActionNode(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf, const std::string & command)
: BtActionNode<Action>(xml_tag_name, action_name, conf), command_(command)
{cancel_timeout_ = std::chrono::milliseconds(2000);}

BT::PortsList ScenarioActionNode::providedPorts()
{
  return providedBasicPorts({});
}

BT::NodeStatus ScenarioActionNode::tick()
{
  try {
    const auto status = BtActionNode<Action>::tick();
    if (status == BT::NodeStatus::FAILURE && demo_feedback(config().blackboard).error_code == 0) {
      report_failure(config().blackboard, -1202, "GRASP_ACTION_REQUEST_FAILED", command_,
          "Grasp stage was rejected or its acknowledgement timed out");
    }
    return status;
  } catch (const std::exception & error) {
    report_failure(config().blackboard, -1203, "GRASP_BT_EXCEPTION", command_, error.what());
    return BT::NodeStatus::FAILURE;
  }
}

void ScenarioActionNode::on_tick()
{
  begin_stage(config().blackboard, command_, name());
  goal_ = Action::Goal();
  goal_.command = command_;
  auto offset = [this](const std::string & port, bool & overridden, double & value) {
      if (!config().input_ports.count(port)) {return;}
      auto input = getInput<double>(port);
      if (!input || !std::isfinite(input.value())) {
        throw std::invalid_argument(port + " must be finite");
      }
      overridden = true;
      value = input.value();
    };
  offset("x", goal_.override_observation_x, goal_.observation_x_offset);
  offset("y", goal_.override_observation_y, goal_.observation_y_offset);
  offset("z", goal_.override_observation_z, goal_.observation_z_offset);
  offset("tilt_deg", goal_.override_observation_tilt, goal_.observation_tilt_deg);
  if (config().input_ports.count("home_joint_positions")) {
    const auto home = getInput<std::vector<double>>("home_joint_positions");
    if (!home || home.value().size() != 6 ||
      !std::all_of(home.value().begin(), home.value().end(), [](double v) {
        return std::isfinite(v);
        }))
    {
      throw std::invalid_argument("home_joint_positions requires six finite radians");
    }
    goal_.home_joint_positions = home.value();
  }
  auto scalar = [this](const std::string & port, double minimum, double maximum, bool allow_zero) {
      const auto input = getInput<double>(port);
      if (!input || !std::isfinite(input.value()) || input.value() < minimum ||
        input.value() > maximum || (!allow_zero && input.value() == 0.0))
      {
        throw std::invalid_argument("Invalid " + port);
      }
      return input.value();
    };
  if (config().input_ports.count("settle_time")) {
    goal_.observation_settle_time = scalar("settle_time", 0.0, 30.0, true);
    goal_.override_settle_time = true;
  }
  if (config().input_ports.count("opening")) {
    goal_.gripper_open_width = scalar("opening", 0.0, 0.10, true);
    goal_.override_gripper_width = true;
  }
  if (config().input_ports.count("duration")) {
    goal_.gripper_open_duration = scalar("duration", 0.0, 30.0, false);
    goal_.override_gripper_duration = true;
  }
}

void ScenarioActionNode::on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback)
{
  if (feedback) {
    auto diagnostic = demo_feedback(config().blackboard);
    diagnostic.current_stage = feedback->stage;
    config().blackboard->rootBlackboard()->set("demo_feedback", diagnostic);
  }
}

BT::NodeStatus ScenarioActionNode::on_success()
{
  if (!result_.result || !result_.result->success) {return on_aborted();}
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ScenarioActionNode::on_aborted()
{
  report_failure(config().blackboard, -1201, "GRASP_STAGE_FAILED", "grasp/" + command_,
      result_.result ? result_.result->message : "Grasp action returned no result");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ScenarioActionNode::on_cancelled()
{return BT::NodeStatus::FAILURE;}

}  // namespace rwh_beyond_bt
