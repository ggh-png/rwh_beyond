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

#include "rwh_beyond_bt/plugins/action/set_gripper_action.hpp"

#include <memory>

#include <charconv>
#include <cmath>
#include <stdexcept>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"


namespace rwh_beyond_bt
{

SetGripperAction::SetGripperAction(
  const std::string & xml_tag_name,
  const std::string & action_name, const BT::NodeConfiguration & conf)
: DiagnosticActionNode(xml_tag_name, action_name, conf)
{}

BT::PortsList SetGripperAction::providedPorts()
{
    // The default BT integer parser accepts a numeric prefix (e.g. "50.5" as 50).
    // Require the entire value to be an integer, including when loading XML.
  BT::PortInfo opening_port(BT::PortDirection::INPUT, typeid(int), [](BT::StringView text) {
      int value = 0;
      const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
      if (parsed.ec != std::errc() || parsed.ptr != text.data() + text.size()) {
        throw std::invalid_argument("opening must contain a complete integer");
      }
      return BT::Any(value);
    });
  opening_port.setDescription(
      "Required total opening width in mm (0..100); percent optional via unit");
  return providedBasicPorts({
      {"opening", opening_port},
      BT::InputPort<std::string>("unit", "mm", "Opening unit: mm (default) or percent"),
      BT::InputPort<double>("duration", 3.0, "Trajectory duration in seconds (0, 3600]"),
    });
}

void SetGripperAction::on_tick()
{
  set_stage("set_gripper");
  try {
    const auto opening = getInput<int>("opening");
    const auto unit = getInput<std::string>("unit");
    const auto duration = getInput<double>("duration");
    if (!opening || !unit || !duration) {
      throw std::invalid_argument("opening, unit and duration must have valid values");
    }
    if (opening.value() < 0 || opening.value() > 100) {
      throw std::invalid_argument("opening must be an integer in [0, 100]");
    }
    if (unit.value() != "percent" && unit.value() != "mm") {
      throw std::invalid_argument("unit must be percent or mm");
    }
    if (!std::isfinite(duration.value()) || duration.value() <= 0.0 ||
      duration.value() > 3600.0)
    {
      throw std::invalid_argument("duration must be finite and in (0, 3600] seconds");
    }
      // This model uses total aperture in metres, with a maximum width of 0.1 m.
    const double aperture = unit.value() == "percent" ?
      0.1 * opening.value() / 100.0 : opening.value() / 1000.0;
    goal_.trajectory.joint_names = {"gripper"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {aperture};
    point.time_from_start = rclcpp::Duration::from_seconds(duration.value());
    goal_.trajectory.points = {point};
  } catch (const std::exception & error) {
    fail(-1000, "INVALID_INPUT", error.what());
    should_send_goal_ = false;
  }
}

}  // namespace rwh_beyond_bt

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rwh_beyond_bt::SetGripperAction>(name,
      "gripper_controller/follow_joint_trajectory", config);
    };
  factory.registerBuilder<rwh_beyond_bt::SetGripperAction>("SetGripper", builder);
}
