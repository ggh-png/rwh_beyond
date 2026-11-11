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

#include "rwh_beyond_bt/plugins/action/make_tcp_pose_action.hpp"

#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rwh_beyond_bt/diagnostics.hpp"


namespace rwh_beyond_bt
{

MakeTcpPoseAction::MakeTcpPoseAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{}

BT::PortsList MakeTcpPoseAction::providedPorts()
{
  return {
    BT::InputPort<double>("x", "Required x in metres"),
    BT::InputPort<double>("y", "Required y in metres"),
    BT::InputPort<double>("z", "Required z in metres"),
    BT::InputPort<double>("qx", "Optional quaternion x; supply all four components"),
    BT::InputPort<double>("qy", "Optional quaternion y; supply all four components"),
    BT::InputPort<double>("qz", "Optional quaternion z; supply all four components"),
    BT::InputPort<double>("qw", "Optional quaternion w; supply all four components"),
    BT::InputPort<std::string>("frame_id", "base_link",
        "Reference frame for position and orientation"),
    BT::InputPort<bool>("relative", false, "Interpret only position as an offset along frame axes"),
    BT::OutputPort<TcpPose>("pose", "Constructed and validated TcpPose object"),
  };
}

BT::NodeStatus MakeTcpPoseAction::tick()
{
  begin_stage(config().blackboard, "make_tcp_pose", name());
  try {
    if (config().output_ports.count("pose") == 0) {
      throw std::invalid_argument("pose output must be mapped to a blackboard key");
    }
    const auto x = getInput<double>("x");
    const auto y = getInput<double>("y");
    const auto z = getInput<double>("z");
    const auto frame = getInput<std::string>("frame_id");
    const auto relative = getInput<bool>("relative");
    if (!x || !y || !z || !frame || !relative) {
      throw std::invalid_argument("x, y, z, frame_id and relative must have valid values");
    }
    TcpPose target;
    target.frame_id = frame.value();
    target.pose.position.x = x.value();
    target.pose.position.y = y.value();
    target.pose.position.z = z.value();
    target.relative = relative.value();
    size_t quaternion_inputs = 0;
    for (const auto * port : {"qx", "qy", "qz", "qw"}) {
      quaternion_inputs += config().input_ports.count(port);
    }
    if (quaternion_inputs != 0 && quaternion_inputs != 4) {
      throw std::invalid_argument("supply all of qx, qy, qz, qw or omit all four");
    }
    target.keep_current_orientation = quaternion_inputs == 0;
    if (!target.keep_current_orientation) {
      const auto qx = getInput<double>("qx");
      const auto qy = getInput<double>("qy");
      const auto qz = getInput<double>("qz");
      const auto qw = getInput<double>("qw");
      if (!qx || !qy || !qz || !qw) {
        throw std::invalid_argument("qx, qy, qz and qw must have valid values");
      }
      target.pose.orientation.x = qx.value();
      target.pose.orientation.y = qy.value();
      target.pose.orientation.z = qz.value();
      target.pose.orientation.w = qw.value();
    }
    const auto output = setOutput("pose", validate_tcp_pose(target));
    if (!output) {throw std::runtime_error(output.error());}
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("make_tcp_pose"), "%s", error.what());
    report_failure(config().blackboard, -1000, "INVALID_INPUT", "MakeTcpPose", error.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rwh_beyond_bt

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rwh_beyond_bt::MakeTcpPoseAction>("MakeTcpPose");
}
