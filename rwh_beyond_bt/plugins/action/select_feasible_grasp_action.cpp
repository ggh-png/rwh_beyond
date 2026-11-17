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

#include "rwh_beyond_bt/plugins/action/select_feasible_grasp_action.hpp"

#include <cmath>
#include <memory>
#include "rwh_beyond_bt/perception_actions.hpp"

namespace rwh_beyond_bt
{

SelectFeasibleGraspAction::SelectFeasibleGraspAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: GraspActionNode(xml_tag_name, action_name, conf)
{
}

BT::PortsList SelectFeasibleGraspAction::providedPorts()
{
  return providedBasicPorts({BT::InputPort<std::string>("observation_id"),
             BT::InputPort<ObjectCloud>("cloud"),
             BT::InputPort<Observation>("capture"),
             BT::InputPort<ObjectMask>("mask"),
             BT::InputPort<GraspCandidates>("candidates"),
             BT::OutputPort<std::string>("plan_id"),
             BT::OutputPort<TcpPose>("pregrasp"),
             BT::OutputPort<TcpPose>("grasp_pose"),
             BT::InputPort<double>("pregrasp_distance", "Pregrasp offset in metres; omit for YAML"),
             BT::InputPort<double>("lift_distance", "Lift distance in metres; omit for YAML"),
             BT::InputPort<bool>("use_estimated_width", false,
        "Estimate aperture from object cloud"),
             BT::OutputPort<double>("grasp_width"),
             BT::OutputPort<double>("close_width")});
}

void SelectFeasibleGraspAction::on_tick()
{
  GraspActionNode::on_tick();
  goal_.use_estimated_width = getInput<bool>("use_estimated_width").value();
  goal_.observation_id = required_input<std::string>("observation_id");
  goal_.object_cloud = required_input<ObjectCloud>("cloud");
  const auto capture = required_input<Observation>("capture");
  if (!capture || capture->id != goal_.observation_id) {
    throw BT::RuntimeError("RGB-D capture does not match observation ID");
  }
  goal_.rgb_image = capture->rgb;
  goal_.depth_image = capture->depth;
  goal_.camera_info = capture->info;
  goal_.capture_joints = capture->joints;
  goal_.object_mask = required_input<ObjectMask>("mask");
  if (config().input_ports.count("pregrasp_distance")) {
    goal_.pregrasp_distance = required_input<double>("pregrasp_distance");
  }
  if (config().input_ports.count("lift_distance")) {
    goal_.lift_distance = required_input<double>("lift_distance");
  }
  goal_.candidates = required_input<GraspCandidates>("candidates");
}

BT::NodeStatus SelectFeasibleGraspAction::on_success()
{
  const auto status = GraspActionNode::on_success();
  if (status != BT::NodeStatus::SUCCESS) {
    return status;
  }
  if (goal_.use_estimated_width &&
    (result_.result->width_source != "point_cloud_estimate" ||
    !std::isfinite(result_.result->grasp_width) ||
    !std::isfinite(result_.result->close_width) ||
    result_.result->close_width <= 0.0 ||
    result_.result->close_width > result_.result->grasp_width ||
    result_.result->close_width > 0.1))
  {
    throw BT::RuntimeError("Invalid estimated grasp width in stage result");
  }
  setOutput("grasp_width", result_.result->grasp_width);
  setOutput("close_width", result_.result->close_width);
  setOutput("plan_id", result_.result->plan_id);
  setOutput("pregrasp", result_.result->pregrasp);
  setOutput("grasp_pose", result_.result->grasp_pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rwh_beyond_bt

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rwh_beyond_bt::SelectFeasibleGraspAction>(name,
      "grasp/select_feasible_grasp", config);
    };
  factory.registerBuilder<rwh_beyond_bt::SelectFeasibleGraspAction>("SelectFeasibleGrasp", builder);
}
