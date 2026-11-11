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

#include "rwh_beyond_bt/plugins/action/move_tcp_action.hpp"

#include <memory>

#include <cmath>
#include <stdexcept>

#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "rwh_beyond_bt/tcp_pose.hpp"
#include "tf2_ros/buffer.h"


namespace rwh_beyond_bt
{

MoveTcpAction::MoveTcpAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: MoveGroupNode(xml_tag_name, action_name, conf)
{}

BT::PortsList MoveTcpAction::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<TcpPose>("pose", "Required TcpPose object from the blackboard"),
      BT::InputPort<std::string>("tcp_link", "tcp_link", "Controlled TCP link"),
    });
}

void MoveTcpAction::on_tick()
{
  set_stage("move_tcp");
  try {
    const auto input = getInput<TcpPose>("pose");
    const auto link = getInput<std::string>("tcp_link");
    if (!input || !link || link.value().empty()) {
      throw std::invalid_argument("a valid TcpPose input and nonempty tcp_link are required");
    }
    const auto command = validate_tcp_pose(input.value());
    auto target = command.pose;
    if (command.relative || command.keep_current_orientation) {
      auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
      if (!tf_buffer) {throw std::runtime_error("tf_buffer is unavailable");}
      const auto transform = tf_buffer->lookupTransform(
          command.frame_id, link.value(), tf2::TimePointZero);
      if (command.relative) {
        target.position.x += transform.transform.translation.x;
        target.position.y += transform.transform.translation.y;
        target.position.z += transform.transform.translation.z;
      }
      if (command.keep_current_orientation) {
        target.orientation = transform.transform.rotation;
      }
    }
    if (!std::isfinite(target.position.x) || !std::isfinite(target.position.y) ||
      !std::isfinite(target.position.z))
    {
      throw std::invalid_argument("resolved TCP position must be finite");
    }
    set_common_request();
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::PositionConstraint position;
    position.header.frame_id = command.frame_id;
    position.link_name = link.value();
    position.weight = 1.0;
    position.constraint_region.primitives.resize(1);
    position.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
    position.constraint_region.primitives[0].dimensions = {0.005};
    position.constraint_region.primitive_poses = {target};
    constraints.position_constraints = {position};
    moveit_msgs::msg::OrientationConstraint orientation;
    orientation.header.frame_id = command.frame_id;
    orientation.link_name = link.value();
    orientation.orientation = target.orientation;
    orientation.absolute_x_axis_tolerance = 0.01;
    orientation.absolute_y_axis_tolerance = 0.01;
    orientation.absolute_z_axis_tolerance = 0.01;
    orientation.weight = 1.0;
    constraints.orientation_constraints = {orientation};
    goal_.request.goal_constraints = {constraints};
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
      return std::make_unique<rwh_beyond_bt::MoveTcpAction>(name, "move_action", config);
    };
  factory.registerBuilder<rwh_beyond_bt::MoveTcpAction>("MoveTcp", builder);
}
