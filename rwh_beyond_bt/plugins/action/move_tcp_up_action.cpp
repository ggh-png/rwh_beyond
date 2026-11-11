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

#include "rwh_beyond_bt/plugins/action/move_tcp_up_action.hpp"

#include <memory>

#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "tf2_ros/buffer.h"


namespace rwh_beyond_bt
{

MoveTcpUpAction::MoveTcpUpAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: MoveGroupNode(xml_tag_name, action_name, conf)
{}

void MoveTcpUpAction::on_tick()
{
  set_stage("move_tcp_up");
  auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  const auto transform = tf_buffer->lookupTransform(
      "base_link", "tcp_link", tf2::TimePointZero, tf2::durationFromSec(1.0));

  set_common_request();
  moveit_msgs::msg::Constraints constraints;
  moveit_msgs::msg::PositionConstraint position;
  position.header.frame_id = "base_link";
  position.link_name = "tcp_link";
  position.weight = 1.0;
  position.constraint_region.primitives.resize(1);
  position.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
  position.constraint_region.primitives[0].dimensions = {0.005};
  geometry_msgs::msg::Pose target;
  target.position.x = transform.transform.translation.x;
  target.position.y = transform.transform.translation.y;
  target.position.z = transform.transform.translation.z + 0.30;
  target.orientation = transform.transform.rotation;
  position.constraint_region.primitive_poses = {target};
  constraints.position_constraints = {position};

  moveit_msgs::msg::OrientationConstraint orientation;
  orientation.header.frame_id = "base_link";
  orientation.link_name = "tcp_link";
  orientation.orientation = target.orientation;
  orientation.absolute_x_axis_tolerance = 0.01;
  orientation.absolute_y_axis_tolerance = 0.01;
  orientation.absolute_z_axis_tolerance = 0.01;
  orientation.weight = 1.0;
  constraints.orientation_constraints = {orientation};
  goal_.request.goal_constraints = {constraints};
}

}  // namespace rwh_beyond_bt

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rwh_beyond_bt::MoveTcpUpAction>(name, "move_action", config);
    };
  factory.registerBuilder<rwh_beyond_bt::MoveTcpUpAction>("MoveTcpUp", builder);
}
