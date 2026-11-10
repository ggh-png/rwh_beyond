#pragma once

#include <chrono>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace rwh_beyond_bt
{
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class GripperCommand : public nav2_behavior_tree::BtActionNode<FollowJointTrajectory>
{
public:
  GripperCommand(
    const std::string & name, const std::string & action_name,
    const BT::NodeConfiguration & config, double aperture, std::string stage)
  : BtActionNode(name, action_name, config), aperture_(aperture), stage_(std::move(stage)) {}

  void on_tick() override
  {
    config().blackboard->set("demo_stage", stage_);
    goal_.trajectory.joint_names = {"gripper"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {aperture_};
    point.time_from_start.sec = 3;
    goal_.trajectory.points = {point};
  }

private:
  double aperture_;
  std::string stage_;
};


}  // namespace rwh_beyond_bt
