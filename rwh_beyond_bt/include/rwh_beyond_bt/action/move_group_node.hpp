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
#include "rwh_beyond_bt/action/diagnostic_action_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace rwh_beyond_bt
{
using MoveGroup = moveit_msgs::action::MoveGroup;

class MoveGroupNode : public DiagnosticActionNode<MoveGroup>
{
public:
  MoveGroupNode(
    const std::string & name, const std::string & action_name,
    const BT::NodeConfiguration & config)
  : DiagnosticActionNode(name, action_name, config) {}

protected:
  void setCommonRequest()
  {
    goal_.request.group_name = "arm";
    goal_.request.pipeline_id = "ompl";
    goal_.request.planner_id = "RRTConnectkConfigDefault";
    goal_.request.allowed_planning_time = 10.0;
    goal_.request.max_velocity_scaling_factor = 0.1;
    goal_.request.max_acceleration_scaling_factor = 0.1;
    goal_.request.start_state.is_diff = true;
    goal_.planning_options.plan_only = false;
  }
};


}  // namespace rwh_beyond_bt
