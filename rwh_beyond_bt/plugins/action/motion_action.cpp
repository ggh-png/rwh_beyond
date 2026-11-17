// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/motion_action.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>

#include "rwh_beyond_bt/diagnostics.hpp"

namespace rwh_beyond_bt
{
BT::PortsList motion_ports(std::initializer_list<const char *> names)
{
  BT::PortsList ports = {
    BT::InputPort<Plan>("plan"), BT::OutputPort<Plan>("selected_plan"),
    BT::InputPort<Observation>("capture"),
    BT::InputPort<sensor_msgs::msg::Image>("mask"),
    BT::InputPort<sensor_msgs::msg::PointCloud2>("cloud"),
    BT::InputPort<std::vector<rwh_beyond_msgs::msg::GraspCandidate>>("candidates"),
    BT::InputPort<bool>("use_estimated_width", false, "Estimate close width from point cloud"),
    BT::InputPort<bool>("release_previous", false, "Explicitly release prior payload in place"),
    BT::InputPort<double>("pregrasp_distance", "Approach offset (m); omit for bt_motion YAML"),
    BT::InputPort<double>("lift_distance", "Lift distance (m); omit for bt_motion YAML"),
    BT::InputPort<std::vector<double>>("home_joint_positions", "Six joint angles (rad)"),
    BT::InputPort<double>("x", "Base X offset (m)"),
    BT::InputPort<double>("y", "Base Y offset (m)"),
    BT::InputPort<double>("z", "Base Z offset (m)"),
    BT::InputPort<double>("tilt_deg", "Home-local Y tilt (deg)"),
    BT::InputPort<double>("settle_time", "Observation settling time (s)"),
    BT::InputPort<double>("opening", "Gripper total gap (m)"),
    BT::InputPort<double>("duration", "Gripper duration (s)"),
    BT::OutputPort<TcpPose>("pregrasp"), BT::OutputPort<TcpPose>("grasp_pose"),
    BT::OutputPort<double>("grasp_width"), BT::OutputPort<double>("close_width")};
  for (auto it = ports.begin(); it != ports.end(); ) {
    if (std::find_if(names.begin(), names.end(),
      [&](const char * name) {return it->first == name;}) == names.end())
    {
      it = ports.erase(it);
    } else {
      ++it;
    }
  }
  return ports;
}

MotionAction::MotionAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), context_(MotionContext::get(config.blackboard)) {}

MotionAction::~MotionAction() {stop();}

BT::NodeStatus MotionAction::onStart()
{
  begin_stage(config().blackboard, registrationName(), name());
  try {
    input_plan_.reset();
    prepare();
    home_ = context_->parameter<std::vector<double>>("home_joint_positions", {0, 0, 0, 0, 0, 0});
    if (config().input_ports.count("home_joint_positions")) {
      home_ = required<std::vector<double>>("home_joint_positions");
    }
    if (home_.size() != 6) {throw std::invalid_argument("Home requires six finite radians");}
    for (double q : home_) {
      if (!std::isfinite(q)) {throw std::invalid_argument("Invalid home joint");}
    }
    x_ = scalar("x", 0, -.30, .30);
    y_ = scalar("y", 0, -.30, .30);
    z_ = scalar("z", default_z(), minimum_z(), .3);
    tilt_ = context_->parameter<std::vector<double>>("observation_tilt_rpy_deg", {0, 30, 0});
    if (tilt_.size() != 3) {throw std::invalid_argument("Observation RPY requires three angles");}
    for (double q : tilt_) {
      if (!std::isfinite(q) || std::abs(q) > 90) {
        throw std::invalid_argument("Observation tilt outside +/-90 deg");
      }
    }
    tilt_[1] = scalar("tilt_deg", tilt_[1], -90, 90);
    settle_ = scalar("settle_time", context_->number("observation_settle_time", .5, 0, 30), 0, 30);
    opening_ = scalar("opening", context_->number("gripper_open_width", .10, 0, .10), 0, .10);
    duration_ = scalar("duration", 3, .001, 30);
    cancel_ = std::make_shared<std::atomic_bool>(false);
    future_ = std::async(std::launch::async, [this]() {
          std::unique_lock<std::mutex> lock(context_->operation, std::try_to_lock);
          if (!lock.owns_lock()) {
            throw ActionFailure(
              ErrorCode::CONCURRENT_OPERATION, "bt", "CONCURRENT_OPERATION", registrationName(),
              "Concurrent BT robot operation refused");
          }
          try {
            perform();
          } catch (const std::exception & error) {
            report_exception(config().blackboard, error, registrationName());
            try {
              context_->cleanup(std::make_shared<std::atomic_bool>(false));
            } catch (const std::exception & e) {
              RCLCPP_ERROR(context_->node->get_logger(), "BT scene cleanup: %s", e.what());
            }
            throw;
          }
      });
    return BT::NodeStatus::RUNNING;
  } catch (const std::exception & e) {
    return failure(e);
  }
}

BT::NodeStatus MotionAction::onRunning()
{
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return BT::NodeStatus::RUNNING;
  }
  try {
    future_.get();
    publish_outputs();
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & e) {
    return failure(e);
  }
}

void MotionAction::onHalted() {stop();}
void MotionAction::stop()
{
  if (cancel_) {cancel_->store(true);}
  if (future_.valid()) {
    try {
      future_.get();
    } catch (...) {
    }
  }
}

BT::NodeStatus MotionAction::failure(const std::exception & error)
{
  report_exception(config().blackboard, error, registrationName());
  RCLCPP_ERROR(context_->node->get_logger(), "%s: %s", name().c_str(), error.what());
  return BT::NodeStatus::FAILURE;
}

double MotionAction::scalar(const std::string & port, double fallback, double low, double high)
{
  double value = config().input_ports.count(port) ? required<double>(port) : fallback;
  if (!std::isfinite(value) || value < low || value > high) {
    throw std::invalid_argument("Invalid " + port);
  }
  return value;
}

void MotionAction::prepare_plan()
{
  input_plan_ = required<Plan>("plan");
  if (!input_plan_) {throw std::invalid_argument("Null grasp plan");}
}

void MotionAction::phase(GraspPlan::Phase expected)
{
  if (input_plan_ != context_->selected || input_plan_->phase != expected) {
    throw ActionFailure(ErrorCode::INVALID_PLAN, "bt", "INVALID_PLAN", registrationName(),
      "Grasp plan is stale or BT motion stages are out of order");
  }
}

State MotionAction::home_state(const State & state)
{
  State result = state;
  for (size_t i = 0; i < result.joint_state.name.size(); ++i) {
    for (int j = 1; j <= 6; ++j) {
      if (result.joint_state.name[i] == "joint" + std::to_string(j)) {
        result.joint_state.position[i] = home_[j - 1];
      }
    }
  }
  return result;
}

bool MotionAction::at_home()
{
  auto values = context_->feedback();
  double tolerance = context_->number("home_joint_tolerance", .01, .001, .1);
  for (int i = 1; i <= 6; ++i) {
    if (std::abs(values.at("joint" + std::to_string(i)) - home_[i - 1]) > tolerance) {
      return false;
    }
  }
  return true;
}
}  // namespace rwh_beyond_bt
