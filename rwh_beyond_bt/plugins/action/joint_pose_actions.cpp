// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/joint_pose_actions.hpp"

#include <cmath>
#include <stdexcept>

#include "rwh_beyond_bt/joint_pose_store.hpp"

namespace rwh_beyond_bt
{
namespace
{
template<class T> T input(const BT::TreeNode & node, const std::string & port)
{
  const auto result = node.getInput<T>(port);
  if (!result) {throw std::invalid_argument("Missing/invalid " + port + ": " + result.error());}
  return result.value();
}
std::string poses_file(const BT::TreeNode & node, MotionContext & context)
{
  std::string path;
  if (node.config().input_ports.count("poses_file")) {
    path = input<std::string>(node, "poses_file");
  }
  if (path.empty()) {
    path = context.parameter<std::string>("joint_poses_file", default_joint_poses_file());
  }
  return path;
}
BT::NodeStatus failed(const BT::TreeNode & node, const std::exception & error)
{
  report_exception(node.config().blackboard, error, node.registrationName());
  return BT::NodeStatus::FAILURE;
}
}  // namespace

BT::PortsList CaptureJointPoseAction::providedPorts()
{
  return {BT::InputPort<std::string>("pose_name"), BT::InputPort<std::string>("poses_file"),
    BT::InputPort<bool>("overwrite", false, "Replace an explicitly named existing pose"),
    BT::InputPort<double>("timeout", 5.0, "Fresh feedback wait in seconds"),
    BT::OutputPort<std::vector<double>>("joint_positions")};
}
BT::NodeStatus CaptureJointPoseAction::onStart()
{
  begin_stage(config().blackboard, "capture_joint_pose", name());
  try {
    const auto timeout = input<double>(*this, "timeout");
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > 30) {
      throw std::invalid_argument("Capture timeout must be in (0,30] seconds");
    }
    deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(
        timeout));
    context_ = MotionContext::get(config().blackboard);
    return onRunning();
  } catch (const std::exception & error) {
    return failed(*this, error);
  }
}
BT::NodeStatus CaptureJointPoseAction::onRunning()
{
  Joints values;
  try {
    values = context_->feedback();
  } catch (const std::exception & error) {
    if (std::chrono::steady_clock::now() < deadline_) {return BT::NodeStatus::RUNNING;}
    return failed(*this, error);
  }
  try {
    std::vector<double> positions;
    for (int i = 1; i <= 6; ++i) {
      positions.push_back(values.at("joint" + std::to_string(i)));
    }
    save_joint_pose(poses_file(*this, *context_), input<std::string>(*this, "pose_name"), positions,
      input<bool>(*this, "overwrite"));
    setOutput("joint_positions", positions);
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & error) {
    return failed(*this, error);
  }
}
BT::PortsList CheckJointPosesAction::providedPorts()
{
  return {BT::InputPort<std::vector<std::string>>("pose_names"),
    BT::InputPort<std::string>("poses_file")};
}
BT::NodeStatus CheckJointPosesAction::tick()
{
  begin_stage(config().blackboard, "check_joint_poses", name());
  try {
    const auto context = MotionContext::get(config().blackboard);
    const auto names = input<std::vector<std::string>>(*this, "pose_names");
    if (names.empty()) {
      throw std::invalid_argument("CheckJointPoses needs at least one pose name");
    }
    const auto file = poses_file(*this, *context);
    for (const auto & pose : names) {
      load_joint_pose(file, pose);
    }
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & error) {
    return failed(*this, error);
  }
}
BT::PortsList MoveJointPoseAction::providedPorts()
{
  return {BT::InputPort<std::string>("pose_name"), BT::InputPort<std::string>("poses_file"),
    BT::InputPort<double>("settle_time", "Settling time after verified joint arrival (seconds)"),
    BT::InputPort<bool>("release_on_arrival", false,
      "Permit payload release after confirmed arrival at this placement pose")};
}
void MoveJointPoseAction::prepare()
{
  std::unique_lock<std::mutex> lock(context_->operation, std::try_to_lock);
  if (!lock.owns_lock()) {
    throw ActionFailure(ErrorCode::CONCURRENT_OPERATION, "bt", "CONCURRENT_OPERATION",
            registrationName(), "Concurrent BT robot operation refused");
  }
  context_->release_ready = false;
  release_on_arrival_ = required<bool>("release_on_arrival");
  pose_name_ = required<std::string>("pose_name");
  target_ = load_joint_pose(poses_file(*this, *context_), pose_name_);
}
void MoveJointPoseAction::perform()
{
  auto & context = *context_;
  context.release_ready = false;
  context.prepare_motion(cancel_);
  const auto start = context.state(cancel_);
  auto goal = start;
  for (size_t i = 0; i < goal.joint_state.name.size(); ++i) {
    for (int j = 1; j <= 6; ++j) {
      if (goal.joint_state.name[i] == "joint" + std::to_string(j)) {
        goal.joint_state.position[i] = target_[j - 1];
      }
    }
  }
  const double tolerance = context.number("joint_pose_tolerance", .01, .001, .1);
  const auto reached = [&]() {
      const auto values = context.feedback();
      for (int i = 1; i <= 6; ++i) {
        if (std::abs(values.at("joint" + std::to_string(i)) - target_[i - 1]) > tolerance) {
          return false;
        }
      }
      return true;
    };
  RCLCPP_INFO(context.node->get_logger(),
      "MoveJointPose '%s' radians=[%.4f %.4f %.4f %.4f %.4f %.4f]",
    pose_name_.c_str(), target_[0], target_[1], target_[2], target_[3], target_[4], target_[5]);
  if (!reached()) {
    context.execute(context.plan(start, goal, Constraints(), cancel_, false,
      PlanningProfile::OBSERVATION), cancel_);
  }
  context.wait(reached, 10, cancel_);
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(settle_);
  context.wait([&]() {return std::chrono::steady_clock::now() >= until;}, settle_ + 1, cancel_);
  if (release_on_arrival_ && context.attached) {
    context.release_ready = true;
  }
}
}  // namespace rwh_beyond_bt
