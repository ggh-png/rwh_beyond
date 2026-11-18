// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#ifndef RWH_BEYOND_BT__PLUGINS__ACTION__JOINT_POSE_ACTIONS_HPP_
#define RWH_BEYOND_BT__PLUGINS__ACTION__JOINT_POSE_ACTIONS_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rwh_beyond_bt/plugins/action/motion_action.hpp"

namespace rwh_beyond_bt
{
class CaptureJointPoseAction : public BT::StatefulActionNode
{
public:
  using BT::StatefulActionNode::StatefulActionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {context_.reset();}

private:
  std::shared_ptr<MotionContext> context_;
  std::chrono::steady_clock::time_point deadline_;
};
class CheckJointPosesAction : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
class MoveJointPoseAction : public MotionAction
{
public:
  using MotionAction::MotionAction;
  ~MoveJointPoseAction() override {stop();}
  static BT::PortsList providedPorts();

private:
  void prepare() override;
  void perform() override;
  double default_settle() override
  {
    return context_->number("joint_pose_settle_time", MotionAction::default_settle(), 0, 30);
  }
  std::string pose_name_;
  std::vector<double> target_;
};
}  // namespace rwh_beyond_bt
#endif  // RWH_BEYOND_BT__PLUGINS__ACTION__JOINT_POSE_ACTIONS_HPP_
