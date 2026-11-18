// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#ifndef RWH_BEYOND_BT__PLUGINS__ACTION__MULTIVIEW_ACTIONS_HPP_
#define RWH_BEYOND_BT__PLUGINS__ACTION__MULTIVIEW_ACTIONS_HPP_

#include "rwh_beyond_bt/plugins/action/motion_action.hpp"

namespace rwh_beyond_bt
{
class ResetObjectCloudFusionAction : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
class AccumulateObjectCloudAction : public MotionAction
{
public:
  using MotionAction::MotionAction;
  ~AccumulateObjectCloudAction() override {stop();}
  static BT::PortsList providedPorts();

private:
  void prepare() override;
  void perform() override;
  void publish_outputs() override;
  Fusion previous_, result_;
  Observation capture_;
  sensor_msgs::msg::Image mask_;
};
class BuildFusedObjectCloudAction : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};
}  // namespace rwh_beyond_bt
#endif  // RWH_BEYOND_BT__PLUGINS__ACTION__MULTIVIEW_ACTIONS_HPP_
