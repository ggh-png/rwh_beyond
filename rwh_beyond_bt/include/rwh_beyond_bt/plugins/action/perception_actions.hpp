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

#ifndef RWH_BEYOND_BT__PLUGINS__ACTION__PERCEPTION_ACTIONS_HPP_
#define RWH_BEYOND_BT__PLUGINS__ACTION__PERCEPTION_ACTIONS_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rwh_beyond_msgs/action/segment_image.hpp"
#include "rwh_beyond_msgs/action/infer_grasps.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace rwh_beyond_bt
{
// One immutable, registered RGB-D exposure owned by the BT Blackboard.
struct RgbdObservation
{
  sensor_msgs::msg::Image rgb, depth;
  sensor_msgs::msg::CameraInfo info;
  sensor_msgs::msg::JointState joints;
  std::string id;
};
using Observation = std::shared_ptr<const RgbdObservation>;

sensor_msgs::msg::PointCloud2 project_object_cloud(
  const RgbdObservation & observation, const sensor_msgs::msg::Image & mask);

// Keep the largest Euclidean component of an XYZ object cloud at 1 cm connectivity.
sensor_msgs::msg::PointCloud2 largest_object_cluster(sensor_msgs::msg::PointCloud2 cloud);

class CaptureRgbdAction : public BT::StatefulActionNode
{
public:
  CaptureRgbdAction(const std::string & name, const BT::NodeConfiguration & config);
  ~CaptureRgbdAction() override;
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct State;
  std::unique_ptr<State> state_;
};

class BuildObjectCloudAction : public BT::SyncActionNode
{
public:
  BuildObjectCloudAction(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

class SegmentObjectAction
  : public nav2_behavior_tree::BtActionNode<rwh_beyond_msgs::action::SegmentImage>
{
public:
  using Action = rwh_beyond_msgs::action::SegmentImage;
  SegmentObjectAction(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void on_tick() override;
  void on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback) override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

private:
  std::chrono::steady_clock::time_point deadline_;
};

class InferGraspsAction
  : public nav2_behavior_tree::BtActionNode<rwh_beyond_msgs::action::InferGrasps>
{
public:
  using Action = rwh_beyond_msgs::action::InferGrasps;
  InferGraspsAction(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void on_tick() override;
  void on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback) override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

private:
  std::chrono::steady_clock::time_point deadline_;
};
}  // namespace rwh_beyond_bt
#endif  // RWH_BEYOND_BT__PLUGINS__ACTION__PERCEPTION_ACTIONS_HPP_
