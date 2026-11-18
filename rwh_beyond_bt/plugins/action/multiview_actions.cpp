// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/multiview_actions.hpp"

#include <memory>
#include <stdexcept>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace rwh_beyond_bt
{
namespace
{
template<class T> T input(const BT::TreeNode & node, const std::string & port)
{
  auto value = node.getInput<T>(port);
  if (!value) {throw std::invalid_argument("Missing/invalid " + port + ": " + value.error());}
  return value.value();
}
BT::NodeStatus failure(const BT::TreeNode & node, const std::exception & error)
{
  report_exception(node.config().blackboard, error, node.registrationName());
  return BT::NodeStatus::FAILURE;
}
}  // namespace

BT::PortsList ResetObjectCloudFusionAction::providedPorts()
{
  return {BT::InputPort<double>("voxel_size", .003, "Voxel edge length in meters"),
    BT::InputPort<double>("max_centroid_distance", .08, "Maximum target centroid drift in meters"),
    BT::OutputPort<Fusion>("fusion")};
}
BT::NodeStatus ResetObjectCloudFusionAction::tick()
{
  begin_stage(config().blackboard, "reset_object_cloud_fusion", name());
  try {
    auto session = std::make_shared<CloudFusion>();
    session->base_frame = MotionContext::get(config().blackboard)->base;
    session->voxel_size = input<double>(*this, "voxel_size");
    session->max_centroid_distance = input<double>(*this, "max_centroid_distance");
    validate_fusion(*session);
    setOutput("fusion", Fusion(session));
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & error) {
    return failure(*this, error);
  }
}

BT::PortsList AccumulateObjectCloudAction::providedPorts()
{
  return {BT::InputPort<Observation>("capture"), BT::InputPort<sensor_msgs::msg::Image>("mask"),
    BT::BidirectionalPort<Fusion>("fusion")};
}
void AccumulateObjectCloudAction::prepare()
{
  result_.reset();
  previous_ = required<Fusion>("fusion");
  capture_ = required<Observation>("capture");
  mask_ = required<sensor_msgs::msg::Image>("mask");
  if (!previous_ || previous_->base_frame != context_->base) {
    throw std::invalid_argument("Fusion must be reset in the current robot base frame");
  }
}
void AccumulateObjectCloudAction::perform()
{
  context_->validate_capture(capture_);
  result_ = append_fusion(previous_, capture_, mask_, context_->camera_pose_at(capture_, cancel_));
}
void AccumulateObjectCloudAction::publish_outputs()
{
  if (result_) {setOutput("fusion", result_);}
}

BT::PortsList BuildFusedObjectCloudAction::providedPorts()
{
  return {BT::InputPort<Fusion>("fusion"), BT::InputPort<int>("min_views", 1, "Minimum view count"),
    BT::OutputPort<sensor_msgs::msg::PointCloud2>("cloud")};
}
BT::NodeStatus BuildFusedObjectCloudAction::tick()
{
  begin_stage(config().blackboard, "build_fused_object_cloud", name());
  try {
    const auto fusion = input<Fusion>(*this, "fusion");
    const auto minimum = input<int>(*this, "min_views");
    if (!fusion || minimum < 1 || minimum > 8 || fusion->views.size() < size_t(minimum)) {
      throw std::invalid_argument("Insufficient multi-view observations (min_views must be 1-8)");
    }
    const auto cloud = fuse_object_cloud(*fusion);
    setOutput("cloud", cloud);
    auto display = cloud;
    display.header.frame_id = fusion->base_frame;
    sensor_msgs::PointCloud2Iterator<float> x(display, "x"), y(display, "y"), z(display, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      const Eigen::Vector3d point = fusion->views.back().base_camera * Eigen::Vector3d(*x, *y, *z);
      *x = point.x(); *y = point.y(); *z = point.z();
    }
    const auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    // Retain the transient-local publisher as long as this BT node exists.
    if (!publisher_) {
      publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "grasp/fused_object_cloud", rclcpp::QoS(1).reliable().transient_local());
    }
    publisher_->publish(display);
    RCLCPP_INFO(node->get_logger(), "Fused %zu views into %u voxels in %s", fusion->views.size(),
      cloud.width, fusion->base_frame.c_str());
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception & error) {
    return failure(*this, error);
  }
}
}  // namespace rwh_beyond_bt
