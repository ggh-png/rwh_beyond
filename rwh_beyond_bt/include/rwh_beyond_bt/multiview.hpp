// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#ifndef RWH_BEYOND_BT__MULTIVIEW_HPP_
#define RWH_BEYOND_BT__MULTIVIEW_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "rwh_beyond_bt/plugins/action/perception_actions.hpp"

namespace rwh_beyond_bt
{
struct FusionView
{
  Observation capture;
  sensor_msgs::msg::Image mask;
  Eigen::Isometry3d base_camera{Eigen::Isometry3d::Identity()};
};
struct CloudFusion
{
  std::string base_frame;
  double voxel_size{.003}, max_centroid_distance{.08};
  std::chrono::steady_clock::time_point started{std::chrono::steady_clock::now()};
  std::vector<FusionView> views;
};
using Fusion = std::shared_ptr<const CloudFusion>;

Fusion append_fusion(
  const Fusion & previous, const Observation & capture,
  const sensor_msgs::msg::Image & mask, const Eigen::Isometry3d & base_camera);
// The result is expressed in the LAST capture optical frame, with its timestamp.
sensor_msgs::msg::PointCloud2 fuse_object_cloud(const CloudFusion & fusion);
void validate_fusion(const CloudFusion & fusion);
}  // namespace rwh_beyond_bt
#endif  // RWH_BEYOND_BT__MULTIVIEW_HPP_
