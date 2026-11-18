// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/multiview.hpp"

#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace rwh_beyond_bt
{
namespace
{
using Points = std::vector<Eigen::Vector3d>;
Points points_in_base(const FusionView & view)
{
  const auto cloud = project_object_cloud(*view.capture, view.mask);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  Points points;
  points.reserve(cloud.width);
  for (; x != x.end(); ++x, ++y, ++z) {
    const Eigen::Vector3d point = view.base_camera * Eigen::Vector3d(*x, *y, *z);
    if (!point.allFinite() || point.cwiseAbs().maxCoeff() > 100) {
      throw std::invalid_argument("Fusion point outside finite 100m workspace");
    }
    points.push_back(point);
  }
  return points;
}
Eigen::Vector3d centroid(const Points & points)
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto & point : points) {
    sum += point;
  }
  return sum / points.size();
}
}  // namespace

void validate_fusion(const CloudFusion & fusion)
{
  if (fusion.base_frame.empty() || !std::isfinite(fusion.voxel_size) ||
    fusion.voxel_size < .001 || fusion.voxel_size > .02 ||
    !std::isfinite(fusion.max_centroid_distance) || fusion.max_centroid_distance <= 0 ||
    fusion.max_centroid_distance > .2 || fusion.views.size() > 8)
  {
    throw std::invalid_argument(
        "Invalid fusion settings (voxel 1-20mm, centroid 0-0.2m, <=8 views)");
  }
  if (std::chrono::steady_clock::now() - fusion.started > std::chrono::seconds(180)) {
    throw std::invalid_argument("Fusion scan expired; reset and observe again");
  }
  int64_t previous_stamp = -1;
  for (const auto & view : fusion.views) {
    if (!view.capture || view.capture->id.empty() ||
      view.mask.header != view.capture->depth.header ||
      !view.base_camera.matrix().allFinite() ||
      !view.base_camera.linear().isUnitary(1e-6) ||
      std::abs(view.base_camera.linear().determinant() - 1) > 1e-6)
    {throw std::invalid_argument("Invalid fusion exposure or camera transform");}
    const auto stamp = rclcpp::Time(view.capture->depth.header.stamp).nanoseconds();
    if (stamp <= previous_stamp) {
      throw std::invalid_argument("Fusion requires distinct increasing capture timestamps");
    }
    previous_stamp = stamp;
  }
}

Fusion append_fusion(
  const Fusion & previous, const Observation & capture,
  const sensor_msgs::msg::Image & mask, const Eigen::Isometry3d & base_camera)
{
  if (!previous) {
    throw std::invalid_argument("ResetObjectCloudFusion must run before accumulation");
  }
  auto next = std::make_shared<CloudFusion>(*previous);
  next->views.push_back({capture, mask, base_camera});
  validate_fusion(*next);
  const auto points = points_in_base(next->views.back());
  if (next->views.size() > 1) {
    const auto anchor = points_in_base(next->views.front());
    if ((centroid(points) - centroid(anchor)).norm() > next->max_centroid_distance) {
      throw std::invalid_argument(
          "Target centroid changed across views; object moved or SAM switched target");
    }
  }
  return next;
}

sensor_msgs::msg::PointCloud2 fuse_object_cloud(const CloudFusion & fusion)
{
  validate_fusion(fusion);
  if (fusion.views.empty()) {throw std::invalid_argument("Fusion has no observations");}
  struct Voxel
  {
    Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
    size_t count{0};
  };
  std::map<std::array<int64_t, 3>, Voxel> voxels;
  const auto anchor = centroid(points_in_base(fusion.views.front()));
  for (const auto & view : fusion.views) {
    const auto points = points_in_base(view);
    if ((centroid(points) - anchor).norm() > fusion.max_centroid_distance) {
      throw std::invalid_argument("Fusion target centroid inconsistent");
    }
    for (const auto & point : points) {
      std::array<int64_t, 3> key;
      for (int axis = 0; axis < 3; ++axis) {
        key[axis] = static_cast<int64_t>(std::floor(point[axis] / fusion.voxel_size));
      }
      auto & voxel = voxels[key];
      voxel.sum += point; ++voxel.count;
    }
  }
  if (voxels.size() < 100 || voxels.size() > 200000) {
    throw std::invalid_argument("Fused cloud outside [100,200000] voxels; adjust voxel_size");
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = fusion.views.back().capture->depth.header;
  cloud.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(voxels.size());
  const auto reference_base = fusion.views.back().base_camera.inverse();
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto & entry : voxels) {
    const auto & voxel = entry.second;
    const Eigen::Vector3d point = reference_base * (voxel.sum / voxel.count);
    *x = point.x(); *y = point.y(); *z = point.z(); ++x; ++y; ++z;
  }
  return cloud;
}
}  // namespace rwh_beyond_bt
