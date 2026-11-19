// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/multiview.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/registration/icp.h"
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

using PclCloud = pcl::PointCloud<pcl::PointXYZ>;
PclCloud::Ptr registration_cloud(const Points & points, double voxel_size, size_t max_points)
{
  auto raw = PclCloud::Ptr(new PclCloud);
  raw->reserve(points.size());
  for (const auto & point : points) {
    raw->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
  }
  auto filtered = PclCloud::Ptr(new PclCloud);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  const float leaf = static_cast<float>(std::max(voxel_size, .003));
  filter.setLeafSize(leaf, leaf, leaf);
  filter.setInputCloud(raw);
  filter.filter(*filtered);
  if (filtered->size() <= max_points) {return filtered;}
  auto sampled = PclCloud::Ptr(new PclCloud);
  sampled->reserve(max_points);
  for (size_t i = 0; i < max_points; ++i) {
    sampled->push_back((*filtered)[i * filtered->size() / max_points]);
  }
  return sampled;
}

void align_view(FusionView & view, const Points & points, const CloudFusion & fusion)
{
  Points accumulated;
  for (size_t i = 0; i + 1 < fusion.views.size(); ++i) {
    const auto & previous = fusion.views[i];
    for (const auto & point : points_in_base(previous)) {
      accumulated.push_back(previous.correction * point);
    }
  }
  const auto & settings = fusion.icp;
  auto source = registration_cloud(points, fusion.voxel_size, settings.max_points);
  auto target = registration_cloud(accumulated, fusion.voxel_size, settings.max_points);
  view.icp_status = "insufficient points";
  if (source->size() < static_cast<size_t>(settings.min_correspondences) ||
    target->size() < static_cast<size_t>(settings.min_correspondences)) {return;}

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(settings.max_correspondence_distance);
  icp.setMaximumIterations(settings.max_iterations);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(1e-8);
  icp.getConvergeCriteria()->setFailureAfterMaximumIterations(true);
  PclCloud aligned;
  icp.align(aligned);
  view.icp_status = "not converged";
  if (!icp.hasConverged()) {return;}

  const Eigen::Matrix4d matrix = icp.getFinalTransformation().cast<double>();
  view.icp_status = "invalid transform";
  if (!matrix.allFinite() ||
    !matrix.block<3, 3>(0, 0).isUnitary(1e-4) ||
    std::abs(matrix.block<3, 3>(0, 0).determinant() - 1) > 1e-4)
  {return;}
  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  correction.matrix() = matrix;
  const double displacement = (correction * centroid(points) - centroid(points)).norm();
  const double rotation = Eigen::AngleAxisd(correction.linear()).angle();
  view.icp_status = "correction exceeds safety limit";
  if (displacement > settings.max_translation ||
    rotation > settings.max_rotation_deg * M_PI / 180) {return;}

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target);
  size_t matches = 0;
  double squared_error = 0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (const auto & point : aligned) {
    if (tree.nearestKSearch(point, 1, indices, distances) == 1 &&
      distances[0] <= settings.max_correspondence_distance * settings.max_correspondence_distance)
    {
      ++matches;
      squared_error += distances[0];
    }
  }
  view.icp_overlap = static_cast<double>(matches) / source->size();
  view.icp_rmse = matches ? std::sqrt(squared_error / matches) :
    std::numeric_limits<double>::infinity();
  view.icp_status = "insufficient overlap or high RMSE";
  if (matches < static_cast<size_t>(settings.min_correspondences) ||
    view.icp_overlap < settings.min_overlap || view.icp_rmse > settings.max_rmse) {return;}
  view.correction = correction;
  view.icp_accepted = true;
  view.icp_status = "accepted";
}
}  // namespace

void validate_fusion(const CloudFusion & fusion)
{
  const auto & icp = fusion.icp;
  if (icp.max_iterations < 1 || icp.max_iterations > 200 ||
    icp.max_points < 100 || icp.max_points > 50000 ||
    icp.min_correspondences < 10 || icp.min_correspondences > icp.max_points ||
    !std::isfinite(icp.max_correspondence_distance) || icp.max_correspondence_distance < .001 ||
    icp.max_correspondence_distance > .1 ||
    !std::isfinite(icp.min_overlap) || icp.min_overlap < .1 || icp.min_overlap > 1 ||
    !std::isfinite(icp.max_rmse) || icp.max_rmse < .0001 ||
    icp.max_rmse > icp.max_correspondence_distance ||
    !std::isfinite(icp.max_translation) || icp.max_translation <= 0 ||
    icp.max_translation > .03 || !std::isfinite(icp.max_rotation_deg) ||
    icp.max_rotation_deg <= 0 || icp.max_rotation_deg > 15)
  {
    throw std::invalid_argument("Invalid ICP settings; check bt_motion.icp_* parameters");
  }
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
      std::abs(view.base_camera.linear().determinant() - 1) > 1e-6 ||
      !view.correction.matrix().allFinite() ||
      !view.correction.linear().isUnitary(1e-4) ||
      std::abs(view.correction.linear().determinant() - 1) > 1e-4)
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
    if (next->use_icp) {
      const auto started = std::chrono::steady_clock::now();
      align_view(next->views.back(), points, *next);
      next->views.back().icp_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    } else {
      next->views.back().icp_status = "disabled";
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
    for (const auto & raw_point : points) {
      const Eigen::Vector3d point = view.correction * raw_point;
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
  return largest_object_cluster(std::move(cloud));
}
}  // namespace rwh_beyond_bt
