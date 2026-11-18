// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>

#include "gtest/gtest.h"
#include "rwh_beyond_bt/multiview.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace rwh_beyond_bt;  // NOLINT(build/namespaces)

namespace
{
std::shared_ptr<RgbdObservation> exposure(int second, double shift = 0)
{
  auto capture = std::make_shared<RgbdObservation>();
  capture->id = std::to_string(second);
  auto & depth = capture->depth;
  depth.header.frame_id = "camera_optical"; depth.header.stamp.sec = second;
  depth.width = 20; depth.height = 20; depth.step = 80; depth.encoding = "32FC1";
  depth.data.resize(1600);
  const float distance = .5f;
  for (size_t i = 0; i < 400; ++i) {
    std::memcpy(depth.data.data() + i * 4, &distance, 4);
  }
  capture->info.header = depth.header; capture->info.width = capture->info.height = 20;
  capture->info.k = {100, 0, 9.37 + shift * 200, 0, 100, 9.37, 0, 0, 1};
  return capture;
}
sensor_msgs::msg::Image mask_for(const Observation & capture)
{
  sensor_msgs::msg::Image mask;
  mask.header = capture->depth.header; mask.width = mask.height = 20;
  mask.step = 20; mask.encoding = "mono8"; mask.data.assign(400, 1);
  return mask;
}
Fusion empty_session()
{
  auto session = std::make_shared<CloudFusion>(); session->base_frame = "base_link";
  return session;
}
}  // namespace

TEST(MultiView, AlignsTranslatedViewsAndReturnsLastOpticalCoordinates)
{
  const auto first = exposure(1), second = exposure(2, .02);
  auto fusion = append_fusion(empty_session(), first, mask_for(first),
    Eigen::Isometry3d::Identity());
  auto transform = Eigen::Isometry3d::Identity(); transform.translation().x() = .02;
  fusion = append_fusion(fusion, second, mask_for(second), transform);
  auto cloud = fuse_object_cloud(*fusion);
  EXPECT_EQ(cloud.header, second->depth.header);
  EXPECT_EQ(cloud.width, 400u);  // The two identical surfaces occupy the same voxels.
  float lowest = std::numeric_limits<float>::max();
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
  for (; x != x.end(); ++x) {
    lowest = std::min(lowest, *x);
                                                           }
  EXPECT_NEAR(lowest, -9.37 * .005 - .02, 1e-6);
  EXPECT_EQ(fusion->views.size(), 2u);
}
TEST(MultiView, IcpCorrectsSmallPoseErrorBeforeFusion)
{
  const auto first = exposure(1), second = exposure(2);
  auto fusion = append_fusion(empty_session(), first, mask_for(first),
    Eigen::Isometry3d::Identity());
  auto transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = .004;
  fusion = append_fusion(fusion, second, mask_for(second), transform);
  ASSERT_TRUE(fusion->views.back().icp_accepted) << fusion->views.back().icp_status;
  EXPECT_NEAR(fusion->views.back().correction.translation().z(), -.004, 1e-4);
  EXPECT_GT(fusion->views.back().icp_overlap, .9);
  const auto cloud = fuse_object_cloud(*fusion);
  EXPECT_EQ(cloud.header, second->depth.header);
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  for (; z != z.end(); ++z) {
    EXPECT_NEAR(*z, .496, 1e-4);
  }
}
TEST(MultiView, IcpFallbackRetainsTfWhenViewsDoNotOverlap)
{
  const auto first = exposure(1), second = exposure(2);
  auto fusion = append_fusion(empty_session(), first, mask_for(first),
    Eigen::Isometry3d::Identity());
  auto transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = .04;
  fusion = append_fusion(fusion, second, mask_for(second), transform);
  EXPECT_FALSE(fusion->views.back().icp_accepted);
  EXPECT_TRUE(fusion->views.back().correction.matrix().isIdentity());
  EXPECT_GT(fuse_object_cloud(*fusion).width, 400u);
}
TEST(MultiView, IcpRejectsExcessivePoseCorrection)
{
  const auto first = exposure(1), second = exposure(2);
  auto fusion = append_fusion(empty_session(), first, mask_for(first),
    Eigen::Isometry3d::Identity());
  auto transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = .013;
  fusion = append_fusion(fusion, second, mask_for(second), transform);
  EXPECT_FALSE(fusion->views.back().icp_accepted);
  EXPECT_EQ(fusion->views.back().icp_status, "correction exceeds safety limit");
  EXPECT_TRUE(fusion->views.back().correction.matrix().isIdentity());
}
TEST(MultiView, ConfiguredCorrectionLimitChangesAcceptance)
{
  const auto first = exposure(1), second = exposure(2);
  auto session = std::make_shared<CloudFusion>();
  session->base_frame = "base_link";
  session->icp.max_translation = .02;
  auto fusion = append_fusion(session, first, mask_for(first), Eigen::Isometry3d::Identity());
  auto transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = .013;
  fusion = append_fusion(fusion, second, mask_for(second), transform);
  ASSERT_TRUE(fusion->views.back().icp_accepted) << fusion->views.back().icp_status;
  EXPECT_NEAR(fusion->views.back().correction.translation().z(), -.013, 1e-4);
}
TEST(MultiView, RejectsInvalidIcpSettings)
{
  CloudFusion session;
  session.base_frame = "base_link";
  session.icp.max_iterations = 0;
  EXPECT_THROW(validate_fusion(session), std::invalid_argument);
  session.icp.max_iterations = 40;
  session.icp.max_rmse = .02;
  EXPECT_THROW(validate_fusion(session), std::invalid_argument);
  session.icp.max_rmse = .004;
  session.icp.min_overlap = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validate_fusion(session), std::invalid_argument);
  session.icp.min_overlap = .35;
  session.icp.min_correspondences = session.icp.max_points + 1;
  EXPECT_THROW(validate_fusion(session), std::invalid_argument);
}
TEST(MultiView, IcpCanBeDisabled)
{
  const auto first = exposure(1), second = exposure(2);
  auto session = std::make_shared<CloudFusion>();
  session->base_frame = "base_link";
  session->use_icp = false;
  auto fusion = append_fusion(session, first, mask_for(first), Eigen::Isometry3d::Identity());
  auto transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = .004;
  fusion = append_fusion(fusion, second, mask_for(second), transform);
  EXPECT_EQ(fusion->views.back().icp_status, "disabled");
  EXPECT_TRUE(fusion->views.back().correction.matrix().isIdentity());
  EXPECT_GT(fuse_object_cloud(*fusion).width, 400u);
}
TEST(MultiView, RotatedCameraUsesAcquisitionPose)
{
  const auto first = exposure(1), second = exposure(2);
  auto fusion = append_fusion(empty_session(), first, mask_for(first),
    Eigen::Isometry3d::Identity());
  auto transform = Eigen::Isometry3d::Identity();
  transform.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  fusion = append_fusion(fusion, second, mask_for(second), transform);
  auto cloud = fuse_object_cloud(*fusion);
  EXPECT_EQ(cloud.header, second->depth.header);
  EXPECT_GE(cloud.width, 400u);
  EXPECT_LT(cloud.width, 800u);
}
TEST(MultiView, RejectsDuplicateCaptureAndDoesNotMutatePreviousSession)
{
  const auto capture = exposure(1);
  const auto previous = append_fusion(empty_session(), capture, mask_for(capture),
      Eigen::Isometry3d::Identity());
  EXPECT_THROW(append_fusion(previous, capture, mask_for(capture), Eigen::Isometry3d::Identity()),
    std::invalid_argument);
  EXPECT_EQ(previous->views.size(), 1u);
}
TEST(MultiView, RejectsMovedTargetOrDifferentObject)
{
  const auto first = exposure(1), second = exposure(2);
  auto fusion = append_fusion(empty_session(), first, mask_for(first),
    Eigen::Isometry3d::Identity());
  auto moved = Eigen::Isometry3d::Identity(); moved.translation().x() = .3;
  EXPECT_THROW(append_fusion(fusion, second, mask_for(second), moved), std::invalid_argument);
}
TEST(MultiView, ResetIsRequiredAndBadSettingsOrExpiredScanAreRejected)
{
  const auto capture = exposure(1);
  EXPECT_THROW(append_fusion(Fusion(), capture, mask_for(capture), Eigen::Isometry3d::Identity()),
    std::invalid_argument);
  CloudFusion session; session.base_frame = "base_link"; session.voxel_size = 0;
  EXPECT_THROW(fuse_object_cloud(session), std::invalid_argument);
  session.voxel_size = .003; session.started -= std::chrono::seconds(181);
  EXPECT_THROW(validate_fusion(session), std::invalid_argument);
}
TEST(MultiView, RejectsWrongMaskAndInvalidTransform)
{
  const auto capture = exposure(1);
  auto wrong = mask_for(capture); wrong.header.frame_id = "another_camera";
  EXPECT_THROW(append_fusion(empty_session(), capture, wrong, Eigen::Isometry3d::Identity()),
    std::invalid_argument);
  auto scaled = Eigen::Isometry3d::Identity(); scaled.linear() *= 2;
  EXPECT_THROW(append_fusion(empty_session(), capture, mask_for(capture), scaled),
    std::invalid_argument);
}
TEST(MultiView, VoxelCollapseFailsRatherThanSendingInsufficientPointsToModel)
{
  auto capture = exposure(1); capture->info.k[0] = capture->info.k[4] = 10000;
  auto fusion = append_fusion(empty_session(), capture, mask_for(capture),
      Eigen::Isometry3d::Identity());
  EXPECT_THROW(fuse_object_cloud(*fusion), std::invalid_argument);
}
