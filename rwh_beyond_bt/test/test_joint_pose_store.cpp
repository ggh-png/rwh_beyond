// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rwh_beyond_bt/joint_pose_store.hpp"

using namespace rwh_beyond_bt;  // NOLINT(build/namespaces)

class JointPoseStoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    directory = std::filesystem::temp_directory_path() /
      ("joint-pose-test-" + std::to_string(getpid()) +
      "-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory);
    path = (directory / "poses.yaml").string();
  }
  void TearDown() override {std::filesystem::remove_all(directory);}
  std::filesystem::path directory;
  std::string path;
};
TEST_F(JointPoseStoreTest, PersistsNamedRadiansAndKeepsOtherPoses)
{
  const std::vector<double> front{.123456789012345, -.2, .3, -.4, .5, -.6};
  const std::vector<double> left{.4, -.3, .2, -.1, .6, -.5};
  save_joint_pose(path, "front", front); save_joint_pose(path, "left", left);
  EXPECT_EQ(load_joint_pose(path, "front"), front);
  EXPECT_EQ(load_joint_pose(path, "left"), left);
  EXPECT_EQ(read_joint_poses(path).size(), 2u);
}
TEST_F(JointPoseStoreTest, ExistingPoseNeedsExplicitOverwrite)
{
  const std::vector<double> original(6, .1), replacement(6, .2);
  save_joint_pose(path, "front", original);
  EXPECT_THROW(save_joint_pose(path, "front", replacement), std::runtime_error);
  EXPECT_EQ(load_joint_pose(path, "front"), original);
  save_joint_pose(path, "front", replacement, true);
  EXPECT_EQ(load_joint_pose(path, "front"), replacement);
}
TEST_F(JointPoseStoreTest, RejectsIncompleteNonfiniteAndInvalidNamesWithoutCreatingFile)
{
  EXPECT_THROW(save_joint_pose(path, "front", {0., 0.}), std::invalid_argument);
  auto bad = std::vector<double>(6, 0.); bad[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(save_joint_pose(path, "front", bad), std::invalid_argument);
  EXPECT_THROW(save_joint_pose(path, "../front", std::vector<double>(6, 0.)),
    std::invalid_argument);
  EXPECT_FALSE(std::filesystem::exists(path));
}
TEST_F(JointPoseStoreTest, MissingFileAndMissingPoseAreNotZeroTargets)
{
  EXPECT_THROW(load_joint_pose(path, "front"), std::runtime_error);
  save_joint_pose(path, "front", std::vector<double>(6, .1));
  EXPECT_THROW(load_joint_pose(path, "left"), std::runtime_error);
}
TEST_F(JointPoseStoreTest, RejectsDegreesAndWrongJointOrder)
{
  std::ofstream(path) <<
    "version: 1\nunits: degrees\n"
    "joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]\nposes: {}\n";
  EXPECT_THROW(read_joint_poses(path), std::invalid_argument);
  std::ofstream(path) <<
    "version: 1\nunits: radians\n"
    "joint_names: [joint2, joint1, joint3, joint4, joint5, joint6]\nposes: {}\n";
  EXPECT_THROW(read_joint_poses(path), std::invalid_argument);
}
TEST_F(JointPoseStoreTest, RefusesMalformedExistingFileWithoutOverwritingIt)
{
  const std::string malformed = "not a joint pose file\n";
  std::ofstream(path) << malformed;
  EXPECT_THROW(save_joint_pose(path, "front", std::vector<double>(6, 0.)), std::invalid_argument);
  std::ifstream file(path); std::string line; std::getline(file, line);
  EXPECT_EQ(line, "not a joint pose file");
}
