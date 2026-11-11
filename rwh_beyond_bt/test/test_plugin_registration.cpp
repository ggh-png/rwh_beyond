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

#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "gtest/gtest.h"
#include "rwh_beyond_msgs/msg/grasp_candidate.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

TEST(PluginRegistration, IndividualLibrariesMatchCompatibilityLoader)
{
  const std::vector<std::pair<std::string, std::string>> plugins = {
    {"clear_octomap", "ClearOctomap"}, {"move_home", "MoveHome"},
    {"move_observation", "MoveObservation"}, {"grasp_step", "GraspStep"},
    {"move_tcp_up", "MoveTcpUp"}, {"close_gripper", "CloseGripper"},
    {"open_gripper", "OpenGripper"}, {"set_gripper", "SetGripper"},
    {"move_tcp", "MoveTcp"}, {"make_tcp_pose", "MakeTcpPose"},
    {"check_grasp_ready", "CheckGraspReady"}, {"segment_object", "SegmentObject"},
    {"build_object_cloud", "BuildObjectCloud"}, {"infer_grasps", "InferGrasps"},
    {"select_feasible_grasp", "SelectFeasibleGrasp"},
    {"move_to_pregrasp", "MoveToPregrasp"}, {"approach_grasp", "ApproachGrasp"},
    {"attach_object", "AttachObject"}, {"lift_object", "LiftObject"}
  };
  BT::BehaviorTreeFactory individual;
  const auto builtin_count = individual.manifests().size();
  for (const auto & entry : plugins) {
    const auto library = "rwh_beyond_" + entry.first + "_action_bt_node";
    ASSERT_NO_THROW(individual.registerFromPlugin(
      std::string(BT_PLUGIN_DIR) + "/" + BT::SharedLibrary::getOSName(library)));
    ASSERT_EQ(individual.manifests().count(entry.second), 1u);
  }
  EXPECT_EQ(individual.manifests().size(), builtin_count + plugins.size());

  BT::BehaviorTreeFactory aggregate;
  ASSERT_NO_THROW(aggregate.registerFromPlugin(BT_PLUGIN_PATH));
  ASSERT_EQ(aggregate.manifests().size(), individual.manifests().size());
  for (const auto & entry : plugins) {
    const auto & actual = aggregate.manifests().at(entry.second).ports;
    const auto & expected = individual.manifests().at(entry.second).ports;
    ASSERT_EQ(actual.size(), expected.size());
    for (const auto & port : expected) {
      ASSERT_EQ(actual.count(port.first), 1u);
      EXPECT_EQ(actual.at(port.first).direction(), port.second.direction());
      EXPECT_EQ(actual.at(port.first).type(), port.second.type());
    }
    if (entry.second != "ClearOctomap" && entry.second != "MakeTcpPose") {
      EXPECT_EQ(actual.count("server_name"), 1u);
      EXPECT_EQ(actual.count("server_timeout"), 1u);
    }
  }
  const auto & manifests = individual.manifests();
  EXPECT_EQ(manifests.at("SegmentObject").ports.at("mask").type(),
    typeid(sensor_msgs::msg::Image));
  EXPECT_EQ(manifests.at("BuildObjectCloud").ports.at("cloud").type(),
    typeid(sensor_msgs::msg::PointCloud2));
  EXPECT_EQ(manifests.at("InferGrasps").ports.at("candidates").type(),
    typeid(std::vector<rwh_beyond_msgs::msg::GraspCandidate>));
}
