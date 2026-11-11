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

#include <atomic>
#include <chrono>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "gtest/gtest.h"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rwh_beyond_msgs/action/run_demo.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using Feedback = rwh_beyond_msgs::action::RunDemo::Feedback;

class ClearOctomapTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    server_ = std::make_shared<rclcpp::Node>("octomap_mock");
    client_ = std::make_shared<rclcpp::Node>("octomap_bt_client");
    clear_ = server_->create_service<std_srvs::srv::Empty>(
      "/clear_octomap",
      [this](std_srvs::srv::Empty::Request::SharedPtr,
      std_srvs::srv::Empty::Response::SharedPtr)
      {
        ++clears_;
      });
    scene_ = server_->create_service<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene",
        [this](moveit_msgs::srv::GetPlanningScene::Request::SharedPtr,
        moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response) {
          ++queries_;
          if (!empty_) {response->scene.world.octomap.octomap.data = {1, 2};}
      });
    cloud_ = server_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/filtered_points", rclcpp::SensorDataQoS());
    timer_ = server_->create_wall_timer(20ms, [this]() {
          if (clears_ == 0 || !publish_) {return;}
          sensor_msgs::msg::PointCloud2 message;
          message.width = 1;
          message.height = 1;
          if (!stale_) {message.header.stamp = server_->now();}
          cloud_->publish(message);
    });
    executor_.add_node(server_);
    spinner_ = std::thread([this]() {executor_.spin();});
    board_ = BT::Blackboard::create();
    board_->set("node", client_);
    factory_.registerFromPlugin(BT_PLUGIN_PATH);
    factory_.registerSimpleAction("Next", [this](BT::TreeNode &) {
        ++next_;
        return BT::NodeStatus::SUCCESS;
    });
  }

  void TearDown() override
  {
    executor_.cancel();
    spinner_.join();
  }

  BT::Tree tree(const std::string & node)
  {
    return factory_.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\">" + node +
      "</BehaviorTree></root>", board_);
  }

  BT::NodeStatus finish(BT::Tree & tree)
  {
    const auto end = std::chrono::steady_clock::now() + 3s;
    auto status = tree.tickOnce();
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(2ms);
      status = tree.tickOnce();
    }
    return status;
  }

  rclcpp::Node::SharedPtr server_, client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_;
  rclcpp::Service<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  BT::Blackboard::Ptr board_;
  BT::BehaviorTreeFactory factory_;
  std::atomic<int> clears_{0}, queries_{0}, next_{0};
  std::atomic<bool> stale_{false}, empty_{false}, publish_{true};
};

TEST_F(ClearOctomapTest, ClearsOnceAndBlocksNextUntilFreshMap)
{
  auto bt = tree("<Sequence><ClearOctomap timeout=\"2\"/><Next/></Sequence>");
  EXPECT_EQ(bt.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(next_, 0);
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(clears_, 1);
  EXPECT_GE(queries_, 1);
  EXPECT_EQ(next_, 1);
  bt.haltTree();
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(clears_, 1);
  EXPECT_EQ(next_, 2);
}

TEST_F(ClearOctomapTest, RepeatOptionClearsOnEachActivation)
{
  auto bt = tree("<ClearOctomap once=\"false\" timeout=\"2\"/>");
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  bt.haltTree();
  ASSERT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(clears_, 2);
}

TEST_F(ClearOctomapTest, StaleCloudFailsWithoutStartingNextOrRetryingClear)
{
  stale_ = true;
  auto bt = tree("<Sequence><ClearOctomap timeout=\"0.5\"/><Next/></Sequence>");
  EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
  EXPECT_EQ(clears_, 1);
  EXPECT_EQ(next_, 0);
  const auto feedback = board_->get<Feedback>("demo_feedback");
  EXPECT_EQ(feedback.current_stage, "clear_octomap");
  EXPECT_EQ(feedback.error_name, "OCTOMAP_TIMEOUT");
  EXPECT_NE(feedback.error_msg.find("fresh post-clear"), std::string::npos);
}

TEST_F(ClearOctomapTest, EmptyMapFailsAndFailedAttemptIsNotCached)
{
  empty_ = true;
  auto bt = tree("<ClearOctomap timeout=\"0.5\"/>");
  EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
  EXPECT_EQ(clears_, 1);
  empty_ = false;
  bt.haltTree();
  EXPECT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(clears_, 2);
}

TEST_F(ClearOctomapTest, HaltedPreparationCannotBeSkippedAsSuccessful)
{
  publish_ = false;
  auto bt = tree("<ClearOctomap timeout=\"2\"/>");
  const auto end = std::chrono::steady_clock::now() + 1s;
  while (clears_ == 0 && std::chrono::steady_clock::now() < end) {
    ASSERT_EQ(bt.tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_EQ(clears_, 1);
  bt.haltTree();
  publish_ = true;
  EXPECT_EQ(finish(bt), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(clears_, 2);
}

TEST_F(ClearOctomapTest, InvalidInputsNeverClear)
{
  for (const auto * xml : {"<ClearOctomap timeout=\"0\"/>", "<ClearOctomap fresh_clouds=\"0\"/>",
      "<ClearOctomap timeout=\"nan\"/>", "<ClearOctomap service_name=\"\"/>"})
  {
    auto bt = tree(xml);
    EXPECT_EQ(finish(bt), BT::NodeStatus::FAILURE);
    EXPECT_EQ(board_->get<Feedback>("demo_feedback").error_name, "INVALID_INPUT");
  }
  EXPECT_EQ(clears_, 0);
}
