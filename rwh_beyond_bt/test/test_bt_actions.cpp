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
#include <cstring>
#include <thread>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/utils/shared_library.h"
#include "rwh_beyond_msgs/msg/grasp_candidate.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "gtest/gtest.h"
#include "rwh_beyond_bt/plugins/action/perception_actions.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"
#include "rwh_beyond_msgs/action/run_demo.hpp"
#include "rwh_beyond_msgs/msg/tcp_pose.hpp"

using TcpPose = rwh_beyond_msgs::msg::TcpPose;
using namespace std::chrono_literals;

class BtActions : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    server_ = std::make_shared<rclcpp::Node>("custom_action_mock_servers");
    client_ = std::make_shared<rclcpp::Node>("custom_action_bt_client");
    executor_.add_node(server_);
    spinner_ = std::thread([this]() {executor_.spin();});
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", client_);
    blackboard_->set("bt_loop_duration", 10ms);
    blackboard_->set("server_timeout", 1000ms);
    blackboard_->set("cancel_timeout", 1000ms);
    blackboard_->set("wait_for_service_timeout", 3000ms);
    factory_.registerFromPlugin(BT_PLUGIN_PATH);
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spinner_.joinable()) {spinner_.join();}
  }

  BT::NodeStatus run(const std::string & node)
  {
    return runXml("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\">" + node +
      "</BehaviorTree></root>");
  }

  BT::NodeStatus runXml(const std::string & xml)
  {
    BT::Tree tree;
    try {
      tree = factory_.createTreeFromText(xml, blackboard_);
    } catch (const std::exception &) {
      // Literal parsing / port mapping errors reject the tree before any goal.
      return BT::NodeStatus::FAILURE;
    }
    const auto end = std::chrono::steady_clock::now() + 3s;
    auto status = tree.tickOnce();
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(1ms);
      status = tree.tickOnce();
    }
    return status;
  }

  rclcpp::Node::SharedPtr server_, client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
};

namespace
{
rwh_beyond_bt::RgbdObservation sample_capture()
{
  rwh_beyond_bt::RgbdObservation observation;
  observation.id = "capture-42";
  observation.depth.header.frame_id = "camera";
  observation.depth.width = 12;
  observation.depth.height = 12;
  observation.depth.encoding = "16UC1";
  observation.depth.step = 24;
  observation.depth.data.resize(288);
  for (size_t i = 0; i < 144; ++i) {
    observation.depth.data[2 * i] = 0xe8;
    observation.depth.data[2 * i + 1] = 0x03;  // one metre
  }
  observation.rgb = observation.depth;
  observation.rgb.encoding = "rgb8";
  observation.rgb.step = 36;
  observation.rgb.data.assign(432, 128);
  observation.info.header = observation.depth.header;
  observation.info.width = 12;
  observation.info.height = 12;
  observation.info.k = {100., 0., 6., 0., 100., 6., 0., 0., 1.};
  observation.joints.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"};
  observation.joints.position.assign(7, 0.);
  return observation;
}
sensor_msgs::msg::Image sample_mask()
{
  sensor_msgs::msg::Image mask;
  mask.header.frame_id = "camera";
  mask.width = 12;
  mask.height = 12;
  mask.encoding = "mono8";
  mask.step = 12;
  mask.data.assign(144, 1);
  return mask;
}
}  // namespace

TEST_F(BtActions, DirectBridgesAndLocalCloudPassArtifactsIntoMotionSelection)
{
  using Segment = rwh_beyond_msgs::action::SegmentImage;
  using Infer = rwh_beyond_msgs::action::InferGrasps;
  const auto capture = std::make_shared<rwh_beyond_bt::RgbdObservation>(sample_capture());
  blackboard_->set("capture", rwh_beyond_bt::Observation(capture));
  blackboard_->set("target_text", std::string("blue cup"));
  std::vector<std::string> calls;
  auto sam = rclcpp_action::create_server<Segment>(server_, "segment_image",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;}, [&](auto handle) {
        calls.push_back("sam");
        EXPECT_EQ(handle->get_goal()->target_text, "blue cup");
        EXPECT_EQ(handle->get_goal()->rgb_image, capture->rgb);
        auto result = std::make_shared<Segment::Result>();
        result->success = true;
        result->object_mask = sample_mask();
        handle->succeed(result);
      });
  auto infer = rclcpp_action::create_server<Infer>(server_, "infer_grasps",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;}, [&](auto handle) {
        calls.push_back("infer");
        EXPECT_EQ(handle->get_goal()->input_mode, Infer::Goal::POINT_CLOUD);
        EXPECT_EQ(handle->get_goal()->object_cloud.width, 144u);
        EXPECT_EQ(handle->get_goal()->object_cloud.header, capture->depth.header);
        auto result = std::make_shared<Infer::Result>();
        result->success = true;
        result->message = "OK: test";
        rwh_beyond_msgs::msg::GraspCandidate candidate;
        candidate.pose.header = capture->depth.header;
        candidate.pose.pose.orientation.w = 1.;
        result->candidates.push_back(candidate);
        handle->succeed(result);
      });
  EXPECT_EQ(run(
    R"(<Sequence>
    <SegmentObject capture="{capture}" mask="{mask}"/>
    <BuildObjectCloud capture="{capture}" mask="{mask}" cloud="{cloud}"/>
    <InferGrasps cloud="{cloud}" candidates="{candidates}"/>
    </Sequence>)"),
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(blackboard_->get<std::vector<rwh_beyond_msgs::msg::GraspCandidate>>(
    "candidates").size(), 1u);
  EXPECT_EQ(calls, std::vector<std::string>({"sam", "infer"}));
}

TEST_F(BtActions, LocalProjectionHandlesUnitsAndRejectsWrongExposure)
{
  auto observation = sample_capture();
  auto mask = sample_mask();
  const auto cloud = rwh_beyond_bt::project_object_cloud(observation, mask);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), z(cloud, "z");
  EXPECT_NEAR(*x, -.06, 1e-6);
  EXPECT_FLOAT_EQ(*z, 1.);
  observation.depth.is_bigendian = true;
  for (size_t i = 0; i < 144; ++i) {
    std::swap(observation.depth.data[2 * i], observation.depth.data[2 * i + 1]);
  }
  const auto big_endian = rwh_beyond_bt::project_object_cloud(observation, mask);
  sensor_msgs::PointCloud2ConstIterator<float> bz(big_endian, "z");
  EXPECT_FLOAT_EQ(*bz, 1.);
  observation.depth.is_bigendian = false;
  observation.depth.encoding = "32FC1";
  observation.depth.step = 48;
  observation.depth.data.resize(576);
  const float meters = .5;
  for (size_t i = 0; i < 144; ++i) {
    std::memcpy(observation.depth.data.data() + i * 4, &meters, 4);
  }
  const auto metric = rwh_beyond_bt::project_object_cloud(observation, mask);
  sensor_msgs::PointCloud2ConstIterator<float> mz(metric, "z");
  EXPECT_FLOAT_EQ(*mz, .5);
  mask.header.stamp.sec = 2;
  EXPECT_THROW(rwh_beyond_bt::project_object_cloud(observation, mask), std::invalid_argument);
  mask = sample_mask();
  mask.data.assign(144, 0);
  EXPECT_THROW(rwh_beyond_bt::project_object_cloud(observation, mask), std::invalid_argument);
}

TEST_F(BtActions, LocalProjectionFailureNeverCallsGraspBridge)
{
  using Infer = rwh_beyond_msgs::action::InferGrasps;
  std::atomic<int> goals{0};
  auto infer = rclcpp_action::create_server<Infer>(server_, "infer_grasps",
      [&](auto, auto) {++goals; return rclcpp_action::GoalResponse::REJECT;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;}, [](auto) {});
  blackboard_->set("capture", rwh_beyond_bt::Observation(
      std::make_shared<rwh_beyond_bt::RgbdObservation>(sample_capture())));
  auto mask = sample_mask();
  mask.data.assign(144, 0);
  blackboard_->set("mask", mask);
  EXPECT_EQ(run(
    R"(<Sequence>
      <BuildObjectCloud capture="{capture}" mask="{mask}" cloud="{cloud}"/>
      <InferGrasps cloud="{cloud}" candidates="{candidates}"/>
    </Sequence>)"),
    BT::NodeStatus::FAILURE);
  EXPECT_EQ(goals, 0);
}

TEST_F(BtActions, CaptureRequiresFreshCompleteRegisteredExposure)
{
  auto rgb = server_->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw",
    rclcpp::SensorDataQoS());
  auto depth = server_->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_rect_raw",
    rclcpp::SensorDataQoS());
  auto info = server_->create_publisher<sensor_msgs::msg::CameraInfo>("camera/depth/camera_info",
    rclcpp::SensorDataQoS());
  auto joints = server_->create_publisher<sensor_msgs::msg::JointState>("feedback/joint_states",
    rclcpp::SensorDataQoS());
  auto timer = server_->create_wall_timer(10ms, [&]() {
        auto capture = sample_capture();
        const builtin_interfaces::msg::Time now = server_->now();
        capture.rgb.header.stamp = now;
        capture.depth.header.stamp = now;
        capture.info.header.stamp = now;
        capture.joints.header.stamp = now;
        rgb->publish(capture.rgb); depth->publish(capture.depth); info->publish(capture.info);
        joints->publish(capture.joints);
    });
  EXPECT_EQ(run(R"(<CaptureRgbd capture="{capture}" observation_id="{observation}" timeout="1"/>)"),
    BT::NodeStatus::SUCCESS);
  auto result = blackboard_->get<rwh_beyond_bt::Observation>("capture");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->rgb.header.stamp, result->depth.header.stamp);
  EXPECT_EQ(result->info.header.stamp, result->depth.header.stamp);
  timer->cancel();
  EXPECT_EQ(run(
    R"(<CaptureRgbd capture="{capture}" observation_id="{observation}" timeout="0.05"/>)"),
    BT::NodeStatus::FAILURE);
}

TEST_F(BtActions, HaltingSegmentationCancelsTheBridgeGoal)
{
  using Segment = rwh_beyond_msgs::action::SegmentImage;
  blackboard_->set("capture", rwh_beyond_bt::Observation(
      std::make_shared<rwh_beyond_bt::RgbdObservation>(sample_capture())));
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Segment>> active;
  std::atomic<bool> accepted{false}, canceled{false};
  auto server = rclcpp_action::create_server<Segment>(server_, "segment_image",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [&](auto) {canceled = true; return rclcpp_action::CancelResponse::ACCEPT;},
      [&](auto handle) {active = handle; accepted = true;});
  auto timer = server_->create_wall_timer(5ms, [&]() {
        if (active && active->is_canceling()) {
          active->canceled(std::make_shared<Segment::Result>());
          active.reset();
        }
    });
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Test">
      <SegmentObject capture="{capture}" mask="{mask}"/>
    </BehaviorTree></root>)",
    blackboard_);
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!accepted && std::chrono::steady_clock::now() < deadline) {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(accepted);
  tree.haltTree();
  EXPECT_TRUE(canceled);
}

TEST(PluginRegistration, DirectMotionAndPerceptionRegistered)
{
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(BT_PLUGIN_PATH);
  for (auto id : {"CheckGraspReady", "SelectFeasibleGrasp", "MoveToPregrasp", "ApproachGrasp",
      "AttachObject", "LiftObject", "MoveHome", "MoveObservation", "MovePlace", "OpenGripper",
      "ReleaseObject", "CaptureRgbd", "SegmentObject", "BuildObjectCloud", "InferGrasps",
      "ClearOctomap", "ResetObjectCloudFusion", "AccumulateObjectCloud", "BuildFusedObjectCloud",
      "CaptureJointPose", "MoveJointPose", "CheckJointPoses"})
  {
    EXPECT_EQ(factory.manifests().count(id), 1u);
                                                }
  EXPECT_EQ(factory.manifests().at("MoveToPregrasp").ports.count("plan_id"), 0u);
  EXPECT_EQ(factory.manifests().at("SelectFeasibleGrasp").ports.count("selected_plan"), 1u);
  for (auto id : {"MoveToPregrasp", "ApproachGrasp", "AttachObject", "LiftObject"}) {
    const auto & ports = factory.manifests().at(id).ports;
    EXPECT_EQ(ports.size(), 1u);
    EXPECT_EQ(ports.count("plan"), 1u);
  }
  const auto & observation = factory.manifests().at("MoveObservation").ports;
  EXPECT_EQ(observation.size(), 7u);
  EXPECT_EQ(observation.count("tilt_deg"), 1u);
  EXPECT_EQ(observation.count("plan"), 0u);
}

TEST(PluginRegistration, MultiViewXmlAndIncludedScanTreeLoad)
{
  for (const auto * file : {"demo_multiview.xml", "scan_multiview.xml"}) {
    BT::BehaviorTreeFactory factory;
    factory.registerFromPlugin(BT_PLUGIN_PATH);
    factory.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
    EXPECT_NO_THROW(factory.registerBehaviorTreeFromFile(std::string(BT_TREE_DIR) + "/" + file));
  }
}
