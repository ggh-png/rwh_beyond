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
#include "rwh_beyond_bt/perception_actions.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rwh_beyond_msgs/action/run_demo.hpp"
#include "rwh_beyond_msgs/action/run_grasp_stage.hpp"
#include "rwh_beyond_msgs/action/run_grasp_step.hpp"
#include "rwh_beyond_msgs/msg/tcp_pose.hpp"

using TcpPose = rwh_beyond_msgs::msg::TcpPose;
using GraspStageAction = rwh_beyond_msgs::action::RunGraspStage;
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
  auto select = rclcpp_action::create_server<GraspStageAction>(server_,
    "grasp/select_feasible_grasp",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;}, [&](auto handle) {
        calls.push_back("select");
        auto goal = handle->get_goal();
        EXPECT_EQ(goal->observation_id, capture->id);
        EXPECT_EQ(goal->depth_image, capture->depth);
        EXPECT_EQ(goal->camera_info, capture->info);
        EXPECT_EQ(goal->capture_joints, capture->joints);
        EXPECT_EQ(goal->object_cloud.width, 144u);
        EXPECT_EQ(goal->candidates.size(), 1u);
        auto result = std::make_shared<GraspStageAction::Result>();
        result->success = true;
        result->plan_id = "plan-9";
        result->pregrasp.frame_id = "base_link";
        result->grasp_width = .034;
        result->close_width = .032;
        result->width_source = "point_cloud_estimate";
        handle->succeed(result);
      });
  for (const auto width : {"false", "true"}) {
    EXPECT_EQ(run(std::string(
      R"(<Sequence>
      <SegmentObject capture="{capture}" mask="{mask}"/>
      <BuildObjectCloud capture="{capture}" mask="{mask}" cloud="{cloud}"/>
      <InferGrasps cloud="{cloud}" candidates="{candidates}"/>
      <SelectFeasibleGrasp capture="{capture}" mask="{mask}" observation_id="capture-42"
        cloud="{cloud}" candidates="{candidates}" use_estimated_width=")")
      +
        width +
      R"(" plan_id="{plan}" pregrasp="{pre}" grasp_width="{width}" close_width="{close}"/>
      </Sequence>)"),
      BT::NodeStatus::SUCCESS);
    EXPECT_EQ(blackboard_->get<std::string>("plan"), "plan-9");
    EXPECT_DOUBLE_EQ(blackboard_->get<double>("close"), .032);
  }
  EXPECT_EQ(calls, std::vector<std::string>({"sam", "infer", "select", "sam", "infer", "select"}));
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

TEST_F(BtActions, ObservationXmlOverridesAreDeliveredAndOmissionUsesYaml)
{
  using Step = rwh_beyond_msgs::action::RunGraspStep;
  std::vector<Step::Goal> goals;
  auto server = rclcpp_action::create_server<Step>(server_, "grasp/move_observation",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [&](auto handle) {
        goals.push_back(*handle->get_goal());
        auto result = std::make_shared<Step::Result>();
        result->success = true;
        handle->succeed(result);
      });
  EXPECT_EQ(run(R"(<MoveObservation x="0.03" y="-0.02" z="0.20" tilt_deg="50"/>)"),
    BT::NodeStatus::SUCCESS);
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_TRUE(goals[0].override_observation_z);
  EXPECT_TRUE(goals[0].override_observation_tilt);
  EXPECT_DOUBLE_EQ(goals[0].observation_x_offset, 0.03);
  EXPECT_DOUBLE_EQ(goals[0].observation_y_offset, -0.02);
  EXPECT_DOUBLE_EQ(goals[0].observation_z_offset, 0.2);
  EXPECT_DOUBLE_EQ(goals[0].observation_tilt_deg, 50.0);
  EXPECT_EQ(run(R"(<MoveObservation/>)"), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(goals.size(), 2u);
  EXPECT_FALSE(goals[1].override_observation_z);
  EXPECT_FALSE(goals[1].override_observation_tilt);
}

TEST_F(BtActions, InvalidObservationXmlNeverSendsMotionGoal)
{
  using Step = rwh_beyond_msgs::action::RunGraspStep;
  std::atomic<int> goals{0};
  auto server = rclcpp_action::create_server<Step>(server_, "grasp/move_observation",
      [&](auto, auto) {
        ++goals;
        return rclcpp_action::GoalResponse::REJECT;
      },
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;}, [](auto) {});
  for (const auto * xml : {
    R"(<MoveObservation x="nan"/>)",
    R"(<MoveObservation y="inf" tilt_deg="50"/>)",
    R"(<MoveObservation z="0.2" tilt_deg="nan"/>)"})
  {
    EXPECT_EQ(run(xml), BT::NodeStatus::FAILURE);
  }
  EXPECT_EQ(goals, 0);
}

TEST_F(BtActions, ScenarioNodesForwardCoordinatorCommands)
{
  using Step = rwh_beyond_msgs::action::RunGraspStep;
  std::vector<rclcpp_action::Server<Step>::SharedPtr> servers;
  std::vector<std::string> calls;
  for (const auto & command : {"move_home", "open_gripper", "move_observation"}) {
    servers.push_back(rclcpp_action::create_server<Step>(server_, std::string("grasp/") + command,
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [&, command](auto handle) {
        EXPECT_EQ(handle->get_goal()->command, command);
        calls.push_back(command);
        auto result = std::make_shared<Step::Result>();
        result->success = true;
        handle->succeed(result);
      }));
  }
  EXPECT_EQ(run("<Sequence><MoveHome/><OpenGripper/><MoveObservation/></Sequence>"),
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls, std::vector<std::string>({"move_home", "open_gripper", "move_observation"}));
  EXPECT_EQ(factory_.manifests().count("GraspStep"), 0u);
  EXPECT_EQ(factory_.manifests().at("MoveHome").ports.count("command"), 0u);
}

TEST_F(BtActions, ScenarioFailureStopsSequenceAndReportsNode)
{
  using Step = rwh_beyond_msgs::action::RunGraspStep;
  auto server = rclcpp_action::create_server<Step>(server_, "grasp/move_home",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [](auto handle) {
        auto result = std::make_shared<Step::Result>();
        result->success = false;
        result->message = "Home trajectory rejected";
        handle->succeed(result);
    });
  std::atomic<int> openings{0};
  auto open = rclcpp_action::create_server<Step>(server_, "grasp/open_gripper",
      [&](auto, auto) {++openings; return rclcpp_action::GoalResponse::REJECT;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;}, [](auto) {});
  EXPECT_EQ(run("<Sequence><MoveHome name=\"home\"/><OpenGripper/></Sequence>"),
    BT::NodeStatus::FAILURE);
  const auto diagnostic =
    blackboard_->get<rwh_beyond_msgs::action::RunDemo::Feedback>("demo_feedback");
  EXPECT_EQ(diagnostic.current_stage, "move_home");
  EXPECT_EQ(diagnostic.current_node, "home");
  EXPECT_EQ(diagnostic.error_source, "grasp/move_home");
  EXPECT_EQ(diagnostic.error_msg, "Home trajectory rejected");
  EXPECT_EQ(openings, 0);
}

TEST_F(BtActions, ScenarioServerNameOverrideAndHaltCancelGoal)
{
  using Step = rwh_beyond_msgs::action::RunGraspStep;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Step>> active;
  std::atomic<bool> accepted{false}, canceled{false};
  auto server = rclcpp_action::create_server<Step>(server_, "custom_home",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [&](auto) {canceled = true; return rclcpp_action::CancelResponse::ACCEPT;},
      [&](auto handle) {active = handle; accepted = true;});
  auto timer = server_->create_wall_timer(5ms, [&]() {
        if (active && active->is_canceling()) {
          active->canceled(std::make_shared<Step::Result>());
          active.reset();
        }
  });
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Test"><MoveHome server_name="custom_home"/></BehaviorTree></root>)",
    blackboard_);
  auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!accepted && std::chrono::steady_clock::now() < deadline) {
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(accepted);
  tree.haltTree();
  EXPECT_TRUE(canceled);
}

TEST_F(BtActions, ScenarioXmlValuesReachServerAndDoNotLeakToNextGoal)
{
  using Step = rwh_beyond_msgs::action::RunGraspStep;
  std::vector<rclcpp_action::Server<Step>::SharedPtr> servers;
  std::vector<Step::Goal> goals;
  for (const auto & endpoint : {"move_home", "open_gripper", "move_observation"}) {
    servers.push_back(rclcpp_action::create_server<Step>(server_, std::string("grasp/") + endpoint,
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [&](auto handle) {
        goals.push_back(*handle->get_goal());
        auto result = std::make_shared<Step::Result>();
        result->success = true;
        handle->succeed(result);
      }));
  }
  EXPECT_EQ(run(R"(<MoveHome home_joint_positions="0.1;0;0;0;0;0"/>)"), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_DOUBLE_EQ(goals.back().home_joint_positions.at(0), 0.1);
  EXPECT_EQ(run(R"(<OpenGripper opening="0.08" duration="1.5"/>)"), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(goals.back().override_gripper_width);
  EXPECT_DOUBLE_EQ(goals.back().gripper_open_width, 0.08);
  EXPECT_DOUBLE_EQ(goals.back().gripper_open_duration, 1.5);
  EXPECT_EQ(run(R"(<MoveObservation home_joint_positions="0.1;0;0;0;0;0" settle_time="0"/>)"),
    BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(goals.back().override_settle_time);
  EXPECT_DOUBLE_EQ(goals.back().observation_settle_time, 0.0);
  EXPECT_EQ(run("<MoveHome/>"), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(goals.back().home_joint_positions.empty());
  EXPECT_EQ(run("<OpenGripper/>"), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(goals.back().override_gripper_width);
  EXPECT_FALSE(goals.back().override_gripper_duration);
}

TEST_F(BtActions, InvalidScenarioXmlValuesNeverSendGoal)
{
  using Step = rwh_beyond_msgs::action::RunGraspStep;
  std::atomic<int> goals{0};
  auto server = rclcpp_action::create_server<Step>(server_, "invalid_test",
      [&](auto, auto) {++goals; return rclcpp_action::GoalResponse::REJECT;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;}, [](auto) {});
  for (const auto & xml : {
    R"(<MoveHome server_name="invalid_test" home_joint_positions="0;0;0"/>)",
    R"(<MoveHome server_name="invalid_test" home_joint_positions="nan;0;0;0;0;0"/>)",
    R"(<OpenGripper server_name="invalid_test" opening="0.11"/>)",
    R"(<OpenGripper server_name="invalid_test" duration="0"/>)",
    R"(<MoveObservation server_name="invalid_test" settle_time="-1"/>)"})
  {
    EXPECT_EQ(run(xml), BT::NodeStatus::FAILURE);
  }
  EXPECT_EQ(goals, 0);
}

TEST_F(BtActions, ObservationAllowsIndependentSignedAxesAndTilt)
{
  using Step = rwh_beyond_msgs::action::RunGraspStep;
  std::vector<Step::Goal> goals;
  auto server = rclcpp_action::create_server<Step>(server_, "grasp/move_observation",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [&](auto handle) {
        goals.push_back(*handle->get_goal());
        auto result = std::make_shared<Step::Result>();
        result->success = true;
        handle->succeed(result);
    });
  EXPECT_EQ(run(R"(<MoveObservation x="-0.03"/>)"), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_TRUE(goals.back().override_observation_x);
  EXPECT_FALSE(goals.back().override_observation_z);
  EXPECT_FALSE(goals.back().override_observation_tilt);
  EXPECT_EQ(run(R"(<MoveObservation z="-0.02"/>)"), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(goals.back().observation_z_offset, -0.02);
  EXPECT_EQ(run(R"(<MoveObservation z="0" tilt_deg="0"/>)"), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(goals.back().override_observation_z);
  EXPECT_TRUE(goals.back().override_observation_tilt);
  EXPECT_DOUBLE_EQ(goals.back().observation_z_offset, 0.0);
  EXPECT_DOUBLE_EQ(goals.back().observation_tilt_deg, 0.0);
}

TEST_F(BtActions, RepeatPickReadinessForwardsExplicitReleaseFlag)
{
  bool release_previous = false;
  auto ready = rclcpp_action::create_server<GraspStageAction>(server_, "grasp/check_ready",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [&](auto handle) {
        release_previous = handle->get_goal()->release_previous;
        auto result = std::make_shared<GraspStageAction::Result>();
        result->success = true;
        handle->succeed(result);
    });
  EXPECT_EQ(runXml(
    R"(
    <root BTCPP_format="4"><BehaviorTree ID="Test">
      <CheckGraspReady release_previous="true"/>
    </BehaviorTree></root>)"),
    BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(release_previous);
}

TEST(PluginRegistration, IndividualLibrariesMatchCompatibilityLoader)
{
  const std::vector<std::pair<std::string, std::string>> plugins = {
    {"clear_octomap", "ClearOctomap"}, {"move_home", "MoveHome"},
    {"move_observation", "MoveObservation"},
    {"move_place", "MovePlace"},
    {"open_gripper", "OpenGripper"}, {"release_object", "ReleaseObject"},
    {"check_grasp_ready", "CheckGraspReady"},
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
  individual.registerFromPlugin(std::string(BT_PLUGIN_DIR) + "/" +
    BT::SharedLibrary::getOSName("rwh_beyond_perception_bt_nodes"));
  EXPECT_EQ(individual.manifests().size(), builtin_count + plugins.size() + 4);
  for (const auto * id : {"CaptureRgbd", "SegmentObject", "BuildObjectCloud", "InferGrasps"}) {
    ASSERT_EQ(individual.manifests().count(id), 1u);
  }

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
    if (entry.second != "ClearOctomap") {
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
