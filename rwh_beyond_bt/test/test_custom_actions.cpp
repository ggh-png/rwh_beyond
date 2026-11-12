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
#include <cmath>
#include <limits>
#include <mutex>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "moveit_msgs/action/move_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rwh_beyond_msgs/action/run_demo.hpp"
#include "rwh_beyond_msgs/action/run_grasp_stage.hpp"
#include "rwh_beyond_msgs/action/run_grasp_step.hpp"
#include "rwh_beyond_msgs/msg/tcp_pose.hpp"
#include "tf2_ros/buffer.h"

using FJT = control_msgs::action::FollowJointTrajectory;
using TcpPose = rwh_beyond_msgs::msg::TcpPose;
using MoveGroup = moveit_msgs::action::MoveGroup;
using namespace std::chrono_literals;

class CustomActions : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    server_ = std::make_shared<rclcpp::Node>("custom_action_mock_servers");
    client_ = std::make_shared<rclcpp::Node>("custom_action_bt_client");
    gripper_ = rclcpp_action::create_server<FJT>(
      server_, "gripper_controller/follow_joint_trajectory",
      [this](auto, auto goal) {
        std::lock_guard<std::mutex> lock(mutex_);
        gripper_goal_ = *goal;
        ++gripper_goals_;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto handle) {
        auto result = std::make_shared<FJT::Result>();
        result->error_code = gripper_error_;
        handle->succeed(result);
      });
    arm_ = rclcpp_action::create_server<MoveGroup>(
      server_, "move_action",
      [this](auto, auto goal) {
        std::lock_guard<std::mutex> lock(mutex_);
        arm_goal_ = *goal;
        ++arm_goals_;
        return arm_reject_ ? rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto handle) {
        auto result = std::make_shared<MoveGroup::Result>();
        result->error_code.val = arm_error_;
        result->error_code.source = arm_source_;
        result->error_code.message = arm_message_;
        if (arm_feedback_delay_ms_ > 0) {
          auto feedback = std::make_shared<MoveGroup::Feedback>();
          feedback->state = "MONITOR";
          handle->publish_feedback(feedback);
          std::this_thread::sleep_for(std::chrono::milliseconds(arm_feedback_delay_ms_.load()));
        }
        if (arm_abort_) {handle->abort(result);} else {handle->succeed(result);}
      });
    executor_.add_node(server_);
    spinner_ = std::thread([this]() {executor_.spin();});
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", client_);
    blackboard_->set("bt_loop_duration", 10ms);
    blackboard_->set("server_timeout", 1000ms);
    blackboard_->set("cancel_timeout", 1000ms);
    blackboard_->set("wait_for_service_timeout", 3000ms);
    buffer_ = std::make_shared<tf2_ros::Buffer>(client_->get_clock());
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "tcp_link";
    tf.transform.translation.x = 0.1;
    tf.transform.translation.y = 0.2;
    tf.transform.translation.z = 0.3;
    tf.transform.rotation.z = 0.7071067811865476;
    tf.transform.rotation.w = 0.7071067811865476;
    ASSERT_TRUE(buffer_->setTransform(tf, "test", true));
    blackboard_->set("tf_buffer", buffer_);
    factory_.registerFromPlugin(BT_PLUGIN_PATH);
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spinner_.joinable()) {spinner_.join();}
  }

  static TcpPose makePose(double x = 0.4, double y = 0.0, double z = 0.5)
  {
    TcpPose target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    target.pose.orientation.w = 1.0;
    target.keep_current_orientation = true;
    return target;
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
  rclcpp_action::Server<FJT>::SharedPtr gripper_;
  rclcpp_action::Server<MoveGroup>::SharedPtr arm_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::mutex mutex_;
  FJT::Goal gripper_goal_;
  MoveGroup::Goal arm_goal_;
  std::atomic<int> gripper_goals_{0}, arm_goals_{0};
  std::atomic<int> gripper_error_{FJT::Result::SUCCESSFUL};
  std::string arm_source_, arm_message_;
  std::atomic<bool> arm_abort_{false}, arm_reject_{false};
  std::atomic<int> arm_feedback_delay_ms_{0};
  std::atomic<int> arm_error_{moveit_msgs::msg::MoveItErrorCodes::SUCCESS};
};

TEST_F(CustomActions, GripperConvertsIntegerInputsAndBlackboard)
{
  EXPECT_EQ(factory_.manifests().at("SetGripper").ports.at("unit").defaultValueString(), "mm");
  blackboard_->set("opening", 25);
  ASSERT_EQ(run("<SetGripper opening=\"{opening}\" duration=\"1.25\"/>"), BT::NodeStatus::SUCCESS);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(gripper_goal_.trajectory.joint_names, std::vector<std::string>({"gripper"}));
    ASSERT_EQ(gripper_goal_.trajectory.points.size(), 1u);
    EXPECT_DOUBLE_EQ(gripper_goal_.trajectory.points[0].positions[0], 0.025);
    EXPECT_EQ(gripper_goal_.trajectory.points[0].time_from_start.sec, 1);
    EXPECT_EQ(gripper_goal_.trajectory.points[0].time_from_start.nanosec, 250000000u);
  }
  ASSERT_EQ(run("<SetGripper opening=\"60\" unit=\"mm\"/>"), BT::NodeStatus::SUCCESS);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_DOUBLE_EQ(gripper_goal_.trajectory.points[0].positions[0], 0.06);
  }
  ASSERT_EQ(run("<SetGripper opening=\"50\"/>"), BT::NodeStatus::SUCCESS);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_DOUBLE_EQ(gripper_goal_.trajectory.points[0].positions[0], 0.05);
  }
  EXPECT_EQ(run("<SetGripper opening=\"0\"/>"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(run("<SetGripper opening=\"100\"/>"), BT::NodeStatus::SUCCESS);
}

TEST_F(CustomActions, InvalidGripperInputNeverSendsGoal)
{
  for (const auto & attrs : {
    "", "opening=\"-1\"", "opening=\"101\"", "opening=\"50.5\"",
    "opening=\"abc\"", "opening=\"50\" unit=\"degrees\"",
    "opening=\"50\" duration=\"0\"", "opening=\"50\" duration=\"nan\"",
    "opening=\"50\" duration=\"3601\""})
  {
    EXPECT_EQ(run(std::string("<SetGripper ") + attrs + "/>"), BT::NodeStatus::FAILURE) << attrs;
  }
  EXPECT_EQ(gripper_goals_, 0);
}


TEST_F(CustomActions, ArmAbsoluteAndRelativeGoalsPreserveOrientation)
{
  blackboard_->set("target", makePose(0.4, -0.1, 0.5));
  ASSERT_EQ(run("<MoveTcp pose=\"{target}\"/>"), BT::NodeStatus::SUCCESS);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & position = arm_goal_.request.goal_constraints.at(0).position_constraints.at(0);
    const auto & pose = position.constraint_region.primitive_poses.at(0);
    EXPECT_EQ(position.header.frame_id, "base_link");
    EXPECT_EQ(position.link_name, "tcp_link");
    EXPECT_DOUBLE_EQ(pose.position.x, 0.4);
    EXPECT_DOUBLE_EQ(pose.position.y, -0.1);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.5);
    EXPECT_DOUBLE_EQ(pose.orientation.z, 0.7071067811865476);
    EXPECT_EQ(arm_goal_.request.group_name, "arm");
    EXPECT_FALSE(arm_goal_.planning_options.plan_only);
  }
  auto offset = makePose(0.0, 0.0, 0.3);
  offset.relative = true;
  blackboard_->set("target", offset);
  ASSERT_EQ(run("<MoveTcp pose=\"{target}\"/>"), BT::NodeStatus::SUCCESS);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & pose = arm_goal_.request.goal_constraints.at(0).position_constraints.at(0).
      constraint_region.primitive_poses.at(0);
    EXPECT_DOUBLE_EQ(pose.position.x, 0.1);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.2);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.6);
  }
}

TEST_F(CustomActions, InvalidPoseOrMissingTfNeverSendsGoal)
{
  auto bad = makePose();
  bad.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  blackboard_->set("target", bad);
  EXPECT_EQ(run("<MoveTcp pose=\"{target}\"/>"), BT::NodeStatus::FAILURE);
  bad = makePose();
  bad.pose.position.y = std::numeric_limits<double>::infinity();
  blackboard_->set("target", bad);
  EXPECT_EQ(run("<MoveTcp pose=\"{target}\"/>"), BT::NodeStatus::FAILURE);
  for (const auto & frame : {"", "missing_frame"}) {
    bad = makePose();
    bad.frame_id = frame;
    blackboard_->set("target", bad);
    EXPECT_EQ(run("<MoveTcp pose=\"{target}\"/>"), BT::NodeStatus::FAILURE);
  }
  blackboard_->set("target", makePose());
  EXPECT_EQ(run("<MoveTcp pose=\"{target}\" tcp_link=\"\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(run("<MoveTcp pose=\"{unset_target}\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(run("<MoveTcp/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(run("<MoveTcp x=\"0.4\" y=\"0\" z=\"0.5\"/>"), BT::NodeStatus::FAILURE);
  blackboard_->set("wrong_type", std::string("not a TcpPose"));
  EXPECT_EQ(run("<MoveTcp pose=\"{wrong_type}\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(arm_goals_, 0);
}

TEST_F(CustomActions, ApplicationErrorsFailEvenWithSucceededRosStatus)
{
  gripper_error_ = FJT::Result::GOAL_TOLERANCE_VIOLATED;
  EXPECT_EQ(run("<SetGripper opening=\"50\"/>"), BT::NodeStatus::FAILURE);
  arm_error_ = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  blackboard_->set("target", makePose());
  EXPECT_EQ(run("<MoveTcp pose=\"{target}\"/>"), BT::NodeStatus::FAILURE);
}

TEST_F(CustomActions, AbsoluteFullPoseUsesMessageFrameAndOrientationWithoutCurrentTf)
{
  blackboard_->set("tf_buffer", std::shared_ptr<tf2_ros::Buffer>{});
  auto target = makePose(0.4, 0.1, 0.5);
  target.keep_current_orientation = false;
  target.frame_id = "world";
  blackboard_->set("target", target);
  ASSERT_EQ(run("<MoveTcp pose=\"{target}\" tcp_link=\"tool_tip\"/>"), BT::NodeStatus::SUCCESS);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & constraints = arm_goal_.request.goal_constraints.at(0);
  const auto & pose =
    constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0);
  const auto & orientation = constraints.orientation_constraints.at(0);
  EXPECT_DOUBLE_EQ(pose.position.x, 0.4);
  EXPECT_DOUBLE_EQ(pose.position.y, 0.1);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.5);
  EXPECT_DOUBLE_EQ(orientation.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(orientation.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(orientation.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(orientation.orientation.w, 1.0);
  EXPECT_EQ(orientation.header.frame_id, "world");
  EXPECT_EQ(orientation.link_name, "tool_tip");
}

TEST_F(CustomActions, RelativePositionNormalizesOrientationWithoutMutatingMessage)
{
  auto target = makePose(0, 0, 0.3);
  target.relative = true;
  target.keep_current_orientation = false;
  target.pose.orientation.y = 2.0;
  target.pose.orientation.w = 2.0;
  blackboard_->set("target", target);
  ASSERT_EQ(run("<MoveTcp pose=\"{target}\"/>"), BT::NodeStatus::SUCCESS);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & constraints = arm_goal_.request.goal_constraints.at(0);
  const auto & pose =
    constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0);
  const auto & q = constraints.orientation_constraints.at(0).orientation;
  EXPECT_DOUBLE_EQ(pose.position.x, 0.1);
  EXPECT_DOUBLE_EQ(pose.position.y, 0.2);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.6);
  EXPECT_DOUBLE_EQ(q.x, 0.0);
  EXPECT_NEAR(q.y, std::sqrt(0.5), 1e-12);
  EXPECT_DOUBLE_EQ(q.z, 0.0);
  EXPECT_NEAR(q.w, std::sqrt(0.5), 1e-12);
  const auto unchanged = blackboard_->get<TcpPose>("target");
  EXPECT_EQ(unchanged, target);
}

TEST_F(CustomActions, InvalidQuaternionMessageNeverSendsGoal)
{
  for (const auto w : {0.0, 1e-15, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    auto target = makePose();
    target.keep_current_orientation = false;
    target.pose.orientation.w = w;
    blackboard_->set("target", target);
    EXPECT_EQ(run("<MoveTcp pose=\"{target}\"/>"), BT::NodeStatus::FAILURE);
  }
  EXPECT_EQ(arm_goals_, 0);
}

TEST_F(CustomActions, MakeTcpPosePublishesOneTypedBlackboardObjectWithoutMotion)
{
  ASSERT_EQ(run("<MakeTcpPose x=\"0.25\" y=\"0\" z=\"0.35\" "
    "qx=\"0\" qy=\"0\" qz=\"0\" qw=\"2\" frame_id=\"world\" "
    "pose=\"{target_pose}\"/>"), BT::NodeStatus::SUCCESS);
  const auto target = blackboard_->get<TcpPose>("target_pose");
  EXPECT_EQ(target.frame_id, "world");
  EXPECT_DOUBLE_EQ(target.pose.position.x, 0.25);
  EXPECT_DOUBLE_EQ(target.pose.position.z, 0.35);
  EXPECT_DOUBLE_EQ(target.pose.orientation.w, 1.0);
  EXPECT_FALSE(target.relative);
  EXPECT_FALSE(target.keep_current_orientation);
  EXPECT_EQ(blackboard_->getEntry("target_x"), nullptr);
  EXPECT_EQ(blackboard_->getEntry("target_qw"), nullptr);
  const auto & ports = factory_.manifests().at("MoveTcp").ports;
  EXPECT_EQ(ports.count("pose"), 1u);
  for (const auto * old : {"x", "y", "z", "qx", "qy", "qz", "qw", "frame_id", "relative"}) {
    EXPECT_EQ(ports.count(old), 0u);
  }
  EXPECT_EQ(arm_goals_, 0);
  EXPECT_EQ(gripper_goals_, 0);
}

TEST_F(CustomActions, MakeAndMovePoseSequenceUsesSingleObject)
{
  ASSERT_EQ(run("<Sequence>"
    "<MakeTcpPose x=\"0\" y=\"0\" z=\"0.30\" relative=\"true\" pose=\"{lift_pose}\"/>"
    "<MoveTcp pose=\"{lift_pose}\"/>"
    "</Sequence>"), BT::NodeStatus::SUCCESS);
  const auto target = blackboard_->get<TcpPose>("lift_pose");
  EXPECT_TRUE(target.relative);
  EXPECT_TRUE(target.keep_current_orientation);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & pose = arm_goal_.request.goal_constraints.at(0).position_constraints.at(0).
    constraint_region.primitive_poses.at(0);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.6);
  EXPECT_DOUBLE_EQ(pose.orientation.z, 0.7071067811865476);
}

TEST_F(CustomActions, InvalidBuilderInputDoesNotOverwriteOrMove)
{
  const auto original = makePose();
  blackboard_->set("made_pose", original);
  for (const auto & attrs : {
    "x=\"0\" y=\"0\"", "x=\"nan\" y=\"0\" z=\"0\"",
    "x=\"abc\" y=\"0\" z=\"0\"", "x=\"0\" y=\"0\" z=\"0\" qx=\"0\"",
    "x=\"0\" y=\"0\" z=\"0\" qx=\"0\" qy=\"0\" qz=\"0\" qw=\"0\"",
    "x=\"0\" y=\"0\" z=\"0\" qx=\"0\" qy=\"inf\" qz=\"0\" qw=\"1\"",
    "x=\"0\" y=\"0\" z=\"0\" frame_id=\"\"",
    "x=\"0\" y=\"0\" z=\"0\" relative=\"invalid\""})
  {
    EXPECT_EQ(run(std::string("<Sequence><MakeTcpPose ") + attrs +
      " pose=\"{made_pose}\"/><MoveTcp pose=\"{made_pose}\"/></Sequence>"),
      BT::NodeStatus::FAILURE);
    EXPECT_EQ(blackboard_->get<TcpPose>("made_pose"), original);
  }
  EXPECT_EQ(run("<MakeTcpPose x=\"0\" y=\"0\" z=\"0\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(arm_goals_, 0);
}

TEST_F(CustomActions, TypedPoseCanBeReusedAndUpdated)
{
  blackboard_->set("target", makePose());
  EXPECT_EQ(run("<MoveTcp pose=\"{target}\"/>"), BT::NodeStatus::SUCCESS);
  blackboard_->set("target", makePose(0.2, 0.1, 0.3));
  ASSERT_EQ(run("<MoveTcp pose=\"{target}\"/>"), BT::NodeStatus::SUCCESS);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & pose = arm_goal_.request.goal_constraints.at(0).position_constraints.at(0).
    constraint_region.primitive_poses.at(0);
  EXPECT_DOUBLE_EQ(pose.position.x, 0.2);
  EXPECT_DOUBLE_EQ(pose.position.y, 0.1);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.3);
  EXPECT_EQ(arm_goals_, 2);
}

TEST_F(CustomActions, TypedPosePassesThroughSubtreeRemapping)
{
  ASSERT_EQ(runXml(
    R"(
    <root BTCPP_format="4" main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <Sequence>
          <MakeTcpPose x="0.25" y="0" z="0.35" pose="{target_pose}"/>
          <SubTree ID="Motion" target="{target_pose}" _autoremap="true"/>
        </Sequence>
      </BehaviorTree>
      <BehaviorTree ID="Motion"><MoveTcp pose="{target}"/></BehaviorTree>
    </root>)"),
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(arm_goals_, 1);
}

TEST_F(CustomActions, MoveItAbortPreservesSignedCodeMessageSourceAndStage)
{
  arm_error_ = moveit_msgs::msg::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE;
  arm_abort_ = true;
  arm_source_ = "plan_execution";
  arm_message_ = "Trajectory invalid after scene update";
  arm_feedback_delay_ms_ = 30;
  EXPECT_EQ(run("<MoveHome name=\"return_home\"/>"), BT::NodeStatus::FAILURE);
  const auto diagnostic =
    blackboard_->get<rwh_beyond_msgs::action::RunDemo::Feedback>("demo_feedback");
  EXPECT_EQ(diagnostic.current_stage, "move_home");
  EXPECT_EQ(diagnostic.current_node, "return_home");
  EXPECT_EQ(diagnostic.moveit_state, "MONITOR");
  EXPECT_EQ(diagnostic.error_code, -3);
  EXPECT_EQ(diagnostic.error_name, "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE");
  EXPECT_EQ(diagnostic.error_source, "plan_execution");
  EXPECT_EQ(diagnostic.error_msg, arm_message_);
}

TEST_F(CustomActions, EmptyMoveItMessageHasUsefulFallbackAndNextStageClearsError)
{
  arm_error_ = moveit_msgs::msg::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE;
  arm_abort_ = true;
  EXPECT_EQ(run("<MoveHome/>"), BT::NodeStatus::FAILURE);
  auto diagnostic = blackboard_->get<rwh_beyond_msgs::action::RunDemo::Feedback>("demo_feedback");
  EXPECT_NE(diagnostic.error_msg.find("remaining trajectory became invalid"), std::string::npos);
  EXPECT_NE(diagnostic.error_msg.find("exact contact pairs are not"), std::string::npos);
  arm_error_ = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  arm_abort_ = false;
  EXPECT_EQ(run("<MoveHome/>"), BT::NodeStatus::SUCCESS);
  diagnostic = blackboard_->get<rwh_beyond_msgs::action::RunDemo::Feedback>("demo_feedback");
  EXPECT_EQ(diagnostic.error_code, 0);
  EXPECT_TRUE(diagnostic.error_msg.empty());
  EXPECT_TRUE(diagnostic.moveit_state.empty());
}

TEST_F(CustomActions, FixedArmAndGripperActionsCheckApplicationErrors)
{
  arm_error_ = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  for (const auto * xml : {"<MoveHome/>", "<MoveTcpUp/>"}) {
    EXPECT_EQ(run(xml), BT::NodeStatus::FAILURE);
    EXPECT_EQ(blackboard_->get<rwh_beyond_msgs::action::RunDemo::Feedback>(
      "demo_feedback").error_code, -31);
  }
  gripper_error_ = FJT::Result::GOAL_TOLERANCE_VIOLATED;
  for (const auto * xml : {"<CloseGripper/>", "<OpenGripper/>", "<SetGripper opening=\"50\"/>"}) {
    EXPECT_EQ(run(xml), BT::NodeStatus::FAILURE);
    const auto diagnostic =
      blackboard_->get<rwh_beyond_msgs::action::RunDemo::Feedback>("demo_feedback");
    EXPECT_EQ(diagnostic.error_code, FJT::Result::GOAL_TOLERANCE_VIOLATED);
    EXPECT_EQ(diagnostic.error_name, "GOAL_TOLERANCE_VIOLATED");
  }
}

TEST_F(CustomActions, LocalInputAndRejectedActionHaveNonzeroDiagnostics)
{
  EXPECT_EQ(run("<MoveTcp/>"), BT::NodeStatus::FAILURE);
  auto diagnostic = blackboard_->get<rwh_beyond_msgs::action::RunDemo::Feedback>("demo_feedback");
  EXPECT_EQ(diagnostic.current_stage, "move_tcp");
  EXPECT_EQ(diagnostic.error_name, "INVALID_INPUT");
  EXPECT_NE(diagnostic.error_code, 0);
  arm_reject_ = true;
  EXPECT_EQ(run("<MoveHome/>"), BT::NodeStatus::FAILURE);
  diagnostic = blackboard_->get<rwh_beyond_msgs::action::RunDemo::Feedback>("demo_feedback");
  EXPECT_EQ(diagnostic.error_name, "ACTION_REQUEST_FAILED");
  EXPECT_NE(diagnostic.error_code, 0);
}

TEST_F(CustomActions, FailureInsideSubtreeReachesRootDiagnostics)
{
  arm_abort_ = true;
  arm_error_ = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
  EXPECT_EQ(runXml(
    R"(
    <root BTCPP_format="4" main_tree_to_execute="Main">
      <BehaviorTree ID="Main"><SubTree ID="Motion" _autoremap="true"/></BehaviorTree>
      <BehaviorTree ID="Motion"><MoveHome name="subtree_home"/></BehaviorTree>
    </root>)"),
    BT::NodeStatus::FAILURE);
  const auto diagnostic =
    blackboard_->get<rwh_beyond_msgs::action::RunDemo::Feedback>("demo_feedback");
  EXPECT_EQ(diagnostic.error_code, -4);
  EXPECT_EQ(diagnostic.current_node, "subtree_home");
}

TEST_F(CustomActions, CloseAllowsExpectedBlockedFingerPathButOpenKeepsTracking)
{
  ASSERT_EQ(run("<CloseGripper/>"), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(gripper_goal_.path_tolerance.size(), 1u);
  EXPECT_EQ(gripper_goal_.path_tolerance[0].name, "gripper");
  EXPECT_EQ(gripper_goal_.path_tolerance[0].position, -1.0);
  ASSERT_EQ(run("<OpenGripper/>"), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(gripper_goal_.path_tolerance.empty());
}

TEST_F(CustomActions, ObservationUsesSimulationJointPoseAndMoveItCollisionPlanning)
{
  ASSERT_EQ(run("<MoveObservation/>"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(arm_goal_.request.group_name, "arm");
  ASSERT_EQ(arm_goal_.request.goal_constraints.size(), 1u);
  const auto & joints = arm_goal_.request.goal_constraints[0].joint_constraints;
  ASSERT_EQ(joints.size(), 6u);
  EXPECT_NEAR(joints[1].position, 1.0279353924406198, 1e-12);
  EXPECT_FALSE(arm_goal_.planning_options.plan_only);
}

// Exercise real ROS action transport and Blackboard payloads across the split nodes.
using GraspStageAction = rwh_beyond_msgs::action::RunGraspStage;

TEST_F(CustomActions, SplitGraspStagesPassTypedArtifacts)
{
  std::vector<rclcpp_action::Server<GraspStageAction>::SharedPtr> servers;
  std::vector<std::string> calls;
  const std::vector<std::string> stages = {"segment_object", "build_object_cloud", "infer_grasps",
    "select_feasible_grasp", "move_pregrasp", "approach", "attach", "lift"};
  for (const auto & stage : stages) {
    servers.push_back(rclcpp_action::create_server<GraspStageAction>(server_, "grasp/" + stage,
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [&, stage](auto handle) {
        calls.push_back(stage);
        auto goal = handle->get_goal();
        auto result = std::make_shared<GraspStageAction::Result>();
        result->success = true;
        result->observation_id = "capture-42";
        if (stage == "segment_object") {result->object_mask.width = 37;}
        if (stage == "build_object_cloud") {
          EXPECT_EQ(goal->observation_id, "capture-42");
          EXPECT_EQ(goal->object_mask.width, 37u);
          result->object_cloud.width = 321;
          result->object_cloud.header.frame_id = "camera";
        }
        if (stage == "infer_grasps" || stage == "select_feasible_grasp") {
          EXPECT_EQ(goal->observation_id, "capture-42");
          EXPECT_EQ(goal->object_cloud.width, 321u);
          EXPECT_EQ(goal->object_cloud.header.frame_id, "camera");
        }
        if (stage == "infer_grasps") {
          rwh_beyond_msgs::msg::GraspCandidate candidate;
          candidate.id = "candidate-7";
          result->candidates.push_back(candidate);
        }
        if (stage == "select_feasible_grasp") {
          EXPECT_EQ(goal->candidates.size(), 1u);
          if (!goal->candidates.empty()) {EXPECT_EQ(goal->candidates[0].id, "candidate-7");}
          result->plan_id = "plan-9";
          result->pregrasp.frame_id = "base_link";
        }
        if (stage == "move_pregrasp" || stage == "approach" || stage == "attach" ||
        stage == "lift")
        {
          EXPECT_EQ(goal->plan_id, "plan-9");
        }
        handle->succeed(result);
      }));
  }
  EXPECT_EQ(runXml(
    R"(
    <root BTCPP_format="4"><BehaviorTree ID="Test"><Sequence>
      <SegmentObject observation_id="{obs}" mask="{mask}"/>
      <BuildObjectCloud observation_id="{obs}" mask="{mask}" cloud="{cloud}"/>
      <InferGrasps observation_id="{obs}" cloud="{cloud}" candidates="{candidates}"/>
      <SelectFeasibleGrasp observation_id="{obs}" cloud="{cloud}" candidates="{candidates}" plan_id="{plan}" pregrasp="{pre}"/>
      <MoveToPregrasp plan_id="{plan}"/><ApproachGrasp plan_id="{plan}"/>
      <AttachObject plan_id="{plan}"/><LiftObject plan_id="{plan}"/>
    </Sequence></BehaviorTree></root>)"),
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls, stages);
  EXPECT_EQ(blackboard_->get<TcpPose>("pre").frame_id, "base_link");
}

TEST_F(CustomActions, SplitGraspStageFailureStopsSequence)
{
  int moves = 0;
  auto infer = rclcpp_action::create_server<GraspStageAction>(server_, "grasp/infer_grasps",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [](auto handle) {
        auto result = std::make_shared<GraspStageAction::Result>();
        result->message = "inference failed";
        handle->abort(result);
    });
  auto move = rclcpp_action::create_server<GraspStageAction>(server_, "grasp/move_pregrasp",
      [&](auto, auto) {++moves; return rclcpp_action::GoalResponse::REJECT;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;}, [](auto) {});
  blackboard_->set("obs", std::string("capture"));
  blackboard_->set("cloud", sensor_msgs::msg::PointCloud2());
  EXPECT_EQ(runXml(
    R"(
    <root BTCPP_format="4"><BehaviorTree ID="Test"><Sequence>
      <InferGrasps observation_id="{obs}" cloud="{cloud}" candidates="{candidates}"/>
      <MoveToPregrasp plan_id="unused"/>
    </Sequence></BehaviorTree></root>)"),
    BT::NodeStatus::FAILURE);
  EXPECT_EQ(moves, 0);
}

TEST_F(CustomActions, ObservationXmlOverridesAreDeliveredAndOmissionUsesYaml)
{
  using Step = rwh_beyond_msgs::action::RunGraspStep;
  std::vector<Step::Goal> goals;
  auto server = rclcpp_action::create_server<Step>(server_, "grasp_step",
      [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [&](auto handle) {
        goals.push_back(*handle->get_goal());
        auto result = std::make_shared<Step::Result>();
        result->success = true;
        handle->succeed(result);
      });
  EXPECT_EQ(run(R"(<GraspStep command="move_observation" rise_m="0.20" tilt_deg="50"/>)"),
    BT::NodeStatus::SUCCESS);
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_TRUE(goals[0].override_observation);
  EXPECT_DOUBLE_EQ(goals[0].observation_z_offset, 0.2);
  EXPECT_DOUBLE_EQ(goals[0].observation_tilt_deg, 50.0);
  EXPECT_EQ(run(R"(<GraspStep command="move_observation"/>)"), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(goals.size(), 2u);
  EXPECT_FALSE(goals[1].override_observation);
}

TEST_F(CustomActions, InvalidObservationXmlNeverSendsMotionGoal)
{
  using Step = rwh_beyond_msgs::action::RunGraspStep;
  std::atomic<int> goals{0};
  auto server = rclcpp_action::create_server<Step>(server_, "grasp_step",
      [&](auto, auto) {
        ++goals;
        return rclcpp_action::GoalResponse::REJECT;
      },
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;}, [](auto) {});
  for (const auto * xml : {
      R"(<GraspStep command="move_observation" rise_m="0.2"/>)",
      R"(<GraspStep command="move_observation" rise_m="-0.2" tilt_deg="50"/>)",
      R"(<GraspStep command="move_observation" rise_m="0.2" tilt_deg="nan"/>)"})
  {
    EXPECT_EQ(run(xml), BT::NodeStatus::FAILURE);
  }
  EXPECT_EQ(goals, 0);
}
