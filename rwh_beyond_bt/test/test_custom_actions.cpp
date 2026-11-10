#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include "gtest/gtest.h"
#include "behaviortree_cpp/bt_factory.h"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"

using FJT = control_msgs::action::FollowJointTrajectory;
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
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](auto handle) {
        auto result = std::make_shared<MoveGroup::Result>();
        result->error_code.val = arm_error_;
        handle->succeed(result);
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

  BT::NodeStatus run(const std::string & node)
  {
    BT::Tree tree;
    try {
      tree = factory_.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\">" + node +
        "</BehaviorTree></root>", blackboard_);
    } catch (const std::exception &) {
      // BT validates malformed literal port values while loading the XML.
      // The navigator rejects such trees before ticking or sending any goal.
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
  ASSERT_EQ(run("<MoveTcp x=\"0.4\" y=\"-0.1\" z=\"0.5\"/>"), BT::NodeStatus::SUCCESS);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & constraints = arm_goal_.request.goal_constraints.at(0);
    const auto & position = constraints.position_constraints.at(0);
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
  blackboard_->set("dz", 0.3);
  ASSERT_EQ(run("<MoveTcp x=\"0\" y=\"0\" z=\"{dz}\" relative=\"true\"/>"), BT::NodeStatus::SUCCESS);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & pose = arm_goal_.request.goal_constraints.at(0).position_constraints.at(0).
      constraint_region.primitive_poses.at(0);
    EXPECT_DOUBLE_EQ(pose.position.x, 0.1);
    EXPECT_DOUBLE_EQ(pose.position.y, 0.2);
    EXPECT_DOUBLE_EQ(pose.position.z, 0.6);
  }
}

TEST_F(CustomActions, InvalidArmInputOrMissingTfNeverSendsGoal)
{
  for (const auto & attrs : {
      "x=\"0\" y=\"0\"", "x=\"nan\" y=\"0\" z=\"0\"",
      "x=\"0\" y=\"inf\" z=\"0\"", "x=\"abc\" y=\"0\" z=\"0\"",
      "x=\"0\" y=\"0\" z=\"0\" frame_id=\"missing_frame\"",
      "x=\"0\" y=\"0\" z=\"0\" relative=\"invalid\""})
  {
    EXPECT_EQ(run(std::string("<MoveTcp ") + attrs + "/>"), BT::NodeStatus::FAILURE) << attrs;
  }
  EXPECT_EQ(arm_goals_, 0);
}

TEST_F(CustomActions, ApplicationErrorsFailEvenWithSucceededRosStatus)
{
  gripper_error_ = FJT::Result::GOAL_TOLERANCE_VIOLATED;
  EXPECT_EQ(run("<SetGripper opening=\"50\"/>"), BT::NodeStatus::FAILURE);
  arm_error_ = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  EXPECT_EQ(run("<MoveTcp x=\"0.4\" y=\"0\" z=\"0.5\"/>"), BT::NodeStatus::FAILURE);
}

TEST_F(CustomActions, AbsoluteFullPoseUsesRequestedOrientationWithoutCurrentTf)
{
  blackboard_->set("tf_buffer", std::shared_ptr<tf2_ros::Buffer>{});
  ASSERT_EQ(run("<MoveTcp x=\"0.4\" y=\"0.1\" z=\"0.5\" "
    "qx=\"0\" qy=\"0\" qz=\"0\" qw=\"1\"/>"), BT::NodeStatus::SUCCESS);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & constraints = arm_goal_.request.goal_constraints.at(0);
  const auto & position = constraints.position_constraints.at(0);
  const auto & pose = position.constraint_region.primitive_poses.at(0);
  const auto & orientation = constraints.orientation_constraints.at(0);
  EXPECT_DOUBLE_EQ(pose.position.x, 0.4);
  EXPECT_DOUBLE_EQ(pose.position.y, 0.1);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.5);
  EXPECT_DOUBLE_EQ(orientation.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(orientation.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(orientation.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(orientation.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(pose.orientation.w, 1.0);
  EXPECT_EQ(orientation.header.frame_id, "base_link");
  EXPECT_EQ(orientation.link_name, "tcp_link");
}

TEST_F(CustomActions, RelativePositionUsesAbsoluteNormalizedBlackboardOrientation)
{
  blackboard_->set("target_qy", 2.0);
  blackboard_->set("target_qw", 2.0);
  ASSERT_EQ(run("<MoveTcp x=\"0\" y=\"0\" z=\"0.3\" relative=\"true\" "
    "qx=\"0\" qy=\"{target_qy}\" qz=\"0\" qw=\"{target_qw}\"/>"), BT::NodeStatus::SUCCESS);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & constraints = arm_goal_.request.goal_constraints.at(0);
  const auto & pose = constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0);
  const auto & q = constraints.orientation_constraints.at(0).orientation;
  EXPECT_DOUBLE_EQ(pose.position.x, 0.1);
  EXPECT_DOUBLE_EQ(pose.position.y, 0.2);
  EXPECT_DOUBLE_EQ(pose.position.z, 0.6);
  EXPECT_DOUBLE_EQ(q.x, 0.0);
  EXPECT_NEAR(q.y, std::sqrt(0.5), 1e-12);
  EXPECT_DOUBLE_EQ(q.z, 0.0);
  EXPECT_NEAR(q.w, std::sqrt(0.5), 1e-12);
  EXPECT_DOUBLE_EQ(pose.orientation.y, q.y);
}

TEST_F(CustomActions, InvalidQuaternionNeverSendsGoal)
{
  for (const auto & attrs : {
      "qx=\"0\"", "qx=\"0\" qy=\"0\" qz=\"0\"",
      "qx=\"0\" qy=\"0\" qz=\"0\" qw=\"0\"",
      "qx=\"0\" qy=\"0\" qz=\"0\" qw=\"1e-15\"",
      "qx=\"nan\" qy=\"0\" qz=\"0\" qw=\"1\"",
      "qx=\"0\" qy=\"inf\" qz=\"0\" qw=\"1\"",
      "qx=\"abc\" qy=\"0\" qz=\"0\" qw=\"1\"",
      "qx=\"0\" qy=\"0\" qz=\"0\" qw=\"{unset_qw}\""})
  {
    EXPECT_EQ(run(std::string("<MoveTcp x=\"0.4\" y=\"0\" z=\"0.5\" ") + attrs + "/>"),
      BT::NodeStatus::FAILURE) << attrs;
  }
  EXPECT_EQ(arm_goals_, 0);
}
