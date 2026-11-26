// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "gtest/gtest.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rwh_beyond_bt/motion_context.hpp"
#include "rwh_beyond_bt/joint_pose_store.hpp"
#include "rwh_beyond_bt/diagnostics.hpp"

using namespace std::chrono_literals;
using namespace rwh_beyond_bt;  // NOLINT(build/namespaces)

class DirectMotion : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    server = std::make_shared<rclcpp::Node>("move_group");
    std::string description = "<robot name=\"test_arm\"><link name=\"link0\"/>";
    for (int index = 1; index <= 6; ++index) {
      auto parent = std::to_string(index - 1);
      auto joint = std::to_string(index);
      description += "<link name=\"link" + joint + "\"/><joint name=\"joint" + joint +
        "\" type=\"revolute\"><parent link=\"link" + parent + "\"/><child link=\"link" +
        joint + "\"/><limit lower=\"-3\" upper=\"3\" effort=\"1\" velocity=\"1\"/></joint>";
    }
    server->declare_parameter("robot_description", description + "</robot>");
    client = std::make_shared<rclcpp::Node>("motion_bt_test");
    client->declare_parameter("octomap_enabled", false);
    board = BT::Blackboard::create(); board->set("node", client);
    factory.registerFromPlugin(BT_PLUGIN_PATH);
    joints.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"};
    joints.position.assign(7, 0);
    publisher = server->create_publisher<sensor_msgs::msg::JointState>("feedback/joint_states", 10);
    timer = server->create_wall_timer(10ms, [this]() {
          joints.header.stamp = server->now(); publisher->publish(joints);
    });
    scene_server = server->create_service<moveit_msgs::srv::GetPlanningScene>("get_planning_scene",
        [this](moveit_msgs::srv::GetPlanningScene::Request::SharedPtr request,
        moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response) {
          response->scene = world;
      });
    apply_server =
      server->create_service<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene",
        [this](moveit_msgs::srv::ApplyPlanningScene::Request::SharedPtr request,
        moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr response) {
          ++applies;
          world.allowed_collision_matrix = request->scene.allowed_collision_matrix;
          for (const auto & object : request->scene.world.collision_objects) {
            auto & objects = world.world.collision_objects;
            objects.erase(std::remove_if(objects.begin(), objects.end(), [&](const auto & entry) {
                return entry.id == object.id;
              }), objects.end());
            if (object.operation != moveit_msgs::msg::CollisionObject::REMOVE) {
              objects.push_back(object);
            }
          }
          if (!request->scene.robot_state.attached_collision_objects.empty()) {
            if (request->scene.robot_state.attached_collision_objects[0].object.operation == 1) {
              world.robot_state.attached_collision_objects.clear();
            } else {
              world.robot_state.attached_collision_objects =
              request->scene.robot_state.attached_collision_objects;
            }
          }
          response->success = true;
      });
    fk_server = server->create_service<moveit_msgs::srv::GetPositionFK>("compute_fk",
        [](moveit_msgs::srv::GetPositionFK::Request::SharedPtr,
        moveit_msgs::srv::GetPositionFK::Response::SharedPtr response) {
          response->error_code.val = 1;
          geometry_msgs::msg::PoseStamped pose; pose.pose.orientation.x = 1;
          pose.pose.orientation.w = 0; pose.pose.position.z = .5;
          response->pose_stamped = {pose};
      });
    ik_server = server->create_service<moveit_msgs::srv::GetPositionIK>("compute_ik",
        [this](moveit_msgs::srv::GetPositionIK::Request::SharedPtr request,
        moveit_msgs::srv::GetPositionIK::Response::SharedPtr response) {
          EXPECT_TRUE(request->ik_request.avoid_collisions);
          EXPECT_EQ(request->ik_request.ik_link_name, "tcp_link");
          last_ik_goal = request->ik_request.pose_stamped.pose;
          last_ik_timeout = rclcpp::Duration(request->ik_request.timeout).seconds();
          ++iks; response->error_code.val = ik_error;
          response->solution = request->ik_request.robot_state;
          response->solution.joint_state.position[5] = .2;
      });
    cart_server =
      server->create_service<moveit_msgs::srv::GetCartesianPath>("compute_cartesian_path",
        [this](moveit_msgs::srv::GetCartesianPath::Request::SharedPtr request,
        moveit_msgs::srv::GetCartesianPath::Response::SharedPtr response) {
          ++cartesians; EXPECT_TRUE(request->avoid_collisions);
          if (!request->waypoints.empty()) {last_cartesian_goal = request->waypoints.back();}
          EXPECT_DOUBLE_EQ(request->revolute_jump_threshold, .35);
          response->error_code.val = 1; response->fraction = fraction;
          auto end = request->start_state;
          end.joint_state.position[5] += .1;
          response->solution = trajectory(request->start_state, end);
      });
    plan_server = server->create_service<moveit_msgs::srv::GetMotionPlan>("plan_kinematic_path",
        [this](moveit_msgs::srv::GetMotionPlan::Request::SharedPtr request,
        moveit_msgs::srv::GetMotionPlan::Response::SharedPtr response) {
          ++plans;
          EXPECT_EQ(request->motion_plan_request.planner_id, expected_planner);
          last_attempts = request->motion_plan_request.num_planning_attempts;
          last_planning_time = request->motion_plan_request.allowed_planning_time;
          last_constraints = request->motion_plan_request.path_constraints;
          auto goal = request->motion_plan_request.start_state;
          auto & constraints = request->motion_plan_request.goal_constraints[0].joint_constraints;
          for (auto & c : constraints) {
            for (size_t i = 0; i < goal.joint_state.name.size(); ++i) {
              if (goal.joint_state.name[i] == c.joint_name) {
                goal.joint_state.position[i] = c.position;
              }
            }
          }
          if (wrong_goal) {goal.joint_state.position[5] += .5;}
          response->motion_plan_response.error_code.val = 1;
          response->motion_plan_response.trajectory =
          trajectory(request->motion_plan_request.start_state, goal);
      });
    execution = rclcpp_action::create_server<moveit_msgs::action::ExecuteTrajectory>(server,
      "execute_trajectory",
        [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
        [this](auto) {canceled = true; return rclcpp_action::CancelResponse::ACCEPT;},
        [this](auto handle) {
          ++executions;
          if (hold_execution) {held = handle; return;}
          auto & t = handle->get_goal()->trajectory.joint_trajectory;
          for (size_t i = 0; i < t.joint_names.size(); ++i) {
            for (size_t j = 0; j < joints.name.size(); ++j) {
              if (joints.name[j] == t.joint_names[i]) {
                joints.position[j] = t.points.back().positions[i];
              }
            }
          }
          auto result = std::make_shared<moveit_msgs::action::ExecuteTrajectory::Result>();
          result->error_code.val = execution_error;
          result->error_code.message = "native execution detail";
          if (execution_error == 1) {handle->succeed(result);} else {handle->abort(result);}
      });
    gripper = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(server,
      "gripper_controller/follow_joint_trajectory",
        [](auto, auto) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
        [](auto) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](auto handle) {
          ++closes;
          close_marker = !handle->get_goal()->path_tolerance.empty();
          joints.position[6] = handle->get_goal()->trajectory.points[0].positions[0];
          auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
          if (fail_close) {result->error_code = -1; handle->abort(result);} else {
            handle->succeed(result);
          }
      });
    executor.add_node(server); spinner = std::thread([this]() {executor.spin();});
    context = MotionContext::get(board);
    pose_directory = std::filesystem::temp_directory_path() / ("bt-joint-poses-" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    pose_file = (pose_directory / "poses.yaml").string();
    client->declare_parameter("bt_motion.joint_poses_file", pose_file);
    auto cancel = std::make_shared<std::atomic_bool>(false);
    context->wait([&]() {
        try {context->feedback(); return true;} catch (...) {
          return false;
        }
         }, 3, cancel);
  }
  void TearDown() override
  {
    board.reset(); context.reset(); executor.cancel(); spinner.join();
    std::filesystem::remove_all(pose_directory);
  }
  void use_verified_physical_transport()
  {
    context->simulation = false;
    hardware_health = server->create_service<std_srvs::srv::Trigger>(
      "grasp/hardware_transport_ready",
      [](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = true;
        response->message = "physical_feedback_transport";
      });
  }
  static Trajectory trajectory(const State & start, const State & end)
  {
    Trajectory result;
    result.joint_trajectory.joint_names.assign(start.joint_state.name.begin(),
      start.joint_state.name.begin() + 6);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(start.joint_state.position.begin(),
      start.joint_state.position.begin() + 6);
    result.joint_trajectory.points.push_back(point);
    point.positions.assign(end.joint_state.position.begin(), end.joint_state.position.begin() + 6);
    point.time_from_start.sec = 1; result.joint_trajectory.points.push_back(point);
    return result;
  }
  BT::Tree tree(const std::string & xml)
  {
    return factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\">" + xml +
      "</BehaviorTree></root>", board);
  }
  BT::NodeStatus run(const std::string & xml)
  {
    auto t = tree(xml);
    auto status = BT::NodeStatus::RUNNING;
    auto deadline = std::chrono::steady_clock::now() + 8s;
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
      status = t.tickOnce(); std::this_thread::sleep_for(5ms);
    }
    t.haltTree(); return status;
  }
  void capture()
  {
    auto obs = std::make_shared<RgbdObservation>(); obs->id = "capture";
    obs->depth.header.frame_id = "base_link"; obs->depth.header.stamp = client->now();
    obs->depth.width = obs->depth.height = 32; obs->depth.encoding = "32FC1"; obs->depth.step = 128;
    obs->depth.data.resize(32 * 128);
    obs->info.header = obs->depth.header; obs->info.width = obs->info.height = 32;
    obs->info.k = {100, 0, 16, 0, 100, 16, 0, 0, 1};
    obs->joints = context->state(std::make_shared<std::atomic_bool>(false)).joint_state;
    sensor_msgs::msg::Image mask;
    mask.header = obs->depth.header; mask.width = mask.height = 32; mask.encoding = "mono8";
    mask.step = 32;
    mask.data.resize(1024);
    for (int v = 0; v < 32; ++v) {
      for (int u = 0; u < 32; ++u) {
        bool inside = v >= 8 && v < 24 && u >= 8 && u < 24;
        float depth = inside ? .45f : .4f;
        std::memcpy(&obs->depth.data[v * 128 + u * 4], &depth, 4); mask.data[v * 32 + u] = inside;
      }
    }
    rwh_beyond_msgs::msg::GraspCandidate candidate;
    candidate.pose.header = obs->depth.header; candidate.pose.pose.position.z = .55;
    candidate.pose.pose.orientation.x = 1; candidate.pose.pose.orientation.w = 0;
    candidate.score = .9;
    board->set("capture", Observation(obs)); board->set("mask", mask);
    board->set("cloud", project_object_cloud(*obs, mask));
    board->set("candidates", std::vector<rwh_beyond_msgs::msg::GraspCandidate>{candidate});
  }
  std::string selection =
    "<SelectFeasibleGrasp capture=\"{capture}\" mask=\"{mask}\" cloud=\"{cloud}\""
    " candidates=\"{candidates}\" use_estimated_width=\"true\""
    " selected_plan=\"{plan}\"/>";
  rclcpp::Node::SharedPtr server, client;
  BT::Blackboard::Ptr board;
  BT::BehaviorTreeFactory factory;
  std::shared_ptr<MotionContext> context;
  sensor_msgs::msg::JointState joints;
  moveit_msgs::msg::PlanningScene world;
  std::filesystem::path pose_directory;
  std::string pose_file;
  Constraints last_constraints;
  geometry_msgs::msg::Pose last_cartesian_goal, last_ik_goal;
  std::string expected_planner{"RRTConnect"};
  int last_attempts{0}, ik_error{1};
  double last_planning_time{0}, last_ik_timeout{0};
  double fraction{1};
  int32_t execution_error{1};
  bool wrong_goal{false}, hold_execution{false}, fail_close{false};
  std::atomic_bool canceled{false};
  std::atomic_int executions{0}, closes{0}, applies{0}, iks{0}, cartesians{0}, plans{0};
  bool close_marker{false};
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spinner;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hardware_health;
  rclcpp::Service<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_server;
  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_server;
  rclcpp::Service<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_server;
  rclcpp::Service<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_server;
  rclcpp::Service<moveit_msgs::srv::GetCartesianPath>::SharedPtr cart_server;
  rclcpp::Service<moveit_msgs::srv::GetMotionPlan>::SharedPtr plan_server;
  rclcpp_action::Server<moveit_msgs::action::ExecuteTrajectory>::SharedPtr execution;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::ExecuteTrajectory>> held;
  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr gripper;
};

TEST_F(DirectMotion, SelectPlanApproachCloseAttachAndLiftCallMoveItDirectly)
{
  capture();
  EXPECT_EQ(run("<Sequence>" + selection +
    "<MoveToPregrasp plan=\"{plan}\"/><ApproachGrasp plan=\"{plan}\"/>"
    "<AttachObject plan=\"{plan}\"/><LiftObject plan=\"{plan}\"/></Sequence>"),
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(executions, 3); EXPECT_EQ(closes, 1); EXPECT_TRUE(close_marker);
  EXPECT_TRUE(context->attached); EXPECT_EQ(last_constraints.joint_constraints.size(), 2u);
  ASSERT_EQ(world.robot_state.attached_collision_objects.size(), 1u);
  EXPECT_EQ(world.robot_state.attached_collision_objects[0].touch_links.size(), 3u);
  auto plan = board->get<Plan>("plan");
  EXPECT_NEAR(plan->grasp.position.z, .42, 1e-6);
  EXPECT_NEAR(plan->pregrasp.position.z, .52, 1e-6);
  EXPECT_GT(plan->observed_width, plan->close_width);
}
TEST_F(DirectMotion, ZeroPregraspPlansDirectlyAndClosesWithoutAnApproachTrajectory)
{
  capture();
  client->declare_parameter("bt_motion.pregrasp_distance", 0.0);
  auto xml = selection;
  xml.insert(xml.find("/>"), " pregrasp_distance=\"0.0\"");
  ASSERT_EQ(run("<Sequence>" + xml +
    "<MoveToPregrasp plan=\"{plan}\"/><ApproachGrasp plan=\"{plan}\"/>"
    "<AttachObject plan=\"{plan}\"/></Sequence>"), BT::NodeStatus::SUCCESS);
  auto plan = board->get<Plan>("plan");
  EXPECT_FALSE(plan->has_approach);
  EXPECT_NEAR(plan->pregrasp.position.z, plan->grasp.position.z, 1e-6);
  EXPECT_EQ(cartesians, 1);  // The lift feasibility check still runs.
  EXPECT_EQ(executions, 1);
  EXPECT_EQ(closes, 1);
  EXPECT_TRUE(context->attached);
}

TEST_F(DirectMotion, CarryHomePlaceReleaseAndReturnUseDirectClients)
{
  capture();
  EXPECT_EQ(run("<Sequence>" + selection +
    "<MoveToPregrasp plan=\"{plan}\"/><ApproachGrasp plan=\"{plan}\"/>"
    "<AttachObject plan=\"{plan}\"/><LiftObject plan=\"{plan}\"/>"
    "<MoveHome/><MovePlace x=\"0.20\" z=\"0.15\"/><ReleaseObject/><MoveHome/>"
    "</Sequence>"), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(context->attached);
  EXPECT_EQ(closes, 2);
  EXPECT_FALSE(context->scene_active);
}
TEST_F(DirectMotion, ObservationUsesSignedHomeRelativeOffsets)
{
  EXPECT_EQ(run("<Sequence><MoveHome home_joint_positions=\".1;.1;.1;.1;.1;.1\"/>"
    "<MoveObservation home_joint_positions=\".1;.1;.1;.1;.1;.1\" "
    "x=\"-.02\" y=\"-.01\" z=\"-.05\" tilt_deg=\"-20\" settle_time=\"0\"/>"
    "</Sequence>"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(executions, 2);
  EXPECT_EQ(plans, 2);
}
TEST_F(DirectMotion, PhysicalModeRefusesAnUnverifiedTransport)
{
  context->simulation = false;
  auto health = server->create_service<std_srvs::srv::Trigger>("grasp/hardware_transport_ready",
      [](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = true; response->message = "mock_transport";
    });
  EXPECT_EQ(run("<OpenGripper/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(closes, 0);
  EXPECT_NE(execution_diagnostic(board).error_msg.find("Physical transport"), std::string::npos);
}
TEST_F(DirectMotion, RejectedGoalAndPartialCartesianNeverExecute)
{
  capture(); fraction = .5;
  EXPECT_EQ(run(selection), BT::NodeStatus::FAILURE); EXPECT_EQ(executions, 0);
  EXPECT_FALSE(context->scene_active);
  capture(); fraction = 1; wrong_goal = true;
  EXPECT_EQ(run(selection), BT::NodeStatus::FAILURE); EXPECT_EQ(executions, 0);
  EXPECT_FALSE(context->scene_active);
}
TEST_F(DirectMotion, MovedArmAndReorderedPlanNeverExecute)
{
  capture();
  EXPECT_EQ(run(selection), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(run("<ApproachGrasp plan=\"{plan}\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(executions, 0);
  capture(); EXPECT_EQ(run(selection), BT::NodeStatus::SUCCESS);
  auto old = std::const_pointer_cast<RgbdObservation>(board->get<Observation>("capture"));
  old->joints.position[0] = .1;
  EXPECT_EQ(run("<MoveToPregrasp plan=\"{plan}\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(executions, 0);
}
TEST_F(DirectMotion, PhysicalInterruptedCloseRetainsPayloadAndRefusesAutomaticOpening)
{
  use_verified_physical_transport();
  capture(); fail_close = true;
  EXPECT_EQ(run("<Sequence>" + selection +
    "<MoveToPregrasp plan=\"{plan}\"/><ApproachGrasp plan=\"{plan}\"/>"
    "<AttachObject plan=\"{plan}\"/></Sequence>"),
    BT::NodeStatus::FAILURE);
  EXPECT_TRUE(context->attached); EXPECT_TRUE(context->scene_active);
  EXPECT_EQ(run("<OpenGripper/>"), BT::NodeStatus::FAILURE); EXPECT_EQ(closes, 1);
}
TEST_F(DirectMotion, SimulationCanOpenAndRetryAfterInterruptedClose)
{
  capture(); fail_close = true;
  ASSERT_EQ(run("<Sequence>" + selection +
    "<MoveToPregrasp plan=\"{plan}\"/><ApproachGrasp plan=\"{plan}\"/>"
    "<AttachObject plan=\"{plan}\"/></Sequence>"), BT::NodeStatus::FAILURE);
  ASSERT_TRUE(context->attached);
  fail_close = false;
  EXPECT_EQ(run("<OpenGripper/>"), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(context->attached);
  EXPECT_FALSE(context->scene_active);
}

TEST_F(DirectMotion, SimulationResetRemovesOldPlanningSceneAttachment)
{
  capture();
  ASSERT_EQ(run("<Sequence>" + selection +
    "<MoveToPregrasp plan=\"{plan}\"/><ApproachGrasp plan=\"{plan}\"/>"
    "<AttachObject plan=\"{plan}\"/></Sequence>"), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(context->attached);
  auto reset = server->create_publisher<std_msgs::msg::UInt64>(
    "simulation/reset_id", rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::UInt64 event;
  event.data = 1;
  reset->publish(event);
  auto cancel = std::make_shared<std::atomic_bool>(false);
  ASSERT_NO_THROW(context->wait([&]() {
    context->prepare_motion(cancel);
    return !context->attached;
  }, 2, cancel));
  EXPECT_FALSE(context->scene_active);
  EXPECT_TRUE(world.robot_state.attached_collision_objects.empty());
  EXPECT_TRUE(world.world.collision_objects.empty());
}

TEST_F(DirectMotion, HaltCancelsTheMoveItExecutionGoal)
{
  capture(); ASSERT_EQ(run(selection), BT::NodeStatus::SUCCESS);
  hold_execution = true;
  auto t = tree("<MoveToPregrasp plan=\"{plan}\"/>");
  for (int i = 0; i < 400 && executions == 0; ++i) {
    t.tickOnce(); std::this_thread::sleep_for(5ms);
                                                                                                   }
  ASSERT_EQ(executions, 1); t.haltTree();
  for (int i = 0; i < 100 && !canceled; ++i) {
    std::this_thread::sleep_for(5ms);
                                                                               }
  EXPECT_TRUE(canceled); EXPECT_FALSE(context->scene_active);
}
TEST_F(DirectMotion, DestroyingRunningMotionCancelsTheMoveItGoal)
{
  capture();
  ASSERT_EQ(run(selection), BT::NodeStatus::SUCCESS);
  hold_execution = true;
  {
    auto t = tree("<MoveToPregrasp plan=\"{plan}\"/>");
    for (int i = 0; i < 400 && executions == 0; ++i) {
      t.tickOnce();
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_EQ(executions, 1);
  }
  for (int i = 0; i < 100 && !canceled; ++i) {
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(canceled);
}
TEST_F(DirectMotion, MaximumInsetIsAcceptedAndAppliedToEstimatedWidth)
{
  client->declare_parameter("bt_motion.grasp_width_closing_inset", .02);
  capture();
  ASSERT_EQ(run(selection), BT::NodeStatus::SUCCESS);
  auto plan = board->get<Plan>("plan");
  EXPECT_NEAR(plan->observed_width - plan->close_width, .02, 1e-6);
}
TEST_F(DirectMotion, InvalidInsetFailsBeforePlanningWithClearMessage)
{
  client->declare_parameter("bt_motion.grasp_width_closing_inset", .020001); capture();
  EXPECT_EQ(run(selection), BT::NodeStatus::FAILURE);
  EXPECT_NE(execution_diagnostic(board).error_msg.find("grasp_width_closing_inset"),
    std::string::npos);
  EXPECT_EQ(iks, 0); EXPECT_EQ(plans, 0);
}
TEST(MotionGeometry, RejectsWristViolationAndWrongEndpoint)
{
  State a; a.joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  a.joint_state.position.assign(6, 0);
  Trajectory t; t.joint_trajectory.joint_names = a.joint_state.name;
  trajectory_msgs::msg::JointTrajectoryPoint p; p.positions.assign(6, 0); p.time_from_start.sec = 1;
  t.joint_trajectory.points = {p};
  EXPECT_TRUE(trajectory_reaches(t, a));
  Constraints wrist; moveit_msgs::msg::JointConstraint c;
  c.joint_name = "joint6"; c.tolerance_above = c.tolerance_below = .1;
    wrist.joint_constraints = {c};
  t.joint_trajectory.points[0].positions[5] = .2;
  EXPECT_FALSE(trajectory_valid(t, wrist)); EXPECT_FALSE(trajectory_reaches(t, a));
}

TEST_F(DirectMotion, ObservationCanStartAwayFromHomeWithoutAnImplicitHomeMove)
{
  ASSERT_EQ(run("<MoveObservation settle_time=\"0\"/>"), BT::NodeStatus::SUCCESS);
  // The first Cartesian moves joint6 in this mock, so the second starts away from home.
  EXPECT_EQ(run("<MoveObservation settle_time=\"0\"/>"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(plans, 2);
  EXPECT_EQ(cartesians, 0);
  EXPECT_EQ(executions, 2);
}
TEST_F(DirectMotion, PlaceCanStartAfterLiftWithoutAHomeNode)
{
  capture();
  EXPECT_EQ(run("<Sequence>" + selection +
    "<MoveToPregrasp plan=\"{plan}\"/><ApproachGrasp plan=\"{plan}\"/>"
    "<AttachObject plan=\"{plan}\"/><LiftObject plan=\"{plan}\"/>"
    "<MovePlace x=\"0.20\" z=\"0.15\"/><ReleaseObject/>"
    "</Sequence>"), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(context->attached);
}
TEST_F(DirectMotion, ReleaseCanOpenAnEmptyGripperWhenAllowed)
{
  EXPECT_EQ(run("<ReleaseObject allow_empty=\"true\"/>"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(closes, 1);
  EXPECT_FALSE(context->attached);
}
TEST_F(DirectMotion, SimulationCanPlaceAndReleaseWithoutAnAttachment)
{
  EXPECT_EQ(run("<Sequence><MovePlace x=\"0.20\" z=\"0.15\"/><ReleaseObject/></Sequence>"),
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(closes, 1);
  EXPECT_FALSE(context->attached);
}

TEST_F(DirectMotion, SavedPlacementPosePermitsReleaseOnlyWhenExplicitlyEnabled)
{
  capture();
  ASSERT_EQ(run("<Sequence>" + selection +
    "<MoveToPregrasp plan=\"{plan}\"/><ApproachGrasp plan=\"{plan}\"/>"
    "<AttachObject plan=\"{plan}\"/><LiftObject plan=\"{plan}\"/></Sequence>"),
    BT::NodeStatus::SUCCESS);
  use_verified_physical_transport();
  save_joint_pose(pose_file, "place", std::vector<double>(6, .1));
  ASSERT_EQ(run("<MoveJointPose pose_name=\"place\" settle_time=\"0\"/>"), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(context->release_ready);
  EXPECT_EQ(run("<ReleaseObject opening=\"0.10\" allow_empty=\"true\"/>"),
    BT::NodeStatus::FAILURE);
  ASSERT_EQ(run("<MoveJointPose pose_name=\"place\" settle_time=\"0\""
    " release_on_arrival=\"true\"/>"), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(context->release_ready);
  EXPECT_EQ(run("<ReleaseObject opening=\"0.10\" allow_empty=\"true\"/>"),
    BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(context->attached);
}
TEST_F(DirectMotion, FailedSavedPlacementPoseCannotAuthorizeRelease)
{
  capture();
  ASSERT_EQ(run("<Sequence>" + selection +
    "<MoveToPregrasp plan=\"{plan}\"/><ApproachGrasp plan=\"{plan}\"/>"
    "<AttachObject plan=\"{plan}\"/><LiftObject plan=\"{plan}\"/></Sequence>"),
    BT::NodeStatus::SUCCESS);
  use_verified_physical_transport();
  save_joint_pose(pose_file, "place", std::vector<double>(6, .1));
  context->release_ready = true;
  wrong_goal = true;
  EXPECT_EQ(run("<MoveJointPose pose_name=\"place\" settle_time=\"0\""
    " release_on_arrival=\"true\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(context->release_ready);
  EXPECT_EQ(run("<ReleaseObject allow_empty=\"true\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(context->attached);
}
TEST_F(DirectMotion, InvalidSavedPlacementClearsPreviousReleasePermission)
{
  context->release_ready = true;
  EXPECT_EQ(run("<MoveJointPose pose_name=\"missing\" release_on_arrival=\"true\"/>"),
    BT::NodeStatus::FAILURE);
  EXPECT_FALSE(context->release_ready);
}
TEST_F(DirectMotion, CanceledSavedPlacementCannotAuthorizeRelease)
{
  capture();
  ASSERT_EQ(run("<Sequence>" + selection +
    "<MoveToPregrasp plan=\"{plan}\"/><ApproachGrasp plan=\"{plan}\"/>"
    "<AttachObject plan=\"{plan}\"/><LiftObject plan=\"{plan}\"/></Sequence>"),
    BT::NodeStatus::SUCCESS);
  save_joint_pose(pose_file, "place", std::vector<double>(6, .1));
  const auto previous_executions = executions.load();
  hold_execution = true;
  auto t = tree("<MoveJointPose pose_name=\"place\" settle_time=\"0\""
    " release_on_arrival=\"true\"/>");
  for (int i = 0; i < 400 && executions == previous_executions; ++i) {
    t.tickOnce();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_GT(executions.load(), previous_executions);
  t.haltTree();
  EXPECT_FALSE(context->release_ready);
  EXPECT_TRUE(context->attached);
}
TEST_F(DirectMotion, NativeMoveItExecutionErrorSurvivesCleanup)
{
  capture();
  ASSERT_EQ(run(selection), BT::NodeStatus::SUCCESS);
  execution_error = -4;
  EXPECT_EQ(run("<MoveToPregrasp plan=\"{plan}\"/>"), BT::NodeStatus::FAILURE);
  auto error = execution_diagnostic(board);
  EXPECT_EQ(error.error_code, -4);
  EXPECT_EQ(error.error_domain, "moveit");
  EXPECT_EQ(error.error_name, "CONTROL_FAILED");
  EXPECT_EQ(error.error_source, "/execute_trajectory");
  EXPECT_EQ(error.error_msg, "native execution detail");
  EXPECT_FALSE(context->scene_active);
}
TEST(ExecutionDiagnostics, RootSnapshotAndStageResetDoNotStoreRosFeedback)
{
  auto root = BT::Blackboard::create();
  auto child = BT::Blackboard::create(root);
  rwh_beyond_bt::begin_stage(child, "approach", "approach_node");
  rwh_beyond_bt::report_failure(child, -4, "CONTROL_FAILED", "/execute_trajectory", "original",
    "moveit");
  rwh_beyond_bt::report_failure(root, -1200, "CLEANUP_FAILED", "cleanup", "secondary");
  EXPECT_EQ(execution_diagnostic(root).error_msg, "original");
  rwh_beyond_bt::begin_stage(child, "fallback", "recovery");
  EXPECT_EQ(execution_diagnostic(root).error_code, 0);
  EXPECT_EQ(execution_diagnostic(root).current_node, "recovery");
}

TEST_F(DirectMotion, SideObservationKeepsTiltAndRotatesOnlyOrientationAboutBaseZ)
{
  for (double side : {1., -1.}) {
    const auto xml = "<MoveObservation y=\"" + std::to_string(side * .15) +
      "\" z=\".10\" tilt_deg=\"40\" yaw_deg=\"" + std::to_string(-side * 30) +
      "\" settle_time=\"0\"/>";
    ASSERT_EQ(run(xml), BT::NodeStatus::SUCCESS);
    const auto actual = pose_matrix(last_ik_goal);
    Eigen::Matrix3d expected =
      (Eigen::AngleAxisd(-side * M_PI / 6, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(40 * M_PI / 180, Eigen::Vector3d::UnitY())).toRotationMatrix();
    EXPECT_TRUE(actual.linear().isApprox(expected, 1e-6));
    EXPECT_NEAR(actual.translation().x(), 0, 1e-6);
    EXPECT_NEAR(actual.translation().y(), side * .15, 1e-6);
    EXPECT_NEAR(actual.translation().z(), .60, 1e-6);
  }
}
TEST_F(DirectMotion, FusedCloudCanSelectAndExecuteUsingLatestCapture)
{
  capture();
  const auto latest = board->get<Observation>("capture");
  const auto mask = board->get<sensor_msgs::msg::Image>("mask");
  auto earlier = std::make_shared<RgbdObservation>(*latest);
  earlier->id = "earlier";
  earlier->depth.header.stamp = rclcpp::Time(latest->depth.header.stamp) -
    rclcpp::Duration::from_seconds(.01);
  earlier->info.header = earlier->depth.header;
  auto earlier_mask = mask; earlier_mask.header = earlier->depth.header;
  auto empty = std::make_shared<CloudFusion>(); empty->base_frame = "base_link";
  auto earlier_pose = Eigen::Isometry3d::Identity();
  earlier_pose.translation().z() = .004;
  auto fusion = append_fusion(empty, earlier, earlier_mask, earlier_pose);
  fusion = append_fusion(fusion, latest, mask, Eigen::Isometry3d::Identity());
  ASSERT_TRUE(fusion->views.back().icp_accepted) << fusion->views.back().icp_status;
  board->set("fusion", fusion);
  ASSERT_EQ(run("<BuildFusedObjectCloud fusion=\"{fusion}\" cloud=\"{cloud}\" min_views=\"2\"/>"),
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(board->get<sensor_msgs::msg::PointCloud2>("cloud").header, latest->depth.header);
  auto xml = selection;
  xml.insert(xml.find("/>"), " fusion=\"{fusion}\"");
  EXPECT_EQ(run("<Sequence>" + xml + "<MoveToPregrasp plan=\"{plan}\"/>" +
    "<ApproachGrasp plan=\"{plan}\"/><AttachObject plan=\"{plan}\"/>" +
    "<LiftObject plan=\"{plan}\"/></Sequence>"), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(context->attached);
}
TEST_F(DirectMotion, FusionNodesResetAndAccumulateWithoutRobotExecution)
{
  capture();
  ASSERT_EQ(run("<Sequence><ResetObjectCloudFusion fusion=\"{fusion}\"/>"
    "<AccumulateObjectCloud fusion=\"{fusion}\" capture=\"{capture}\" mask=\"{mask}\"/>"
    "</Sequence>"), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(board->get<Fusion>("fusion")->views.size(), 1u);
  EXPECT_EQ(run("<BuildFusedObjectCloud fusion=\"{fusion}\" cloud=\"{cloud}\" min_views=\"2\"/>"),
    BT::NodeStatus::FAILURE);
  EXPECT_EQ(run("<ResetObjectCloudFusion fusion=\"{fusion}\"/>"), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(board->get<Fusion>("fusion")->views.empty());
  EXPECT_EQ(executions, 0);
  EXPECT_EQ(closes, 0);
}
TEST_F(DirectMotion, FusionResetReadsIcpParametersAndHonorsExplicitOverride)
{
  client->declare_parameter("bt_motion.icp_enabled", false);
  client->declare_parameter("bt_motion.icp_max_iterations", 25);
  client->declare_parameter("bt_motion.icp_max_points", 5000);
  client->declare_parameter("bt_motion.icp_min_correspondences", 60);
  client->declare_parameter("bt_motion.icp_max_correspondence_distance_m", .02);
  client->declare_parameter("bt_motion.icp_min_overlap", .4);
  client->declare_parameter("bt_motion.icp_max_rmse_m", .003);
  client->declare_parameter("bt_motion.icp_max_translation_m", .008);
  client->declare_parameter("bt_motion.icp_max_rotation_deg", 4.0);
  ASSERT_EQ(run("<ResetObjectCloudFusion fusion=\"{fusion}\"/>"), BT::NodeStatus::SUCCESS);
  auto fusion = board->get<Fusion>("fusion");
  EXPECT_FALSE(fusion->use_icp);
  EXPECT_EQ(fusion->icp.max_iterations, 25);
  EXPECT_EQ(fusion->icp.max_points, 5000);
  EXPECT_EQ(fusion->icp.min_correspondences, 60);
  EXPECT_DOUBLE_EQ(fusion->icp.max_correspondence_distance, .02);
  EXPECT_DOUBLE_EQ(fusion->icp.min_overlap, .4);
  EXPECT_DOUBLE_EQ(fusion->icp.max_rmse, .003);
  EXPECT_DOUBLE_EQ(fusion->icp.max_translation, .008);
  EXPECT_DOUBLE_EQ(fusion->icp.max_rotation_deg, 4.0);
  ASSERT_EQ(run("<ResetObjectCloudFusion fusion=\"{fusion}\" use_icp=\"true\"/>"),
    BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(board->get<Fusion>("fusion")->use_icp);
  client->set_parameter(rclcpp::Parameter("bt_motion.icp_max_iterations", 0));
  EXPECT_EQ(run("<ResetObjectCloudFusion fusion=\"{fusion}\"/>"), BT::NodeStatus::FAILURE);
}

TEST_F(DirectMotion, ObservationUsesConfiguredOmplEvenWhenCartesianWouldFail)
{
  fraction = 0;
  expected_planner = "RRTstar";
  client->declare_parameter("bt_motion.moveit_observation_planner_id", "RRTstar");
  client->declare_parameter("bt_motion.moveit_observation_num_planning_attempts", 7);
  client->declare_parameter("bt_motion.moveit_observation_planning_time", 12.0);
  client->declare_parameter("bt_motion.moveit_observation_ik_timeout", 2.5);
  ASSERT_EQ(run(
    "<MoveObservation y=\".15\" z=\".10\" tilt_deg=\"40\" yaw_deg=\"-30\" settle_time=\"0\"/>"),
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(iks, 1); EXPECT_EQ(plans, 1); EXPECT_EQ(cartesians, 0); EXPECT_EQ(executions, 1);
  EXPECT_EQ(last_attempts, 7); EXPECT_DOUBLE_EQ(last_planning_time, 12);
  EXPECT_DOUBLE_EQ(last_ik_timeout, 2.5);
}
TEST_F(DirectMotion, UnreachableObservationNeverPlansOrExecutes)
{
  ik_error = -31;
  EXPECT_EQ(run("<MoveObservation settle_time=\"0\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(plans, 0); EXPECT_EQ(cartesians, 0); EXPECT_EQ(executions, 0);
  EXPECT_EQ(execution_diagnostic(board).error_source, "/compute_ik");
}
TEST_F(DirectMotion, InvalidObservationPlanningBudgetNeverExecutes)
{
  client->declare_parameter("bt_motion.moveit_observation_num_planning_attempts", 0);
  EXPECT_EQ(run("<MoveObservation settle_time=\"0\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(cartesians, 0); EXPECT_EQ(executions, 0);
}

TEST_F(DirectMotion, CartesianObservationDoesNotCallSeparateIkOrOmpl)
{
  client->declare_parameter("bt_motion.observation_motion_mode", "cartesian");
  ik_error = -31;
  ASSERT_EQ(run("<MoveObservation settle_time=\"0\"/>"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(iks, 0); EXPECT_EQ(plans, 0); EXPECT_EQ(cartesians, 1); EXPECT_EQ(executions, 1);
}
TEST_F(DirectMotion, CartesianObservationStillRefusesPartialPathWithoutFallback)
{
  client->declare_parameter("bt_motion.observation_motion_mode", "cartesian");
  fraction = 0;
  EXPECT_EQ(run("<MoveObservation settle_time=\"0\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(iks, 0); EXPECT_EQ(plans, 0); EXPECT_EQ(executions, 0);
  EXPECT_EQ(execution_diagnostic(board).error_code, ErrorCode::PARTIAL_PATH);
}
TEST_F(DirectMotion, InvalidObservationModeIsRejectedBeforePlanning)
{
  client->declare_parameter("bt_motion.observation_motion_mode", "invalid");
  EXPECT_EQ(run("<MoveObservation settle_time=\"0\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(iks, 0); EXPECT_EQ(plans, 0); EXPECT_EQ(cartesians, 0); EXPECT_EQ(executions, 0);
}

TEST_F(DirectMotion, JointPoseCaptureSavesFeedbackWithoutMotion)
{
  ASSERT_EQ(run("<CaptureJointPose pose_name=\"front\" joint_positions=\"{recorded}\"/>"),
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(load_joint_pose(pose_file, "front"), std::vector<double>(6, 0.));
  EXPECT_EQ(board->get<std::vector<double>>("recorded").size(), 6u);
  EXPECT_EQ(iks, 0); EXPECT_EQ(plans, 0); EXPECT_EQ(executions, 0); EXPECT_EQ(closes, 0);
}
TEST_F(DirectMotion, JointPoseMovePlansJointTargetWithoutIkOrCartesian)
{
  const std::vector<double> target{.1, .2, .3, .4, .5, .6};
  save_joint_pose(pose_file, "left", target);
  ASSERT_EQ(run("<MoveJointPose pose_name=\"left\" settle_time=\"0\"/>"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(iks, 0); EXPECT_EQ(cartesians, 0); EXPECT_EQ(plans, 1); EXPECT_EQ(executions, 1);
  EXPECT_EQ(closes, 0);
  const auto feedback = context->feedback();
  for (int i = 1; i <= 6; ++i) {
    const auto joint = "joint" + std::to_string(i);
    EXPECT_NEAR(feedback.at(joint), target[i - 1], .001);
  }
}
TEST_F(DirectMotion, JointPosePreflightStopsBeforeAnyMotionWhenMissing)
{
  save_joint_pose(pose_file, "front", std::vector<double>(6, .1));
  EXPECT_EQ(run("<Sequence><CheckJointPoses pose_names=\"front;left;right\"/>"
    "<MoveJointPose pose_name=\"front\" settle_time=\"0\"/></Sequence>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(plans, 0); EXPECT_EQ(executions, 0);
  EXPECT_EQ(run("<MoveJointPose pose_name=\"missing\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(executions, 0);
}
TEST_F(DirectMotion, AlreadyAtSavedJointPoseDoesNotSendExecution)
{
  save_joint_pose(pose_file, "front", std::vector<double>(6, 0.));
  EXPECT_EQ(run("<MoveJointPose pose_name=\"front\" settle_time=\"0\"/>"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(plans, 0); EXPECT_EQ(executions, 0); EXPECT_EQ(closes, 0);
}
TEST_F(DirectMotion, JointPoseUsesConfiguredSettleTime)
{
  save_joint_pose(pose_file, "front", std::vector<double>(6, 0.));
  client->declare_parameter("bt_motion.joint_pose_settle_time", .1);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(run("<MoveJointPose pose_name=\"front\"/>"), BT::NodeStatus::SUCCESS);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 100ms);
  EXPECT_EQ(plans, 0); EXPECT_EQ(executions, 0);
}
TEST_F(DirectMotion, SavedJointPoseStillRejectsPlannerEndpointMismatch)
{
  save_joint_pose(pose_file, "left", std::vector<double>(6, .1));
  wrong_goal = true;
  EXPECT_EQ(run("<MoveJointPose pose_name=\"left\" settle_time=\"0\"/>"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(executions, 0); EXPECT_EQ(iks, 0);
}
