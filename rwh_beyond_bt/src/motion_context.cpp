// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/motion_context.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>
#include <type_traits>
#include "urdf/model.h"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace rwh_beyond_bt
{
using PC = moveit_msgs::msg::PlanningSceneComponents;
using Object = moveit_msgs::msg::CollisionObject;
namespace
{
using SteadyClock = std::chrono::steady_clock;
double elapsed_seconds(SteadyClock::time_point start)
{
  return std::chrono::duration<double>(SteadyClock::now() - start).count();
}
struct ScopedSeconds
{
  double & total;
  SteadyClock::time_point start{SteadyClock::now()};
  ~ScopedSeconds() {total += elapsed_seconds(start);}
};
template<class Function>
auto timed_call(double & total, size_t & count, Function && function)
{
  ++count;
  ScopedSeconds timer{total};
  return function();
}
struct SelectionTiming
{
  rclcpp::Logger logger;
  SteadyClock::time_point start{SteadyClock::now()};
  double ik{0}, cartesian{0}, planning{0};
  size_t variants{0}, ik_calls{0}, cartesian_calls{0}, planning_calls{0};
  bool selected{false};
  ~SelectionTiming()
  {
    RCLCPP_INFO(logger,
      "TIMING selection outcome=%s total_s=%.3f variants=%zu "
      "ik_s=%.3f ik_calls=%zu cartesian_s=%.3f cartesian_calls=%zu "
      "planning_s=%.3f planning_calls=%zu",
      selected ? "selected" : "not_selected", elapsed_seconds(start), variants,
      ik, ik_calls, cartesian, cartesian_calls, planning, planning_calls);
  }
};
struct ExecutionTiming
{
  rclcpp::Logger logger;
  double planned;
  SteadyClock::time_point action_start{SteadyClock::now()}, feedback_start;
  double action_seconds{0};
  bool action_done{false}, success{false};
  ~ExecutionTiming()
  {
    const double action = action_done ? action_seconds : elapsed_seconds(action_start);
    const double feedback = action_done ? elapsed_seconds(feedback_start) : 0;
    RCLCPP_INFO(logger,
      "TIMING moveit_execution outcome=%s planned_s=%.3f action_s=%.3f feedback_s=%.3f",
      success ? "success" : "not_completed", planned, action, feedback);
  }
};
std::string moveit_name(int32_t code)
{
  switch (code) {
    case 1: return "SUCCESS";
    case 0: return "UNDEFINED";
    case 99999: return "FAILURE";
    case -1: return "PLANNING_FAILED";
    case -2: return "INVALID_MOTION_PLAN";
    case -3: return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case -4: return "CONTROL_FAILED";
    case -5: return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case -6: return "TIMED_OUT";
    case -7: return "PREEMPTED";
    case -10: return "START_STATE_IN_COLLISION";
    case -11: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case -26: return "START_STATE_INVALID";
    case -12: return "GOAL_IN_COLLISION";
    case -13: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case -14: return "GOAL_CONSTRAINTS_VIOLATED";
    case -27: return "GOAL_STATE_INVALID";
    case -28: return "UNRECOGNIZED_GOAL_TYPE";
    case -15: return "INVALID_GROUP_NAME";
    case -16: return "INVALID_GOAL_CONSTRAINTS";
    case -17: return "INVALID_ROBOT_STATE";
    case -18: return "INVALID_LINK_NAME";
    case -19: return "INVALID_OBJECT_NAME";
    case -21: return "FRAME_TRANSFORM_FAILURE";
    case -22: return "COLLISION_CHECKING_UNAVAILABLE";
    case -23: return "ROBOT_STATE_STALE";
    case -24: return "SENSOR_INFO_STALE";
    case -25: return "COMMUNICATION_FAILURE";
    case -29: return "CRASH";
    case -30: return "ABORT";
    case -31: return "NO_IK_SOLUTION";
    default: return "MOVEIT_ERROR_" + std::to_string(code);
  }
}
ActionFailure moveit_failure(
  const moveit_msgs::msg::MoveItErrorCodes & error, const std::string & source)
{
  if (error.val == 0) {
    return ActionFailure(ErrorCode::INVALID_RESPONSE, "moveit", "INVALID_RESPONSE", source,
      "MoveIt returned UNDEFINED (0)");
  }
  return ActionFailure(error.val, "moveit", moveit_name(error.val), source,
    error.message.empty() ? moveit_name(error.val) : error.message);
}
ActionFailure controller_failure(int32_t code, const std::string & source, const std::string & text)
{
  std::string name;
  switch (code) {
    case -1: name = "INVALID_GOAL"; break;
    case -2: name = "INVALID_JOINTS"; break;
    case -3: name = "OLD_HEADER_TIMESTAMP"; break;
    case -4: name = "PATH_TOLERANCE_VIOLATED"; break;
    case -5: name = "GOAL_TOLERANCE_VIOLATED"; break;
    default: name = "CONTROLLER_ERROR_" + std::to_string(code);
  }
  return ActionFailure(code, "controller", name, source, text.empty() ? name : text);
}
}  // namespace
void MotionContext::status(const std::string & source, const std::string & stage)
{
  if (auto board = diagnostic_board.lock()) {
    const bool moveit = source.find("compute_") != std::string::npos ||
      source.find("plan_kinematic") != std::string::npos ||
      source.find("planning_scene") != std::string::npos ||
      source.find("execute_trajectory") != std::string::npos;
    backend_stage(board, source + ":" + stage, moveit ? source + ":" + stage : "");
  }
}

constexpr char TARGET[] = "red_grasp_target";
const std::vector<std::string> FINGERS = {"gripper_link1", "gripper_link2"};

Eigen::Isometry3d pose_matrix(const geometry_msgs::msg::Pose & p)
{
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1) > .01) {
    throw std::invalid_argument("Invalid pose quaternion");
  }
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.normalized().toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  if (!t.matrix().allFinite()) {throw std::invalid_argument("Nonfinite pose");}
  return t;
}
geometry_msgs::msg::Pose matrix_pose(const Eigen::Isometry3d & t)
{
  geometry_msgs::msg::Pose p;
  p.position.x = t.translation().x(); p.position.y = t.translation().y();
  p.position.z = t.translation().z();
  Eigen::Quaterniond q(t.linear());
  p.orientation.x = q.x(); p.orientation.y = q.y();
  p.orientation.z = q.z(); p.orientation.w = q.w();
  return p;
}
double quantile(std::vector<double> values, double probability)
{
  if (values.empty() || !std::all_of(values.begin(), values.end(), [](double v) {
      return std::isfinite(v);
                              })) {throw std::invalid_argument("Invalid geometry samples");}
  std::sort(values.begin(), values.end());
  double index = probability * (values.size() - 1);
  auto lower = static_cast<size_t>(index);
  return values[lower] + (index - lower) * (values[std::min(lower + 1,
      values.size() - 1)] - values[lower]);
}
Joints joint_values(const sensor_msgs::msg::JointState & joints)
{
  if (joints.name.size() != joints.position.size()) {
    throw std::invalid_argument("Invalid joint state");
  }
  Joints result;
  for (size_t i = 0; i < joints.name.size(); ++i) {
    if (!std::isfinite(joints.position[i]) || !result.emplace(joints.name[i],
        joints.position[i]).second)
    {
      throw std::invalid_argument("Nonfinite or duplicate joint");
    }
  }
  return result;
}
bool trajectory_valid(const Trajectory & t, const Constraints & wrist)
{
  auto & j = t.joint_trajectory;
  if (j.points.empty() || j.joint_names.size() != 6 ||
    std::set<std::string>(j.joint_names.begin(), j.joint_names.end()).size() != 6) {return false;}
  for (int i = 1; i <= 6; ++i) {
    if (std::find(j.joint_names.begin(), j.joint_names.end(),
        "joint" + std::to_string(i)) == j.joint_names.end()) {return false;}
  }
  double previous = -1;
  for (auto & p : j.points) {
    double time = rclcpp::Duration(p.time_from_start).seconds();
    if (p.positions.size() != j.joint_names.size() || time <= previous) {return false;}
    previous = time;
    for (const auto * values : {&p.velocities, &p.accelerations, &p.effort}) {
      if (!values->empty() && (values->size() != j.joint_names.size() ||
        !std::all_of(values->begin(), values->end(), [](double v) {return std::isfinite(v);})))
      {
        return false;
      }
    }
    for (double value : p.positions) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    for (auto & c : wrist.joint_constraints) {
      auto i = std::find(j.joint_names.begin(), j.joint_names.end(), c.joint_name);
      if (i == j.joint_names.end()) {return false;}
      double value = p.positions[std::distance(j.joint_names.begin(), i)];
      if (value < c.position - c.tolerance_below || value > c.position + c.tolerance_above) {
        return false;
      }
    }
  }
  return previous > 0;
}
bool trajectory_reaches(const Trajectory & t, const State & target, double tolerance)
{
  if (!trajectory_valid(t)) {return false;}
  sensor_msgs::msg::JointState last;
  last.name = t.joint_trajectory.joint_names;
  last.position = t.joint_trajectory.points.back().positions;
  auto actual = joint_values(last), desired = joint_values(target.joint_state);
  for (int i = 1; i <= 6; ++i) {
    auto name = "joint" + std::to_string(i);
    if (!actual.count(name) || !desired.count(name) ||
      std::abs(actual[name] - desired[name]) > tolerance) {return false;}
  }
  return true;
}
std::pair<double, double> estimate_aperture(
  const Points & points, const Eigen::Isometry3d & grasp, double maximum, double inset)
{
  if (points.size() < 100 || !std::isfinite(maximum) || maximum <= 0 || maximum > .10 ||
    !std::isfinite(inset) || inset < 0 || inset > .005)
  {
    throw std::invalid_argument(
        "grasp_width_closing_inset must be in [0, 0.005] m; opening in (0, 0.10]");
  }
  std::vector<double> x;
  for (auto & p : points) {
    x.push_back((p - grasp.translation()).dot(grasp.linear().col(0)));
  }
  double lower = quantile(x, .02), upper = quantile(x, .98);
  if (upper - lower < .008 || lower >= 0 || upper <= 0) {
    throw std::invalid_argument("Observed width is degenerate or grasp axis is outside object");
  }
  double width = 2 * std::max(std::abs(lower), std::abs(upper));
  return {width, std::min(maximum, width - inset)};
}

MotionContext::MotionContext(rclcpp::Node::SharedPtr parent)
: node(parent)
{
  if (!node->has_parameter("octomap_enabled")) {node->declare_parameter("octomap_enabled", true);}
  base = parameter<std::string>("base_frame", "base_link");
  tcp = parameter<std::string>("tcp_link", "tcp_link");
  simulation = parameter<bool>("simulation_only", true);
  group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(group_, node->get_node_base_interface());
  rclcpp::SubscriptionOptions options;
  options.callback_group = group_;
  joint_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "feedback/joint_states", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (simulation || msg->header.stamp != joints_.header.stamp || joints_.name.empty()) {
        joints_ = *msg; received_ = std::chrono::steady_clock::now();
      }
    }, options);
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "camera/depth/filtered_points", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
      if (msg->width && stamp > cloud_stamp_) {cloud_stamp_ = stamp; ++cloud_seq_;}
    }, options);
  scene_ = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene",
      rclcpp::ServicesQoS(), group_);
  apply_ = node->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene",
      rclcpp::ServicesQoS(), group_);
  ik_ = node->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik", rclcpp::ServicesQoS(),
      group_);
  fk_ = node->create_client<moveit_msgs::srv::GetPositionFK>("compute_fk", rclcpp::ServicesQoS(),
      group_);
  plan_ = node->create_client<moveit_msgs::srv::GetMotionPlan>("plan_kinematic_path",
      rclcpp::ServicesQoS(), group_);
  cartesian_ = node->create_client<moveit_msgs::srv::GetCartesianPath>("compute_cartesian_path",
      rclcpp::ServicesQoS(), group_);
  region_ = node->create_client<rwh_beyond_msgs::srv::SetGraspRegion>(
    simulation ? "simulation/set_grasp_region" : "grasp/set_grasp_region", rclcpp::ServicesQoS(),
      group_);
  clear_ = node->create_client<std_srvs::srv::Empty>("clear_octomap", rclcpp::ServicesQoS(),
      group_);
  hardware_ = node->create_client<std_srvs::srv::Trigger>("grasp/hardware_transport_ready",
      rclcpp::ServicesQoS(), group_);
  sam_ = node->create_client<std_srvs::srv::Trigger>("sam3/health", rclcpp::ServicesQoS(), group_);
  model_ = node->create_client<std_srvs::srv::Trigger>("graspgenx/health", rclcpp::ServicesQoS(),
      group_);
  description_ =
    node->create_client<rcl_interfaces::srv::GetParameters>("move_group/get_parameters",
      rclcpp::ServicesQoS(), group_);
  execute_ = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(node,
      "execute_trajectory", group_);
  gripper_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node,
      "gripper_controller/follow_joint_trajectory", group_);
  buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock(), tf2::durationFromSec(30));
  listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
  grasp_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("grasp/selected_tcp",
      rclcpp::QoS(1).reliable().transient_local());
  pre_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("grasp/selected_pregrasp",
      rclcpp::QoS(1).reliable().transient_local());
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  mask_pub_ = node->create_publisher<sensor_msgs::msg::Image>("grasp/object_mask", qos);
  preview_pub_ = node->create_publisher<sensor_msgs::msg::Image>("grasp/object_mask_preview", qos);
  overlay_pub_ = node->create_publisher<sensor_msgs::msg::Image>("grasp/segmentation_overlay", qos);
  base_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("grasp/object_cloud_base",
      qos);
  candidates_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("grasp/grasp_candidates", qos);
  selected_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("grasp/selected_grasp", qos);
  spinner_ = std::thread([this]() {executor_.spin();});
}
MotionContext::~MotionContext()
{
  executor_.cancel();
  if (spinner_.joinable()) {spinner_.join();}
}
std::shared_ptr<MotionContext> MotionContext::get(const BT::Blackboard::Ptr & board)
{
  auto root = board->rootBlackboard();
  std::shared_ptr<MotionContext> context;
  if (!root->get("bt_motion_context", context)) {
    context = std::make_shared<MotionContext>(board->get<rclcpp::Node::SharedPtr>("node"));
    root->set("bt_motion_context", context);
    std::weak_ptr<MotionContext> weak = context;
    root->set<std::function<void()>>("bt_motion_cleanup", [weak]() {
        if (auto ctx = weak.lock()) {
          std::lock_guard<std::mutex> lock(ctx->operation);
          ctx->cleanup(std::make_shared<std::atomic_bool>(false));
        }
    });
  }
  context->diagnostic_board = board;
  return context;
}
double MotionContext::number(const std::string & name, double fallback, double low, double high)
{
  double value = parameter<double>(name, fallback);
  if (!std::isfinite(value) || value < low || value > high) {
    throw std::invalid_argument("bt_motion." + name + " outside allowed range");
  }
  return value;
}
void MotionContext::wait(
  const std::function<bool()> & predicate, double timeout,
  const Cancel & cancel)
{
  auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  while (true) {
    if (cancel->load() || !rclcpp::ok()) {
      throw ActionFailure(ErrorCode::CANCELED, "bt", "CANCELED", "motion", "BT motion canceled");
    }
    if (predicate()) {return;}
    if (std::chrono::steady_clock::now() >= until) {
      throw ActionFailure(ErrorCode::TIMEOUT, "bt", "TIMEOUT", "motion",
          "BT motion request timed out");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
Joints MotionContext::feedback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() >
    number("hardware_feedback_timeout", 1, .01, 10))
  {
    throw ActionFailure(ErrorCode::STALE_FEEDBACK, "bt", "STALE_FEEDBACK", "/feedback/joint_states",
        "Joint feedback is missing or stale");
  }
  auto values = joint_values(joints_);
  for (int i = 1; i <= 6; ++i) {
    values.at("joint" + std::to_string(i));
  }
  values.at("gripper");
  return values;
}
State MotionContext::state(const Cancel & cancel)
{
  feedback();
  if (!simulation && limits_.empty()) {
    rcl_interfaces::srv::GetParameters::Request request;
    request.names = {"robot_description"};
    auto response = call<rcl_interfaces::srv::GetParameters>(description_, request, cancel);
    urdf::Model model;
    if (response->values.size() != 1 || !model.initString(response->values[0].string_value)) {
      throw std::runtime_error("MoveIt robot_description unavailable");
    }
    for (int i = 1; i <= 6; ++i) {
      auto name = "joint" + std::to_string(i);
      auto joint = model.getJoint(name);
      if (!joint || !joint->limits) {throw std::runtime_error("Missing arm joint limits");}
      limits_[name] = {joint->limits->lower, joint->limits->upper};
    }
  }
  feedback();
  State result;
  {std::lock_guard<std::mutex> lock(mutex_); result.joint_state = joints_;}
  result.is_diff = true;
  double tolerance = number("planning_start_bounds_tolerance_deg", 10, .001, 180) * M_PI / 180;
  for (size_t i = 0; i < result.joint_state.name.size(); ++i) {
    auto name = result.joint_state.name[i];
    if (!limits_.count(name)) {continue;}
    auto bounds = limits_.at(name);
    double & q = result.joint_state.position[i];
    double bounded = std::clamp(q, bounds.first, bounds.second);
    if (std::abs(bounded - q) > tolerance) {
      throw std::runtime_error("Start state outside joint limits: " + name);
    }
    if (bounded != q) {
      RCLCPP_WARN(node->get_logger(), "Planning-only bounds correction: %s %.4f -> %.4f",
          name.c_str(), q, bounded);
    }
    q = bounded;
  }
  return result;
}
void MotionContext::ensure_backend(const Cancel & cancel)
{
  feedback();
  if (!simulation) {
    auto result = call<std_srvs::srv::Trigger>(hardware_, std_srvs::srv::Trigger::Request(), cancel,
        3);
    if (!result->success || result->message != "physical_feedback_transport") {
      throw std::runtime_error("Physical transport unavailable: " + result->message);
    }
  }
}
void MotionContext::check_ready(bool repeat, const Cancel & cancel)
{
  wait(
    [&]() {
      try {
        feedback(); return true;
      } catch (...) {
        return false;
      }
                                                                        }, 5, cancel);
  ensure_backend(cancel);
  prepare_motion(cancel);
  if (attached && repeat) {
    release(number("gripper_open_width", .10, .001, .10), 3, true, cancel);
  }
  if (attached) {
    throw ActionFailure(ErrorCode::PAYLOAD_PRESENT, "bt", "PAYLOAD_PRESENT", "CheckGraspReady",
        "Target remains attached; place/release before picking");
  }
  for (auto & client : {sam_, model_}) {
    auto result = call<std_srvs::srv::Trigger>(client, std_srvs::srv::Trigger::Request(), cancel,
        10);
    if (!result->success) {
      throw ActionFailure(ErrorCode::HEALTH_CHECK_FAILED, "transport", "HEALTH_CHECK_FAILED",
        client->get_service_name(), result->message);
    }
  }
  wait(
    [&]() {
      return ik_->service_is_ready() && fk_->service_is_ready() && plan_->service_is_ready() &&
             cartesian_->service_is_ready() && execute_->action_server_is_ready() &&
             gripper_->action_server_is_ready();
    },
      15, cancel);
}
geometry_msgs::msg::Pose MotionContext::fk(const State & start, const Cancel & cancel)
{
  moveit_msgs::srv::GetPositionFK::Request request;
  request.header.frame_id = base; request.fk_link_names = {tcp}; request.robot_state = start;
  auto response = call<moveit_msgs::srv::GetPositionFK>(fk_, request, cancel);
  if (response->error_code.val != 1) {throw moveit_failure(response->error_code, "/compute_fk");}
  if (response->pose_stamped.size() != 1) {
    throw ActionFailure(ErrorCode::INVALID_RESPONSE, "moveit", "INVALID_RESPONSE", "/compute_fk",
      "TCP FK returned the wrong number of poses");
  }
  return response->pose_stamped[0].pose;
}
State MotionContext::ik(
  const geometry_msgs::msg::Pose & pose, const State & start,
  const Cancel & cancel, bool candidate, PlanningProfile profile)
{
  moveit_msgs::srv::GetPositionIK::Request request;
  auto & r = request.ik_request;
  r.group_name = "arm"; r.ik_link_name = tcp; r.robot_state = start; r.avoid_collisions = true;
  r.pose_stamped.header.frame_id = base; r.pose_stamped.pose = pose;
  const double timeout = profile == PlanningProfile::OBSERVATION ?
    number("moveit_observation_ik_timeout", 1.0, .001, 10) :
    number("moveit_ik_timeout", .2, .001, 10);
  r.timeout = rclcpp::Duration::from_seconds(timeout);
  auto response = call<moveit_msgs::srv::GetPositionIK>(ik_, request, cancel);
  if (response->error_code.val != 1) {
    auto error = moveit_failure(response->error_code, "/compute_ik");
    if (!candidate) {throw error;}
    candidate_error = error;
    return State();
  }
  return response->solution;
}
moveit_msgs::srv::GetCartesianPath::Response::SharedPtr MotionContext::cartesian(
  const geometry_msgs::msg::Pose & pose, const State & start, const Cancel & cancel)
{
  moveit_msgs::srv::GetCartesianPath::Request request;
  request.header.frame_id = base; request.group_name = "arm"; request.link_name = tcp;
  request.start_state = start; request.waypoints = {pose}; request.max_step = .004;
  request.avoid_collisions = true; request.revolute_jump_threshold = .35;
  request.max_velocity_scaling_factor = number("moveit_cartesian_velocity_scaling_factor", .5, .001,
      1);
  request.max_acceleration_scaling_factor = number("moveit_cartesian_acceleration_scaling_factor",
      .5, .001, 1);
  return call<moveit_msgs::srv::GetCartesianPath>(cartesian_, request, cancel);
}
Trajectory MotionContext::plan(
  const State & start, const State & goal,
  const Constraints & wrist, const Cancel & cancel, bool candidate, PlanningProfile profile)
{
  moveit_msgs::srv::GetMotionPlan::Request request;
  auto & r = request.motion_plan_request;
  r.group_name = "arm"; r.pipeline_id = "ompl";
  const bool observation = profile == PlanningProfile::OBSERVATION;
  r.planner_id = parameter<std::string>(observation ? "moveit_observation_planner_id" :
      "moveit_pregrasp_planner_id", "RRTConnect");
  if (r.planner_id.empty()) {throw std::invalid_argument("MoveIt planner ID must not be empty");}
  r.num_planning_attempts =
    parameter<int>(observation ? "moveit_observation_num_planning_attempts" :
      "moveit_pregrasp_num_planning_attempts", observation ? 5 : 3);
  if (r.num_planning_attempts < 1 || r.num_planning_attempts > 10) {
    throw std::invalid_argument("Planning attempts must be in [1,10]");
  }
  r.allowed_planning_time = observation ? number("moveit_observation_planning_time", 10, .01, 120) :
    (candidate ? number("moveit_candidate_planning_time", 4, .01, 60) :
    number("moveit_planning_time", 10, .01, 120));
  r.max_velocity_scaling_factor = number("moveit_velocity_scaling_factor", .5, .001, 1);
  r.max_acceleration_scaling_factor = number("moveit_acceleration_scaling_factor", .5, .001, 1);
  r.start_state = start; r.path_constraints = wrist;
  Constraints target;
  auto values = joint_values(goal.joint_state);
  for (int i = 1; i <= 6; ++i) {
    moveit_msgs::msg::JointConstraint c;
    c.joint_name = "joint" + std::to_string(i); c.position = values.at(c.joint_name);
    c.tolerance_above = c.tolerance_below = .001; c.weight = 1;
    target.joint_constraints.push_back(c);
  }
  r.goal_constraints = {target};
  const auto planning_started = SteadyClock::now();
  auto response = call<moveit_msgs::srv::GetMotionPlan>(plan_, request, cancel,
      r.allowed_planning_time + 10);
  auto & result = response->motion_plan_response;
  if (!candidate) {
    RCLCPP_INFO(node->get_logger(),
      "TIMING moveit_plan profile=%s wall_s=%.3f moveit_s=%.3f code=%d",
      observation ? "observation" : "general", elapsed_seconds(planning_started),
      result.planning_time, result.error_code.val);
  }
  if (result.error_code.val != 1) {
    auto error = moveit_failure(result.error_code, "/plan_kinematic_path");
    if (!candidate) {throw error;}
    candidate_error = error;
    return Trajectory();
  }
  if (!trajectory_valid(result.trajectory, wrist) || !trajectory_reaches(result.trajectory, goal)) {
    if (!candidate) {
      throw ActionFailure(ErrorCode::INVALID_TRAJECTORY, "bt", "INVALID_TRAJECTORY",
        "/plan_kinematic_path", "Returned trajectory violates constraints or misses its goal");
    }
    return Trajectory();
  }
  return result.trajectory;
}

// An accepted action is always canceled on halt, timeout or result failure.
// If halt happens before acceptance, await acknowledgement and cancel the late goal too.
template<class Action>
void run_action(
  const typename rclcpp_action::Client<Action>::SharedPtr & client,
  const typename Action::Goal & goal, MotionContext & context, const Cancel & cancel,
  double timeout, const std::function<bool(const typename Action::Result &)> & success)
{
  const std::string source = context.node->get_node_topics_interface()->resolve_topic_name(
    std::is_same_v<Action, moveit_msgs::action::ExecuteTrajectory> ?
    "execute_trajectory" : "gripper_controller/follow_joint_trajectory");
  context.status(source, "waiting_for_server");
  try {
    context.wait([&]() {
        return client->action_server_is_ready();
                                                                   }, 10, cancel);
  } catch (const ActionFailure & e) {
    throw ActionFailure(e.code == ErrorCode::TIMEOUT ? ErrorCode::SERVER_UNAVAILABLE : e.code,
      "transport", e.code == ErrorCode::TIMEOUT ? "SERVER_UNAVAILABLE" : e.name, source, e.what());
  }
  context.status(source, "sending_goal");
  typename rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [client, cancel](auto handle) {
      if (handle && cancel->load()) {
        try {
          client->async_cancel_goal(handle);
        } catch (...) {
        }
      }
    };
  auto sent = client->async_send_goal(goal, options);
  typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr handle;
  try {
    context.wait(
      [&]() {
        return sent.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        }, 10, cancel);
    handle = sent.get();
    if (!handle) {
      throw ActionFailure(ErrorCode::GOAL_REJECTED, "transport", "GOAL_REJECTED", source,
        "Action goal rejected");
    }
    context.status(source, "executing");
    auto result = client->async_get_result(handle);
    context.wait(
      [&]() {
        return result.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    },
        timeout, cancel);
    auto value = result.get();
    if (!value.result) {
      throw ActionFailure(ErrorCode::INVALID_RESPONSE, "transport", "INVALID_RESPONSE", source,
        "Action returned no result");
    }
    if (!success(*value.result)) {
      if constexpr (std::is_same_v<Action, moveit_msgs::action::ExecuteTrajectory>) {
        throw moveit_failure(value.result->error_code, source);
      } else {
        throw controller_failure(value.result->error_code, source, value.result->error_string);
      }
    }
    if (value.code != rclcpp_action::ResultCode::SUCCEEDED) {
      const bool canceled = value.code == rclcpp_action::ResultCode::CANCELED;
      throw ActionFailure(canceled ? ErrorCode::ACTION_CANCELED : ErrorCode::ACTION_ABORTED,
        "transport", canceled ? "ACTION_CANCELED" : "ACTION_ABORTED", source,
        "Action status was not SUCCEEDED despite a success payload");
    }
    context.status(source, "completed");
  } catch (...) {
    if (handle) {
      try {
        auto canceled = client->async_cancel_goal(handle);
        canceled.wait_for(std::chrono::seconds(2));
      } catch (...) {
      }
    } else {
      cancel->store(true);
      if (sent.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto late = sent.get(); if (late) {
          try {
            client->async_cancel_goal(late);
          } catch (...) {
          }
        }
      }
    }
    try {
      throw;
    } catch (const ActionFailure & error) {
      if (error.source == "motion") {
        throw ActionFailure(error.code, "transport", error.name, source, error.what());
      }
      throw;
    }
  }
}
void MotionContext::execute(const Trajectory & trajectory, const Cancel & cancel)
{
  if (!trajectory_valid(trajectory)) {
    throw ActionFailure(ErrorCode::INVALID_TRAJECTORY, "bt", "INVALID_TRAJECTORY",
        "/execute_trajectory", "Invalid executable trajectory");
  }
  ensure_backend(cancel);
  auto current = feedback();
  auto & j = trajectory.joint_trajectory;
  for (size_t i = 0; i < j.joint_names.size(); ++i) {
    if (j.joint_names[i] != "gripper" &&
      std::abs(current.at(j.joint_names[i]) - j.points.front().positions[i]) > .20)
    {
      throw ActionFailure(ErrorCode::INVALID_PLAN, "bt", "INVALID_PLAN", "/execute_trajectory",
          "Robot moved since planning");
    }
  }
  moveit_msgs::action::ExecuteTrajectory::Goal goal;
  goal.trajectory = trajectory;
  ExecutionTiming timing{
    node->get_logger(), rclcpp::Duration(j.points.back().time_from_start).seconds()};
  run_action<moveit_msgs::action::ExecuteTrajectory>(execute_, goal, *this, cancel, 120,
    [](const auto & result) {return result.error_code.val == 1;});
  timing.action_seconds = elapsed_seconds(timing.action_start);
  timing.feedback_start = SteadyClock::now();
  timing.action_done = true;
  double tolerance = number("physical_position_tolerance", .03, .001, .20);
  wait(
    [&]() {
      auto values = feedback();
      for (size_t i = 0; i < j.joint_names.size(); ++i) {
        if (j.joint_names[i] != "gripper" &&
        std::abs(values.at(j.joint_names[i]) - j.points.back().positions[i]) > tolerance)
        {
          return false;
        }
      }
      return true;
  }, 20, cancel);
  timing.success = true;
}
void MotionContext::execute_cartesian(
  const geometry_msgs::msg::Pose & pose,
  const Constraints & wrist, const Cancel & cancel)
{
  const auto cartesian_started = SteadyClock::now();
  auto result = cartesian(pose, state(cancel), cancel);
  RCLCPP_INFO(node->get_logger(), "TIMING moveit_cartesian wall_s=%.3f fraction=%.3f code=%d",
    elapsed_seconds(cartesian_started), result->fraction, result->error_code.val);
  if (result->error_code.val != 1) {
    throw moveit_failure(result->error_code, "/compute_cartesian_path");
  }
  if (!std::isfinite(result->fraction) || result->fraction < .999 || result->fraction > 1) {
    throw ActionFailure(ErrorCode::PARTIAL_PATH, "bt", "PARTIAL_PATH", "/compute_cartesian_path",
      "Incomplete Cartesian path: fraction=" + std::to_string(result->fraction));
  }
  if (!trajectory_valid(result->solution, wrist)) {
    throw ActionFailure(ErrorCode::INVALID_TRAJECTORY, "bt", "INVALID_TRAJECTORY",
      "/compute_cartesian_path", "Cartesian trajectory is invalid or violates wrist limits");
  }
  execute(result->solution, cancel);
}
void MotionContext::gripper(double opening, double duration, bool closing, const Cancel & cancel)
{
  if (!std::isfinite(opening) || opening < 0 || opening > .10 || !std::isfinite(duration) ||
    duration <= 0 || duration > 30)
  {
    throw std::invalid_argument("Invalid gripper opening/duration");
  }
  ensure_backend(cancel);
  control_msgs::action::FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"gripper"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {opening}; point.time_from_start = rclcpp::Duration::from_seconds(duration);
  goal.trajectory.points = {point};
  if (closing) {
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "gripper"; tolerance.position = tolerance.velocity = -1;
    goal.path_tolerance = {tolerance};
  }
  run_action<control_msgs::action::FollowJointTrajectory>(gripper_, goal, *this, cancel, 45,
    [](const auto & result) {return result.error_code == 0;});
  if (!closing) {
    wait(
      [&]() {
        return std::abs(feedback().at("gripper") - opening) < .005;
                                                                                        }, 10,
        cancel);
  }
}
moveit_msgs::msg::PlanningScene MotionContext::scene(uint32_t components, const Cancel & cancel)
{
  moveit_msgs::srv::GetPlanningScene::Request request;
  request.components.components = components;
  return call<moveit_msgs::srv::GetPlanningScene>(scene_, request, cancel)->scene;
}
void MotionContext::apply(moveit_msgs::msg::PlanningScene value, const Cancel & cancel)
{
  value.is_diff = value.robot_state.is_diff = true;
  moveit_msgs::srv::ApplyPlanningScene::Request request; request.scene = value;
  if (!call<moveit_msgs::srv::ApplyPlanningScene>(apply_, request, cancel)->success) {
    throw ActionFailure(ErrorCode::SCENE_UPDATE_FAILED, "moveit", "SCENE_UPDATE_FAILED",
        "/apply_planning_scene", "Planning Scene update rejected");
  }
}
static void set_pair(
  moveit_msgs::msg::AllowedCollisionMatrix & matrix,
  const std::string & a, const std::string & b, bool allowed)
{
  for (auto name : {a, b}) {
    if (std::find(matrix.entry_names.begin(), matrix.entry_names.end(),
        name) == matrix.entry_names.end())
    {
      matrix.entry_names.push_back(name);
      for (auto & row : matrix.entry_values) {
        row.enabled.push_back(false);
      }
      moveit_msgs::msg::AllowedCollisionEntry row;
      row.enabled.resize(matrix.entry_names.size(), false); matrix.entry_values.push_back(row);
    }
  }
  auto i = std::find(matrix.entry_names.begin(), matrix.entry_names.end(),
      a) - matrix.entry_names.begin();
  auto j = std::find(matrix.entry_names.begin(), matrix.entry_names.end(),
      b) - matrix.entry_names.begin();
  matrix.entry_values.at(i).enabled.at(j) = allowed;
  matrix.entry_values.at(j).enabled.at(i) = allowed;
}
void MotionContext::cleanup(const Cancel & cancel)
{
  selected.reset();
  if (attached || !scene_active) {return;}
  auto current = scene(PC::ALLOWED_COLLISION_MATRIX | PC::ROBOT_STATE_ATTACHED_OBJECTS |
      PC::WORLD_OBJECT_NAMES, cancel);
  for (auto & item : current.robot_state.attached_collision_objects) {
    if (item.object.id == TARGET) {attached = true; return;}
  }
  moveit_msgs::msg::PlanningScene update;
  update.allowed_collision_matrix = current.allowed_collision_matrix;
  for (auto & pair : original_pairs) {
    set_pair(update.allowed_collision_matrix, pair.first, TARGET, pair.second);
  }
  for (auto & item : current.world.collision_objects) {
    if (item.id == TARGET) {
      Object removed; removed.id = TARGET; removed.operation = Object::REMOVE;
      update.world.collision_objects.push_back(removed);
    }
  }
  apply(update, cancel);
  if (node->get_parameter("octomap_enabled").as_bool()) {
    rwh_beyond_msgs::srv::SetGraspRegion::Request request; request.enabled = false;
    if (!call<rwh_beyond_msgs::srv::SetGraspRegion>(region_, request, cancel)->success) {
      throw std::runtime_error("Could not clear target exclusion");
    }
  }
  scene_active = false;
}
void MotionContext::prepare_motion(const Cancel & cancel)
{
  ensure_backend(cancel);
  auto current = scene(PC::ROBOT_STATE_ATTACHED_OBJECTS | PC::WORLD_OBJECT_NAMES, cancel);
  bool found = false;
  for (auto & item : current.robot_state.attached_collision_objects) {
    if (item.object.id != TARGET) {throw std::runtime_error("Unknown attached object");}
    found = true;
  }
  if (attached && !found) {
    throw std::runtime_error(
        "Payload collision attachment missing (or close was interrupted); "
        "inspect gripper before retry");
  }
  attached = found;
  if (!attached) {cleanup(cancel);}
  for (auto & item : current.world.collision_objects) {
    if (item.id == "observed_support_table") {
      moveit_msgs::msg::PlanningScene update;
      Object removed; removed.id = item.id; removed.operation = Object::REMOVE;
      update.world.collision_objects.push_back(removed); apply(update, cancel);
    }
  }
}
void MotionContext::release(double opening, double duration, bool repeat, const Cancel & cancel)
{
  if (!attached || (!repeat && !release_ready)) {
    throw ActionFailure(ErrorCode::RELEASE_NOT_READY, "bt", "RELEASE_NOT_READY", "ReleaseObject",
        "Release requires attached payload and successful place");
  }
  gripper(opening, duration, false, cancel);
  moveit_msgs::msg::PlanningScene update;
  moveit_msgs::msg::AttachedCollisionObject detached;
  detached.object.id = TARGET; detached.object.operation = Object::REMOVE;
  update.robot_state.attached_collision_objects.push_back(detached); apply(update, cancel);
  attached = false; release_ready = false; scene_active = true;
  cleanup(cancel);
}
Eigen::Isometry3d MotionContext::camera_pose_at(const Observation & capture, const Cancel & cancel)
{
  if (!capture) {throw std::invalid_argument("Missing TF reference capture");}
  auto stamp = rclcpp::Time(capture->depth.header.stamp, node->get_clock()->get_clock_type());
  wait(
    [&]() {return buffer_->canTransform(base, capture->depth.header.frame_id, stamp);}, 2, cancel);
  auto transform = buffer_->lookupTransform(base, capture->depth.header.frame_id, stamp).transform;
  geometry_msgs::msg::Pose camera;
  camera.position.x = transform.translation.x; camera.position.y = transform.translation.y;
  camera.position.z = transform.translation.z; camera.orientation = transform.rotation;
  return pose_matrix(camera);
}
void MotionContext::validate_capture(const Observation & capture)
{
  if (!capture || capture->id.empty()) {throw std::invalid_argument("Missing BT capture");}
  double age = (node->now() - rclcpp::Time(capture->depth.header.stamp,
      node->get_clock()->get_clock_type())).seconds();
  if (age < 0 || age > 30) {
    throw ActionFailure(ErrorCode::STALE_OBSERVATION, "bt", "STALE_OBSERVATION",
        "SelectFeasibleGrasp", "Capture expired or future timestamp");
  }
  auto old = joint_values(capture->joints), now = feedback();
  for (int i = 1; i <= 6; ++i) {
    auto name = "joint" + std::to_string(i);
    if (std::abs(old.at(name) - now.at(name)) > .02) {
      throw ActionFailure(ErrorCode::STALE_OBSERVATION, "bt", "STALE_OBSERVATION",
          "SelectFeasibleGrasp", "Arm moved since capture; reobserve");
    }
  }
}
static visualization_msgs::msg::Marker arrow(
  const std_msgs::msg::Header & header, const std::string & ns, int id,
  const Eigen::Vector3d & start, const Eigen::Vector3d & end, int color)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header; marker.ns = ns; marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.pose.orientation.w = 1;
  geometry_msgs::msg::Point point;
  point.x = start.x(); point.y = start.y(); point.z = start.z(); marker.points.push_back(point);
  point.x = end.x(); point.y = end.y(); point.z = end.z(); marker.points.push_back(point);
  marker.scale.x = .003; marker.scale.y = .007; marker.scale.z = .01;
  marker.color.a = 1; marker.color.r = color == 0; marker.color.g = color == 1;
  marker.color.b = color == 2;
  return marker;
}
static visualization_msgs::msg::MarkerArray clear_markers(const std_msgs::msg::Header & header)
{
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker marker;
  marker.header = header; marker.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(marker); return array;
}
void MotionContext::publish_selection(const Plan & plan)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = base; pose.header.stamp = plan->capture->depth.header.stamp;
  pose.pose = plan->grasp;
  grasp_pub_->publish(pose); pose.pose = plan->pregrasp; pre_pub_->publish(pose);
  auto array = clear_markers(pose.header);
  auto transform = pose_matrix(plan->grasp);
  for (int axis = 0; axis < 3; ++axis) {
    array.markers.push_back(arrow(pose.header, "selected_tcp_axes", axis,
      transform.translation(), transform.translation() + transform.linear().col(axis) * .04, axis));
  }
  Eigen::Vector3d pre(plan->pregrasp.position.x, plan->pregrasp.position.y,
    plan->pregrasp.position.z);
  array.markers.push_back(arrow(pose.header, "selected_approach", 0, pre, transform.translation(),
      1));
  selected_pub_->publish(array);
}

static Points cloud_points(const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (cloud.is_bigendian || cloud.point_step == 0 ||
    cloud.data.size() != static_cast<size_t>(cloud.row_step) * cloud.height)
  {
    throw std::invalid_argument("Invalid XYZ cloud storage");
  }
  for (const auto & name : {"x", "y", "z"}) {
    auto field = std::find_if(
      cloud.fields.begin(), cloud.fields.end(), [&](const auto & f) {
        return f.name == name;
    });
    if (field == cloud.fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      field->offset + 4 > cloud.point_step)
    {
      throw std::invalid_argument("Cloud requires float32 XYZ");
    }
  }
  Points points;
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    Eigen::Vector3d point(*x, *y, *z);
    if (!point.allFinite()) {throw std::invalid_argument("Nonfinite cloud point");}
    points.push_back(point);
  }
  if (points.size() < 100 || points.size() > 200000) {
    throw std::invalid_argument("Object cloud count outside [100,200000]");
  }
  return points;
}
Plan MotionContext::select(
  const Observation & capture, const sensor_msgs::msg::Image & mask,
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::vector<rwh_beyond_msgs::msg::GraspCandidate> & candidates,
  bool estimated_width, double pre_distance, double lift_distance, const Cancel & cancel,
  const Fusion & fusion)
{
  SelectionTiming timing{node->get_logger()};
  candidate_error.reset();
  if (attached || scene_active) {throw std::runtime_error("Previous target still active");}
  selected.reset();
  validate_capture(capture);
  if (cloud.header != capture->depth.header || mask.header != cloud.header || candidates.empty()) {
    throw std::invalid_argument("Cloud/mask must match capture; candidates required");
  }
  for (auto & candidate : candidates) {
    if (candidate.pose.header != cloud.header || !std::isfinite(candidate.score)) {
      throw std::invalid_argument("Candidate frame/stamp/score invalid");
    }
    pose_matrix(candidate.pose.pose);
  }
  auto xyz = cloud_points(cloud);
  // Also validate that the cloud is the masked capture, not a stale artifact.
  if (fusion && (fusion->views.size() < 2 || fusion->base_frame != base ||
    fusion->views.back().capture != capture || fusion->views.back().mask != mask))
  {throw std::invalid_argument("Fusion reference must match the current capture and mask");}
  auto projected = cloud_points(fusion ? fuse_object_cloud(*fusion) :
      project_object_cloud(*capture, mask));
  if (xyz.size() != projected.size()) {
    throw std::invalid_argument("Cloud does not match captured mask/depth");
  }
  for (size_t i = 0; i < xyz.size(); ++i) {
    if ((xyz[i] - projected[i]).norm() > 1e-5) {
      throw std::invalid_argument("Cloud differs from masked RGB-D");
    }
  }
  auto base_camera = camera_pose_at(capture, cancel);
  if (fusion && !base_camera.matrix().isApprox(fusion->views.back().base_camera.matrix(), 1e-5)) {
    throw std::invalid_argument("Reference camera TF changed since fusion");
  }
  auto display_header = cloud.header; display_header.frame_id = base;
  auto base_cloud = cloud; base_cloud.header = display_header;
  sensor_msgs::PointCloud2Iterator<float> bx(base_cloud, "x"), by(base_cloud, "y"),
  bz(base_cloud, "z");
  for (auto & point : xyz) {
    Eigen::Vector3d transformed = base_camera * point;
    *bx = transformed.x(); *by = transformed.y(); *bz = transformed.z(); ++bx; ++by; ++bz;
  }
  base_cloud_pub_->publish(base_cloud);
  mask_pub_->publish(mask);
  auto preview = mask;
  for (auto & value : preview.data) {
    value = value ? 255 : 0;
  }
  preview_pub_->publish(preview);
  auto overlay = capture->rgb;
  if ((overlay.encoding == "rgb8" || overlay.encoding == "bgr8") && overlay.width == mask.width &&
    overlay.height == mask.height && overlay.step >= overlay.width * 3 &&
    overlay.data.size() >= overlay.step * overlay.height)
  {
    for (uint32_t v = 0; v < mask.height; ++v) {
      for (uint32_t u = 0; u < mask.width; ++u) {
        if (!mask.data[v * mask.step + u]) {continue;}
        for (int c = 0; c < 3; ++c) {
          auto & value = overlay.data[v * overlay.step + u * 3 + c];
          value = static_cast<uint8_t>(std::round(value * .45 + (c == 1 ? 255 : 0) * .55));
        }
      }
    }
    overlay_pub_->publish(overlay);
  }
  auto markers = clear_markers(display_header);
  int debug_count = parameter<int>("debug_max_candidates", 10);
  if (debug_count < 0 || debug_count > 100) {
    throw std::invalid_argument("debug_max_candidates must be in [0,100]");
  }
  for (size_t i = 0; i < std::min(candidates.size(), static_cast < size_t > (debug_count)); ++i) {
    auto model = base_camera * pose_matrix(candidates[i].pose.pose);
    markers.markers.push_back(arrow(display_header, "model_candidates", i,
      model.translation(), model.translation() + model.linear().col(2) * .06, 2));
  }
  candidates_pub_->publish(markers); selected_pub_->publish(clear_markers(display_header));
  std::vector<double> axis[3];
  for (auto & p : xyz) {
    auto point = base_camera * p;
    for (int a = 0; a < 3; ++a) {
      axis[a].push_back(point[a]);
    }
  }
  Eigen::Vector3d lower, upper;
  for (int a = 0; a < 3; ++a) {
    lower[a] = quantile(axis[a], .01); upper[a] = quantile(axis[a], .99);
  }
  double object_top = upper.z();
  auto background = mask;
  if (background.encoding != "mono8" || background.step < background.width ||
    background.data.size() != static_cast<size_t>(background.step) * background.height)
  {
    throw std::invalid_argument("Invalid segmentation mask");
  }
  for (uint32_t v = 0; v < background.height; ++v) {
    for (uint32_t u = 0; u < background.width; ++u) {
      auto & value = background.data[v * background.step + u]; value = value ? 0 : 255;
    }
  }
  auto environment = cloud_points(project_object_cloud(*capture, background));
  std::vector<double> z;
  Eigen::Vector2d center_xy = ((lower + upper) / 2).head<2>();
  for (auto & p : environment) {
    Eigen::Vector3d point = base_camera * p;
    if ((point.head<2>() - center_xy).norm() < .35 && point.z() < upper.z() - .005) {
      z.push_back(point.z());
    }
  }
  if (z.size() < 100) {
    throw ActionFailure(ErrorCode::GEOMETRY_INVALID, "bt", "GEOMETRY_INVALID",
        "SelectFeasibleGrasp", "No observed nearby support plane; reobserve");
  }
  std::map<int, int> histogram;
  for (double value : z) {
    ++histogram[static_cast<int>(std::floor(value / .005))];
  }
  auto dominant = std::max_element(
    histogram.begin(), histogram.end(), [](auto a, auto b) {
      return a.second < b.second;
    });
  std::vector<double> plane;
  for (double value : z) {
    if (std::abs(value - (dominant->first + .5) * .005) < .006) {
      plane.push_back(value);
    }
  }
  if (plane.size() < 100 || plane.size() < .15 * z.size()) {
    throw ActionFailure(ErrorCode::GEOMETRY_INVALID, "bt", "GEOMETRY_INVALID",
        "SelectFeasibleGrasp", "Uncertain support plane");
  }
  double support = quantile(plane, .5);
  lower.z() = support;
  Eigen::Vector3d dimensions = upper - lower + Eigen::Vector3d::Constant(.006);
  if ((dimensions.array() < .008).any() || (dimensions.array() > .3).any() ||
    upper.z() - support <= .01 || upper.z() - support >= .3)
  {
    throw std::runtime_error("Observed bounds outside tabletop range");
  }
  Eigen::Vector3d center = (upper + lower) / 2;
  double inset = number("grasp_width_closing_inset", .002, 0, .005);
  double max_opening = number("gripper_open_width", .10, .001, .10);
  double top_angle = number("grasp_top_down_max_angle_deg", 55, 0, 180);
  double clearance = number("grasp_table_clearance_m", .01, 0, .3);
  double margin = number("grasp_target_margin_m", .05, 0, .3);
  bool height_filter = parameter<bool>("grasp_height_filter_enabled", false);
  double height_low = number("grasp_height_above_object_min_m", 0, 0, .5);
  double height_high = number("grasp_height_above_object_max_m", 0, height_low, .5);
  if (!std::isfinite(pre_distance) || pre_distance < .02 || pre_distance > .2 ||
    !std::isfinite(lift_distance) || lift_distance < .065 || lift_distance > .2)
  {
    throw std::invalid_argument("Invalid pregrasp/lift distance");
  }
  auto xyz_offset = parameter<std::vector<double>>("model_to_tcp_xyz", {0, 0, .13});
  auto rpy = parameter<std::vector<double>>("model_to_tcp_rpy", {0, 0, -M_PI / 2});
  if (xyz_offset.size() != 3 || rpy.size() != 3) {
    throw std::invalid_argument("Model/TCP transform requires XYZ and RPY triples");
  }
  Eigen::Isometry3d model_tcp = Eigen::Isometry3d::Identity();
  model_tcp.translation() = Eigen::Vector3d(xyz_offset[0], xyz_offset[1], xyz_offset[2]);
  model_tcp.linear() = (Eigen::AngleAxisd(rpy[2],
      Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy[1],
      Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy[0],
      Eigen::Vector3d::UnitX())).toRotationMatrix();
  if (!model_tcp.matrix().allFinite()) {
    throw std::invalid_argument("Nonfinite model/TCP transform");
  }
  auto start = state(cancel);
  auto current = pose_matrix(fk(start, cancel));
  Constraints wrist;
  if (parameter<bool>("moveit_pregrasp_wrist_constraint_enabled", true)) {
    double tolerance = number("moveit_pregrasp_wrist_max_deviation_deg", 100, .001,
        179.999) * M_PI / 180;
    auto positions = joint_values(start.joint_state);
    for (auto name : {"joint4", "joint6"}) {
      moveit_msgs::msg::JointConstraint c;
      c.joint_name = name; c.position = positions.at(name);
      c.tolerance_above = c.tolerance_below = tolerance; c.weight = 1;
      wrist.joint_constraints.push_back(c);
    }
  }
  struct Variant {size_t index; bool flipped; double angle; Eigen::Isometry3d grasp;};
  std::vector<Variant> variants;
  int maximum = parameter<int>("max_candidates", 1000);
  if (maximum < 1 || maximum > 3000) {
    throw std::invalid_argument("max_candidates outside [1,3000]");
  }
  bool prefer = parameter<bool>("moveit_prefer_current_grasp_orientation", true);
  for (size_t i = 0; i < std::min(candidates.size(), static_cast < size_t > (maximum)); ++i) {
    auto model = base_camera * pose_matrix(candidates[i].pose.pose);
    for (bool flipped : {false, true}) {
      Eigen::Isometry3d g = model;
      if (flipped) {g.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));}
      double angle = prefer ? Eigen::AngleAxisd(current.linear().transpose() *
          (g * model_tcp).linear()).angle() : 0;
      variants.push_back({i, flipped, angle, g});
    }
  }
  std::stable_sort(
    variants.begin(), variants.end(), [](auto & a, auto & b) {
      return a.angle < b.angle;
    });
  object = Object(); object.header.frame_id = base; object.id = TARGET;
  object.operation = Object::ADD;
  shape_msgs::msg::SolidPrimitive box; box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {dimensions.x(), dimensions.y(), dimensions.z()};
  geometry_msgs::msg::Pose object_pose;
  object_pose.position.x = center.x(); object_pose.position.y = center.y();
  object_pose.position.z = center.z(); object_pose.orientation.w = 1;
  object.primitives = {box}; object.primitive_poses = {object_pose};
  auto previous_scene = scene(PC::ALLOWED_COLLISION_MATRIX, cancel);
  auto matrix = previous_scene.allowed_collision_matrix;
  original_pairs.clear();
  for (auto & finger : FINGERS) {
    auto a = std::find(matrix.entry_names.begin(), matrix.entry_names.end(), finger);
    auto b = std::find(matrix.entry_names.begin(), matrix.entry_names.end(), TARGET);
    original_pairs[finger] = a != matrix.entry_names.end() && b != matrix.entry_names.end() &&
      matrix.entry_values.at(a - matrix.entry_names.begin()).enabled.at(b -
        matrix.entry_names.begin());
    set_pair(matrix, finger, TARGET, true);
  }
  scene_active = true;
  moveit_msgs::msg::PlanningScene update;
  update.world.collision_objects = {object}; update.allowed_collision_matrix = matrix;
  apply(update, cancel);
  if (node->get_parameter("octomap_enabled").as_bool()) {
    rwh_beyond_msgs::srv::SetGraspRegion::Request region;
    region.enabled = true; region.frame_id = base;
    region.lower.x = center.x() - dimensions.x() / 2 - .003;
    region.lower.y = center.y() - dimensions.y() / 2 - .003; region.lower.z = support + .003;
    region.upper.x = center.x() + dimensions.x() / 2 + .003;
    region.upper.y = center.y() + dimensions.y() / 2 + .003;
    region.upper.z = center.z() + dimensions.z() / 2 + .003;
    if (!call<rwh_beyond_msgs::srv::SetGraspRegion>(region_, region, cancel)->success) {
      throw std::runtime_error("Target region exclusion failed");
    }
    call<std_srvs::srv::Empty>(clear_, std_srvs::srv::Empty::Request(), cancel);
    uint64_t seq;
    auto cleared = node->now().nanoseconds();
    {std::lock_guard<std::mutex> lock(mutex_); seq = cloud_seq_;}
    wait(
      [&]() {
        std::lock_guard<std::mutex> lock(mutex_);
        return cloud_seq_ >= seq + 2 && cloud_stamp_ > cleared;
    },
        20, cancel);
    wait(
      [&]() {
        return !scene(PC::OCTOMAP, cancel).world.octomap.octomap.data.empty();
        }, 15, cancel);
  }
  std::map<std::string, int> rejected;
  for (auto & variant : variants) {
    ++timing.variants;
    if (cancel->load()) {throw std::runtime_error("Selection canceled");}
    auto g = variant.grasp;
    auto model = base_camera * pose_matrix(candidates[variant.index].pose.pose);
    if (model.linear()(2, 2) > -std::cos(top_angle * M_PI / 180)) {
      ++rejected["not_top_down"]; continue;
    }
    Eigen::Vector3d contact = model * Eigen::Vector3d(0, 0, .105);
    bool outside = false;
    for (int a = 0; a < (height_filter ? 2 : 3); ++a) {
      outside |= std::abs(contact[a] - center[a]) > dimensions[a] / 2 + margin;
    }
    if (outside) {++rejected["off_target"]; continue;}
    Eigen::Isometry3d tcp_goal = g * model_tcp;
    if (height_filter &&
      (tcp_goal.translation().z() - object_top < height_low - 1e-9 ||
      tcp_goal.translation().z() - object_top > height_high + 1e-9))
    {
      ++rejected["height"]; continue;
    }
    if (tcp_goal.translation().z() < support + clearance) {++rejected["table_clearance"]; continue;}
    double width = 0, close = number("gripper_close_width", .04, .001, .10);
    if (estimated_width) {
      try {
        auto aperture = estimate_aperture(xyz, pose_matrix(candidates[variant.index].pose.pose),
            max_opening, inset); width = aperture.first; close = aperture.second;
      } catch (const std::invalid_argument &) {
        ++rejected["invalid_observed_width"]; continue;
      }
    }
    Eigen::Isometry3d pre = tcp_goal; pre.translation() -= g.linear().col(2) * pre_distance;
    auto goal = timed_call(timing.ik, timing.ik_calls,
        [&]() {return ik(matrix_pose(pre), start, cancel, true);});
    if (goal.joint_state.name.empty()) {++rejected["pre_ik"]; continue;}
    auto positions = joint_values(goal.joint_state);
    bool wrist_ok = true;
    for (auto & c : wrist.joint_constraints) {
      double q = positions.at(c.joint_name);
      wrist_ok &= q >= c.position - c.tolerance_below && q <= c.position + c.tolerance_above;
    }
    if (!wrist_ok) {++rejected["wrist_goal"]; continue;}
    auto approach = timed_call(timing.cartesian, timing.cartesian_calls,
        [&]() {return cartesian(matrix_pose(tcp_goal), goal, cancel);});
    if (approach->error_code.val != 1 || !std::isfinite(approach->fraction) ||
      approach->fraction < .999 || !trajectory_valid(approach->solution, wrist))
    {
      if (approach->error_code.val != 1) {
        candidate_error = moveit_failure(approach->error_code, "/compute_cartesian_path");
      }
      ++rejected["approach_path"]; continue;
    }
    State grasp_state = goal;
    auto & j = approach->solution.joint_trajectory;
    Joints end;
    for (size_t i = 0; i < j.joint_names.size(); ++i) {
      end[j.joint_names[i]] = j.points.back().positions[i];
    }
    for (size_t i = 0; i < grasp_state.joint_state.name.size(); ++i) {
      if (end.count(grasp_state.joint_state.name[i])) {
        grasp_state.joint_state.position[i] = end.at(grasp_state.joint_state.name[i]);
      }
    }
    auto lift_pose = matrix_pose(tcp_goal); lift_pose.position.z += lift_distance;
    auto lift = timed_call(timing.cartesian, timing.cartesian_calls,
        [&]() {return cartesian(lift_pose, grasp_state, cancel);});
    if (lift->error_code.val != 1 || !std::isfinite(lift->fraction) || lift->fraction < .999 ||
      !trajectory_valid(lift->solution, wrist))
    {
      if (lift->error_code.val != 1) {
        candidate_error = moveit_failure(lift->error_code, "/compute_cartesian_path");
      }
      ++rejected["lift_path"]; continue;
    }
    auto trajectory = timed_call(timing.planning, timing.planning_calls,
        [&]() {return plan(start, goal, wrist, cancel, true);});
    if (!trajectory_valid(trajectory, wrist)) {++rejected["pre_plan"]; continue;}
    selected = std::make_shared<GraspPlan>();
    selected->capture = capture; selected->grasp = matrix_pose(tcp_goal);
    selected->pregrasp = matrix_pose(pre);
    selected->trajectory = trajectory; selected->wrist = wrist; selected->close_width = close;
    selected->observed_width = width; selected->lift_distance = lift_distance;
    publish_selection(selected);
    RCLCPP_INFO(node->get_logger(),
        "BT selected candidate=%zu flipped=%d rotation_deg=%.1f close_width=%.4f", variant.index,
        variant.flipped, variant.angle * 180 / M_PI, close);
    timing.selected = true;
    return selected;
  }
  std::string reasons;
  for (auto & pair : rejected) {
    reasons += pair.first + "=" + std::to_string(pair.second) + " ";
  }
  if (candidate_error) {
    throw ActionFailure(candidate_error->code, candidate_error->domain, candidate_error->name,
      candidate_error->source,
        std::string(candidate_error->what()) + "; No feasible model grasp: " + reasons);
  }
  throw ActionFailure(ErrorCode::NO_FEASIBLE_GRASP, "bt", "NO_FEASIBLE_GRASP",
    "SelectFeasibleGrasp", "No feasible model grasp: " + reasons);
}
}  // namespace rwh_beyond_bt
