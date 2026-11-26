// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#ifndef RWH_BEYOND_BT__MOTION_CONTEXT_HPP_
#define RWH_BEYOND_BT__MOTION_CONTEXT_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include "rwh_beyond_bt/diagnostics.hpp"
#include "rwh_beyond_bt/multiview.hpp"
#include "Eigen/Geometry"
#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "moveit_msgs/srv/get_position_fk.hpp"
#include "moveit_msgs/srv/get_cartesian_path.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rwh_beyond_msgs/srv/set_grasp_region.hpp"
#include "rwh_beyond_bt/plugins/action/perception_actions.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rwh_beyond_bt
{
using State = moveit_msgs::msg::RobotState;
using Trajectory = moveit_msgs::msg::RobotTrajectory;
using Constraints = moveit_msgs::msg::Constraints;
using Joints = std::map<std::string, double>;
using Points = std::vector<Eigen::Vector3d>;
using Cancel = std::shared_ptr<std::atomic_bool>;
enum class PlanningProfile {DEFAULT, OBSERVATION};
struct GraspPlan
{
  enum class Phase {SELECTED, PREGRASP, APPROACHED, ATTACHED, LIFTED};
  Observation capture;
  geometry_msgs::msg::Pose pregrasp, grasp;
  Trajectory trajectory;
  Constraints wrist;
  double observed_width{0}, close_width{0}, lift_distance{0.08};
  bool has_approach{true};
  Phase phase{Phase::SELECTED};
};
using Plan = std::shared_ptr<GraspPlan>;

Eigen::Isometry3d pose_matrix(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Pose matrix_pose(const Eigen::Isometry3d & matrix);
double quantile(std::vector<double> values, double probability);
Joints joint_values(const sensor_msgs::msg::JointState & joints);
bool trajectory_valid(const Trajectory & trajectory, const Constraints & wrist = Constraints());
bool trajectory_reaches(
  const Trajectory & trajectory, const State & target,
  double tolerance = .002);
std::pair<double, double> estimate_aperture(
  const Points & points, const Eigen::Isometry3d & grasp, double maximum, double inset);

// Shared resources owned by the BT Blackboard. No ROS server or stage dispatcher.
class MotionContext : public std::enable_shared_from_this<MotionContext>
{
public:
  explicit MotionContext(rclcpp::Node::SharedPtr node);
  ~MotionContext();
  static std::shared_ptr<MotionContext> get(const BT::Blackboard::Ptr & board);
  rclcpp::Node::SharedPtr node;
  std::string base, tcp;
  bool simulation, attached{false}, scene_active{false}, release_ready{false};
  moveit_msgs::msg::CollisionObject object;
  std::map<std::string, bool> original_pairs;
  Plan selected;
  std::weak_ptr<BT::Blackboard> diagnostic_board;
  std::optional<ActionFailure> candidate_error;
  void status(const std::string & source, const std::string & stage);

  std::mutex operation;  // Parallel BT branches cannot command the same robot.

  template<class T> T parameter(const std::string & name, const T & fallback)
  {
    const auto key = "bt_motion." + name;
    if (!node->has_parameter(key)) {node->declare_parameter<T>(key, fallback);}
    return node->get_parameter(key).get_value<T>();
  }
  double number(const std::string & name, double fallback, double low, double high);
  void wait(const std::function<bool()> & predicate, double timeout, const Cancel & cancel);
  template<class Service>
  typename Service::Response::SharedPtr call(
    const typename rclcpp::Client<Service>::SharedPtr & client,
    const typename Service::Request & request, const Cancel & cancel, double timeout = 15)
  {
    const std::string source = client->get_service_name();
    status(source, "waiting_for_service");
    try {
      wait([&]() {
          return client->service_is_ready();
                                                       }, timeout, cancel);
    } catch (const ActionFailure & e) {
      throw ActionFailure(e.code, "transport", e.name, source, e.what());
    }
    status(source, "waiting_for_response");
    auto future = client->async_send_request(std::make_shared<typename Service::Request>(request));
    try {
      wait([&]() {return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;},
        timeout, cancel);
      return future.get();
    } catch (const ActionFailure & e) {
      client->remove_pending_request(future);
      throw ActionFailure(e.code, "transport", e.name, source, e.what());
    } catch (...) {client->remove_pending_request(future); throw;}
  }
  Joints feedback();
  State state(const Cancel & cancel);
  void ensure_backend(const Cancel & cancel);
  void check_ready(bool release_previous, bool check_direct_models, const Cancel & cancel);
  geometry_msgs::msg::Pose fk(const State & state, const Cancel & cancel);
  State ik(
    const geometry_msgs::msg::Pose & pose, const State & start, const Cancel & cancel,
    bool candidate = false, PlanningProfile profile = PlanningProfile::DEFAULT);
  moveit_msgs::srv::GetCartesianPath::Response::SharedPtr cartesian(
    const geometry_msgs::msg::Pose & pose, const State & start, const Cancel & cancel);
  Trajectory plan(
    const State & start, const State & goal, const Constraints & wrist,
    const Cancel & cancel, bool candidate = false,
    PlanningProfile profile = PlanningProfile::DEFAULT);
  void execute(const Trajectory & trajectory, const Cancel & cancel);
  void execute_cartesian(
    const geometry_msgs::msg::Pose & pose,
    const Constraints & wrist, const Cancel & cancel);
  void gripper(double opening, double duration, bool closing, const Cancel & cancel);
  moveit_msgs::msg::PlanningScene scene(uint32_t components, const Cancel & cancel);
  void apply(moveit_msgs::msg::PlanningScene scene, const Cancel & cancel);
  void cleanup(const Cancel & cancel);
  void prepare_motion(const Cancel & cancel);
  void release(double opening, double duration, bool repeat, const Cancel & cancel);
  void validate_capture(const Observation & capture);
  Eigen::Isometry3d camera_pose_at(const Observation & capture, const Cancel & cancel);
  Plan select(
    const Observation & capture, const sensor_msgs::msg::Image & mask,
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::vector<rwh_beyond_msgs::msg::GraspCandidate> & candidates,
    bool estimated_width, double pre_distance, double lift_distance, const Cancel & cancel,
    const Fusion & fusion = Fusion());
  void publish_selection(const Plan & plan);

private:
  void detach_target(const Cancel & cancel);
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spinner_;
  std::mutex mutex_;
  sensor_msgs::msg::JointState joints_;
  std::chrono::steady_clock::time_point received_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr simulation_reset_sub_;
  std::atomic<uint64_t> simulation_reset_{0};
  uint64_t handled_simulation_reset_{0};
  uint64_t cloud_seq_{0};
  int64_t cloud_stamp_{0};
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::map<std::string, std::pair<double, double>> limits_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_;
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_;
  rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr plan_;
  rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr cartesian_;
  rclcpp::Client<rwh_beyond_msgs::srv::SetGraspRegion>::SharedPtr region_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hardware_, sam_, model_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr description_;
  rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SharedPtr execute_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr gripper_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pub_, pre_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_, preview_pub_, overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr base_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidates_pub_, selected_pub_;
};
}  // namespace rwh_beyond_bt
#endif  // RWH_BEYOND_BT__MOTION_CONTEXT_HPP_
