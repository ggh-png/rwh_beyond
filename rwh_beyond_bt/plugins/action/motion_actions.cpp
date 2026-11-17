// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/motion_context.hpp"
#include <cmath>
#include <algorithm>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "behaviortree_cpp/bt_factory.h"
#include "rwh_beyond_bt/diagnostics.hpp"
#include "rwh_beyond_bt/tcp_pose.hpp"

namespace rwh_beyond_bt
{
enum class Step {READY, SELECT, PREGRASP, APPROACH, ATTACH, LIFT, HOME, OBSERVE, PLACE, OPEN,
  RELEASE};

// Each BT activation owns its asynchronous work and cancellation token. All ROS
// calls originate here or in the shared client utilities, never a stage server.
class MotionAction : public BT::StatefulActionNode
{
public:
  MotionAction(const std::string & name, const BT::NodeConfiguration & config, Step step)
  : BT::StatefulActionNode(name, config), step_(step),
    context_(MotionContext::get(config.blackboard)) {}
  ~MotionAction() override {stop();}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Plan>("plan"), BT::OutputPort<Plan>("selected_plan"),
      BT::InputPort<Observation>("capture"),
      BT::InputPort<sensor_msgs::msg::Image>("mask"),
      BT::InputPort<sensor_msgs::msg::PointCloud2>("cloud"),
      BT::InputPort<std::vector<rwh_beyond_msgs::msg::GraspCandidate>>("candidates"),
      BT::InputPort<bool>("use_estimated_width", false, "Estimate close width from point cloud"),
      BT::InputPort<bool>("release_previous", false, "Explicitly release prior payload in place"),
      BT::InputPort<double>("pregrasp_distance", "Approach offset (m); omit for bt_motion YAML"),
      BT::InputPort<double>("lift_distance", "Lift distance (m); omit for bt_motion YAML"),
      BT::InputPort<std::vector<double>>("home_joint_positions", "Six joint angles (rad)"),
      BT::InputPort<double>("x", "Base X offset (m)"),
      BT::InputPort<double>("y", "Base Y offset (m)"),
      BT::InputPort<double>("z", "Base Z offset (m)"),
      BT::InputPort<double>("tilt_deg", "Home-local Y tilt (deg)"),
      BT::InputPort<double>("settle_time", "Observation settling time (s)"),
      BT::InputPort<double>("opening", "Gripper total gap (m)"),
      BT::InputPort<double>("duration", "Gripper duration (s)"),
      BT::OutputPort<TcpPose>("pregrasp"), BT::OutputPort<TcpPose>("grasp_pose"),
      BT::OutputPort<double>("grasp_width"), BT::OutputPort<double>("close_width")};
  }
  BT::NodeStatus onStart() override
  {
    begin_stage(config().blackboard, registrationName(), name());
    try {
      // Resolve every Blackboard input on the BT thread before starting work.
      selected_.reset(); input_plan_.reset();
      if (step_ == Step::SELECT) {
        capture_ = required<Observation>("capture");
        mask_ = required<sensor_msgs::msg::Image>("mask");
        cloud_ = required<sensor_msgs::msg::PointCloud2>("cloud");
        candidates_ = required<std::vector<rwh_beyond_msgs::msg::GraspCandidate>>("candidates");
        estimated_ = required<bool>("use_estimated_width");
        pre_ = scalar("pregrasp_distance", context_->number("pregrasp_distance", .1, .02, .2), .02,
            .2);
        lift_ = scalar("lift_distance", context_->number("lift_distance", .08, .065, .2), .065, .2);
      }
      if (step_ == Step::PREGRASP || step_ == Step::APPROACH || step_ == Step::ATTACH ||
        step_ == Step::LIFT)
      {
        input_plan_ = required<Plan>("plan");
        if (!input_plan_) {throw std::invalid_argument("Null grasp plan");}
      }
      repeat_ = step_ == Step::READY ? required<bool>("release_previous") : false;
      home_ = context_->parameter<std::vector<double>>("home_joint_positions", {0, 0, 0, 0, 0, 0});
      if (config().input_ports.count("home_joint_positions")) {
        home_ = required<std::vector<double>>("home_joint_positions");
      }
      if (home_.size() != 6) {throw std::invalid_argument("Home requires six finite radians");}
      for (double q : home_) {
        if (!std::isfinite(q)) {
          throw std::invalid_argument("Invalid home joint");
        }
      }
      x_ = scalar("x", 0, -.30, .30); y_ = scalar("y", 0, -.30, .30);
      z_ = scalar("z",
          step_ == Step::OBSERVE ? context_->number("observation_z_offset", .2, -.3, .3) : 0,
          step_ == Step::PLACE ? 0 : -.3, .3);
      tilt_ = context_->parameter<std::vector<double>>("observation_tilt_rpy_deg", {0, 30, 0});
      if (tilt_.size() != 3) {throw std::invalid_argument("Observation RPY requires three angles");}
      for (double q : tilt_) {
        if (!std::isfinite(q) || std::abs(q) > 90) {
          throw std::invalid_argument("Observation tilt outside +/-90 deg");
        }
      }
      tilt_[1] = scalar("tilt_deg", tilt_[1], -90, 90);
      settle_ = scalar("settle_time", context_->number("observation_settle_time", .5, 0, 30), 0,
          30);
      opening_ = scalar("opening", context_->number("gripper_open_width", .10, 0, .10), 0, .10);
      duration_ = scalar("duration", 3, .001, 30);
      cancel_ = std::make_shared<std::atomic_bool>(false);
      future_ = std::async(std::launch::async, [this]() {
            std::unique_lock<std::mutex> lock(context_->operation, std::try_to_lock);
            if (!lock.owns_lock()) {
              throw std::runtime_error("Concurrent BT robot operation refused");
            }
            try {perform();} catch (...) {
              try {
                context_->cleanup(std::make_shared<std::atomic_bool>(false));
              } catch (const std::exception & e) {
                RCLCPP_ERROR(context_->node->get_logger(), "BT scene cleanup: %s", e.what());
              }
              throw;
            }
      });
      return BT::NodeStatus::RUNNING;
    } catch (const std::exception & e) {return failure(e.what());}
  }
  BT::NodeStatus onRunning() override
  {
    if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    try {
      future_.get();
      if (selected_) {
        setOutput("selected_plan", selected_);
        TcpPose pose; pose.frame_id = context_->base; pose.pose = selected_->pregrasp;
        setOutput("pregrasp", pose);
        pose.pose = selected_->grasp; setOutput("grasp_pose", pose);
        setOutput("grasp_width", selected_->observed_width);
        setOutput("close_width", selected_->close_width);
      }
      return BT::NodeStatus::SUCCESS;
    } catch (const std::exception & e) {return failure(e.what());}
  }
  void onHalted() override {stop();}

private:
  template<class T> T required(const std::string & port)
  {
    auto result = getInput<T>(port);
    if (!result) {throw std::invalid_argument("Missing/invalid " + port + ": " + result.error());}
    return result.value();
  }
  double scalar(const std::string & port, double fallback, double low, double high)
  {
    double value = config().input_ports.count(port) ? required<double>(port) : fallback;
    if (!std::isfinite(value) || value < low || value > high) {
      throw std::invalid_argument("Invalid " + port);
    }
    return value;
  }
  void stop()
  {
    if (cancel_) {cancel_->store(true);}
    if (future_.valid()) {try {future_.get();} catch (...) {}}
  }
  BT::NodeStatus failure(const std::string & message)
  {
    report_failure(config().blackboard, -1201, "BT_MOTION_FAILED", registrationName(), message);
    RCLCPP_ERROR(context_->node->get_logger(), "%s: %s", name().c_str(), message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  State home_state(const State & state)
  {
    State result = state;
    for (size_t i = 0; i < result.joint_state.name.size(); ++i) {
      for (int j = 1; j <= 6; ++j) {
        if (result.joint_state.name[i] == "joint" + std::to_string(j)) {
          result.joint_state.position[i] = home_[j - 1];
        }
      }
    }
    return result;
  }
  bool at_home()
  {
    auto values = context_->feedback();
    double tolerance = context_->number("home_joint_tolerance", .01, .001, .1);
    for (int i = 1; i <= 6; ++i) {
      if (std::abs(values.at("joint" + std::to_string(i)) - home_[i - 1]) > tolerance) {
        return false;
      }
    }
    return true;
  }
  void phase(GraspPlan::Phase expected)
  {
    if (input_plan_ != context_->selected || input_plan_->phase != expected) {
      throw std::runtime_error("Grasp plan is stale or BT motion stages are out of order");
    }
  }
  void perform()
  {
    auto & ctx = *context_;
    switch (step_) {
      case Step::READY: ctx.check_ready(repeat_, cancel_); break;
      case Step::SELECT:
        selected_ = ctx.select(capture_, mask_, cloud_, candidates_, estimated_, pre_, lift_,
          cancel_); break;
      case Step::PREGRASP:
        phase(GraspPlan::Phase::SELECTED); ctx.validate_capture(input_plan_->capture);
        ctx.execute(input_plan_->trajectory, cancel_);
        input_plan_->phase = GraspPlan::Phase::PREGRASP; break;
      case Step::APPROACH:
        phase(GraspPlan::Phase::PREGRASP);
        ctx.execute_cartesian(input_plan_->grasp, input_plan_->wrist, cancel_);
        input_plan_->phase = GraspPlan::Phase::APPROACHED; break;
      case Step::ATTACH: {
          phase(GraspPlan::Phase::APPROACHED);
          // An interrupted close can still hold an object. Preserve geometry and
          // prohibit automatic opening/cleanup until an explicit recovery.
          ctx.attached = true;
          ctx.gripper(input_plan_->close_width, ctx.number("gripper_close_duration", 2, .001, 30),
            true, cancel_);
          moveit_msgs::msg::PlanningScene update;
          moveit_msgs::msg::AttachedCollisionObject attachment;
          attachment.link_name = ctx.tcp; attachment.object = ctx.object;
          attachment.touch_links = {"gripper_link1", "gripper_link2", ctx.tcp};
          update.robot_state.attached_collision_objects = {attachment}; ctx.apply(update, cancel_);
          ctx.release_ready = false; input_plan_->phase = GraspPlan::Phase::ATTACHED;
          break;
        }
      case Step::LIFT: {
          phase(GraspPlan::Phase::ATTACHED);
          if (!ctx.attached) {throw std::runtime_error("No payload attachment for lift");}
          auto pose = ctx.fk(ctx.state(cancel_), cancel_);
          pose.position.z += input_plan_->lift_distance;
          ctx.execute_cartesian(pose, input_plan_->wrist, cancel_);
          input_plan_->phase = GraspPlan::Phase::LIFTED;
          break;
        }
      case Step::HOME: {
          ctx.prepare_motion(cancel_);
          if (at_home()) {break;}
          auto start = ctx.state(cancel_); auto goal = home_state(start);
          ctx.execute(ctx.plan(start, goal, Constraints(), cancel_), cancel_);
          ctx.wait([&]() {return at_home();}, 20, cancel_); break;
        }
      case Step::OBSERVE: {
          ctx.prepare_motion(cancel_);
          if (!ctx.attached && !at_home()) {
            throw std::runtime_error("Move home before observation");
          }
          auto pose = ctx.fk(home_state(ctx.state(cancel_)), cancel_);
          auto transform = pose_matrix(pose);
          transform.translation() += Eigen::Vector3d(x_, y_, z_);
          transform.rotate(Eigen::AngleAxisd(tilt_[2] * M_PI / 180, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(tilt_[1] * M_PI / 180, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(tilt_[0] * M_PI / 180, Eigen::Vector3d::UnitX()));
          ctx.execute_cartesian(matrix_pose(transform), Constraints(), cancel_);
          auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(settle_);
          ctx.wait([&]() {return std::chrono::steady_clock::now() >= until;}, settle_ + 1, cancel_);
          break;
        }
      case Step::PLACE: {
          ctx.release_ready = false; ctx.prepare_motion(cancel_);
          if (!ctx.attached || !at_home() || x_ < 0) {
            throw std::runtime_error(
              "Place requires home, attached target and nonnegative X offset");
          }
          auto start = ctx.state(cancel_); auto pose = ctx.fk(home_state(start), cancel_);
          pose.position.x += x_; pose.position.z += z_;
          auto path = ctx.cartesian(pose, start, cancel_);
          if (path->error_code.val != 1 || !std::isfinite(path->fraction) || path->fraction < 0 ||
            path->fraction > 1) {throw std::runtime_error("Place Cartesian planning failed");}
          if (path->fraction >= .999) {ctx.execute(path->solution, cancel_);} else {
            auto goal = ctx.ik(pose, start, cancel_);
            if (goal.joint_state.name.empty()) {throw std::runtime_error("Place target IK failed");}
            ctx.execute(ctx.plan(start, goal, Constraints(), cancel_), cancel_);
          }
          ctx.release_ready = true; break;
        }
      case Step::OPEN:
        if (ctx.attached) {
          throw std::runtime_error("Refusing automatic opening of attached payload");
        }
        ctx.gripper(opening_, duration_, false, cancel_); break;
      case Step::RELEASE:
        ctx.prepare_motion(cancel_); ctx.release(opening_, duration_, false, cancel_); break;
    }
  }
  Step step_;
  std::shared_ptr<MotionContext> context_;
  Cancel cancel_;
  std::future<void> future_;
  Plan selected_, input_plan_;
  Observation capture_;
  sensor_msgs::msg::Image mask_;
  sensor_msgs::msg::PointCloud2 cloud_;
  std::vector<rwh_beyond_msgs::msg::GraspCandidate> candidates_;
  std::vector<double> home_, tilt_;
  bool estimated_{false}, repeat_{false};
  double pre_{.1}, lift_{.08}, x_{0}, y_{0}, z_{0}, settle_{.5}, opening_{.10}, duration_{3};
};
}  // namespace rwh_beyond_bt

BT_REGISTER_NODES(factory)
{
  using rwh_beyond_bt::MotionAction;
  using rwh_beyond_bt::Step;
  for (auto entry : std::vector<std::pair<std::string, Step>>{
    {"CheckGraspReady", Step::READY}, {"SelectFeasibleGrasp", Step::SELECT},
    {"MoveToPregrasp", Step::PREGRASP}, {"ApproachGrasp", Step::APPROACH},
    {"AttachObject", Step::ATTACH}, {"LiftObject", Step::LIFT}, {"MoveHome", Step::HOME},
    {"MoveObservation", Step::OBSERVE}, {"MovePlace", Step::PLACE},
    {"OpenGripper", Step::OPEN}, {"ReleaseObject", Step::RELEASE}})
  {
    auto manifest = BT::CreateManifest<MotionAction>(entry.first);
    std::vector<std::string> ports;
    switch (entry.second) {
      case Step::READY: ports = {"release_previous"}; break;
      case Step::SELECT: ports = {"capture", "mask", "cloud", "candidates", "use_estimated_width",
          "pregrasp_distance", "lift_distance", "selected_plan", "pregrasp", "grasp_pose",
          "grasp_width", "close_width"}; break;
      case Step::PREGRASP: case Step::APPROACH: case Step::ATTACH: case Step::LIFT: ports =
      {"plan"}; break;
      case Step::HOME: ports = {"home_joint_positions"}; break;
      case Step::OBSERVE: ports = {"home_joint_positions", "x", "y", "z", "tilt_deg",
          "settle_time"}; break;
      case Step::PLACE: ports = {"home_joint_positions", "x", "z"}; break;
      case Step::OPEN: case Step::RELEASE: ports = {"opening", "duration"}; break;
    }
    for (auto it = manifest.ports.begin(); it != manifest.ports.end(); ) {
      if (std::find(ports.begin(), ports.end(), it->first) == ports.end()) {
        it = manifest.ports.erase(it);
      } else {++it;}
    }
    factory.registerBuilder(manifest,
      [step = entry.second](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<MotionAction>(name, config, step);
      });
  }
}
