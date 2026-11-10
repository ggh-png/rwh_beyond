#include <chrono>
#include <cmath>
#include <future>
#include <set>

#include "behaviortree_cpp/action_node.h"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rwh_beyond_bt/diagnostics.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"

namespace rwh_beyond_bt
{
using Clear = std_srvs::srv::Empty;
using Scene = moveit_msgs::srv::GetPlanningScene;

class ClearOctomap : public BT::StatefulActionNode
{
public:
  ClearOctomap(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config),
    node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
  {
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("once", true, "Skip after successful initialization for this node instance"),
      BT::InputPort<double>("timeout", 30.0, "Total wall-time limit in seconds"),
      BT::InputPort<int>("fresh_clouds", 2, "Distinct nonempty depth clouds required after clear"),
      BT::InputPort<std::string>("service_name", "/clear_octomap", "MoveIt clear service"),
      BT::InputPort<std::string>("cloud_topic", "/camera/depth/filtered_points", "Robot-filtered depth XYZ"),
      BT::InputPort<std::string>("scene_service", "/get_planning_scene", "MoveIt planning scene service"),
    };
  }

  BT::NodeStatus onStart() override
  {
    beginStage(config().blackboard, "clear_octomap", name());
    cleanup();
    try {
      const auto once = getInput<bool>("once");
      const auto timeout = getInput<double>("timeout");
      const auto count = getInput<int>("fresh_clouds");
      const auto service = getInput<std::string>("service_name");
      const auto topic = getInput<std::string>("cloud_topic");
      const auto scene = getInput<std::string>("scene_service");
      if (!once || !timeout || !count || !service || !topic || !scene ||
        !std::isfinite(timeout.value()) || timeout.value() <= 0.0 || count.value() < 1 ||
        service.value().empty() || topic.value().empty() || scene.value().empty())
      {
        return failure(-1000, "INVALID_INPUT", "Valid service/topic names, positive timeout and fresh_clouds >= 1 are required");
      }
      if (once.value() && initialized_) {return BT::NodeStatus::SUCCESS;}
      initialized_ = false;
      required_ = count.value();
      deadline_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout.value()));
      clear_ = node_->create_client<Clear>(service.value(), rclcpp::ServicesQoS(), group_);
      scene_ = node_->create_client<Scene>(scene.value(), rclcpp::ServicesQoS(), group_);
      rclcpp::SubscriptionOptions options;
      options.callback_group = group_;
      cloud_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic.value(), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          if (phase_ != Phase::CLOUDS && phase_ != Phase::SCENE) {return;}
          const int64_t stamp = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
            msg->header.stamp.nanosec;
          if (stamp > cleared_at_ && msg->width > 0 && msg->height > 0) {stamps_.insert(stamp);}
        }, options);
      phase_ = Phase::SERVICES;
      return BT::NodeStatus::RUNNING;
    } catch (const std::exception & error) {
      return failure(-1000, "INVALID_INPUT", error.what());
    }
  }

  BT::NodeStatus onRunning() override
  {
    try {
      executor_.spin_some();
      if (std::chrono::steady_clock::now() >= deadline_) {
        return failure(-1100, "OCTOMAP_TIMEOUT", "Timed out while " + phaseDescription());
      }
      if (phase_ == Phase::SERVICES && clear_->service_is_ready() && scene_->service_is_ready()) {
        // Exactly one request per activation; never retry an ambiguous timeout.
        clear_future_ = clear_->async_send_request(std::make_shared<Clear::Request>()).share();
        phase_ = Phase::CLEAR;
      } else if (phase_ == Phase::CLEAR && ready(clear_future_)) {
        clear_future_.get();
        cleared_at_ = node_->now().nanoseconds();
        stamps_.clear();
        last_query_stamp_ = cleared_at_;
        phase_ = Phase::CLOUDS;
        RCLCPP_INFO(node_->get_logger(), "%s: Octomap cleared; waiting for fresh depth", name().c_str());
      } else if (phase_ == Phase::CLOUDS && static_cast<int>(stamps_.size()) >= required_ &&
        *stamps_.rbegin() > last_query_stamp_)
      {
        auto request = std::make_shared<Scene::Request>();
        request->components.components = moveit_msgs::msg::PlanningSceneComponents::OCTOMAP;
        scene_future_ = scene_->async_send_request(request).share();
        last_query_stamp_ = *stamps_.rbegin();
        phase_ = Phase::SCENE;
      } else if (phase_ == Phase::SCENE && ready(scene_future_)) {
        const auto response = scene_future_.get();
        if (!response->scene.world.octomap.octomap.data.empty()) {
          initialized_ = true;
          RCLCPP_INFO(node_->get_logger(), "%s: fresh Octomap ready", name().c_str());
          cleanup();
          return BT::NodeStatus::SUCCESS;
        }
        phase_ = Phase::CLOUDS;
      }
      return BT::NodeStatus::RUNNING;
    } catch (const std::exception & error) {
      return failure(-1101, "OCTOMAP_SERVICE_ERROR", error.what());
    }
  }

  void onHalted() override
  {
    // An already-sent Empty service cannot be canceled; halt never marks it ready.
    cleanup();
  }

private:
  enum class Phase {IDLE, SERVICES, CLEAR, CLOUDS, SCENE};

  template<class Future>
  static bool ready(const Future & future)
  {
    return future.valid() && future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }

  std::string phaseDescription() const
  {
    switch (phase_) {
      case Phase::SERVICES: return "waiting for MoveIt clear/scene services";
      case Phase::CLEAR: return "waiting for the clear service response";
      case Phase::CLOUDS: return "waiting for fresh post-clear depth clouds";
      case Phase::SCENE: return "checking the rebuilt Octomap";
      default: return "initializing the Octomap";
    }
  }

  BT::NodeStatus failure(int32_t code, const std::string & error, const std::string & message)
  {
    reportFailure(config().blackboard, code, error, "ClearOctomap", message);
    RCLCPP_ERROR(node_->get_logger(), "%s: %s: %s", name().c_str(), error.c_str(), message.c_str());
    cleanup();
    return BT::NodeStatus::FAILURE;
  }

  void cleanup()
  {
    phase_ = Phase::IDLE;
    cloud_.reset();
    clear_.reset();
    scene_.reset();
    clear_future_ = {};
    scene_future_ = {};
    stamps_.clear();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Clear>::SharedPtr clear_;
  rclcpp::Client<Scene>::SharedPtr scene_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Client<Clear>::SharedFuture clear_future_;
  rclcpp::Client<Scene>::SharedFuture scene_future_;
  std::chrono::steady_clock::time_point deadline_;
  std::set<int64_t> stamps_;
  int64_t cleared_at_{0}, last_query_stamp_{0};
  int required_{2};
  bool initialized_{false};
  Phase phase_{Phase::IDLE};
};

void register_clear_octomap(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<ClearOctomap>("ClearOctomap");
}
}  // namespace rwh_beyond_bt
