#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rwh_beyond_msgs/action/run_demo.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace rwh_beyond_navigator
{

class DemoNavigator
  : public nav2_core::BehaviorTreeNavigator<rwh_beyond_msgs::action::RunDemo>
{
public:
  using ActionT = rwh_beyond_msgs::action::RunDemo;

  std::string getName() override
  {
    return "run_demo";
  }

  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) override
  {
    auto node = parent_node.lock();
    const auto package_share = ament_index_cpp::get_package_share_directory("rwh_beyond_bt");
    const auto default_pick_path = package_share + "/behavior_trees/demo.xml";
    const auto default_place_path = package_share + "/behavior_trees/place.xml";
    if (!node->has_parameter("default_demo_bt_xml")) {
      node->declare_parameter("default_demo_bt_xml", default_pick_path);
    }
    if (!node->has_parameter("default_place_bt_xml")) {
      node->declare_parameter("default_place_bt_xml", default_place_path);
    }
    default_pick_bt_ = node->get_parameter("default_demo_bt_xml").as_string();
    default_place_bt_ = node->get_parameter("default_place_bt_xml").as_string();
    return default_pick_bt_;
  }

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> /*odom_smoother*/) override
  {
    auto node = parent_node.lock();
    if (!node) {
      RCLCPP_ERROR(logger_, "Unable to configure Groot2 monitoring: navigator node expired");
      return false;
    }

    cleanup_client_ = node->create_client<std_srvs::srv::Trigger>("grasp/cleanup");
    const auto enabled_param = getName() + ".enable_groot_monitoring";
    const auto port_param = getName() + ".groot_server_port";
    if (!node->has_parameter(enabled_param)) {
      node->declare_parameter(enabled_param, false);
    }
    if (!node->has_parameter(port_param)) {
      node->declare_parameter(port_param, 1667);
    }
    const auto enabled = node->get_parameter(enabled_param).as_bool();
    const auto port = node->get_parameter(port_param).as_int();
    if (port < 1 || port > 65534) {
      RCLCPP_ERROR(
        logger_, "Invalid %s: %lld (must be between 1 and 65534)", port_param.c_str(),
        static_cast<long long>(port));
      return false;
    }
    bt_action_server_->setGrootMonitoring(enabled, static_cast<unsigned>(port));
    if (enabled) {
      RCLCPP_INFO(
        logger_, "Groot2 monitoring enabled on TCP ports %lld and %lld",
        static_cast<long long>(port), static_cast<long long>(port + 1));
    }
    return true;
  }

  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override
  {
    std::string tree = goal->behavior_tree;
    if (tree.empty()) {
      const auto mode = goal->mode == 0 ? ActionT::Goal::MODE_PICK : goal->mode;
      if (mode == ActionT::Goal::MODE_PICK) {
        tree = default_pick_bt_;
      } else if (mode == ActionT::Goal::MODE_PLACE) {
        tree = default_place_bt_;
      } else {
        RCLCPP_ERROR(logger_, "Unsupported RunDemo mode: %u", goal->mode);
        return false;
      }
    }
    if (!bt_action_server_->loadBehaviorTree(tree)) {
      RCLCPP_ERROR(logger_, "Unable to load arm demo behavior tree");
      return false;
    }
    auto board = bt_action_server_->getBlackboard();
    // Always overwrite, including empty goals, so targets never leak across runs.
    board->rootBlackboard()->set("target_text", goal->target_text);
    board->set("demo_stage", std::string("starting"));
    ActionT::Feedback feedback;
    feedback.current_stage = "starting";
    board->set("demo_feedback", feedback);
    return true;
  }

  void onLoop() override
  {
    auto feedback = std::make_shared<ActionT::Feedback>();
    if (!bt_action_server_->getBlackboard()->get("demo_feedback", *feedback)) {
      feedback->current_stage = "starting";
    }
    bt_action_server_->publishFeedback(feedback);
  }

  void onPreempt(ActionT::Goal::ConstSharedPtr) override
  {
    RCLCPP_WARN(logger_, "arm demo preemption is not supported; cancel the active goal first");
    bt_action_server_->terminatePendingGoal();
  }

  void goalCompleted(
    ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override
  {
    ActionT::Feedback diagnostic;
    if (!bt_action_server_->getBlackboard()->get("demo_feedback", diagnostic)) {
      diagnostic.current_stage = "unknown";
    }
    result->moveit_state = diagnostic.moveit_state;
    if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
      result->error_code = 0;
      result->error_name = "SUCCESS";
      result->error_msg = "arm demo completed";
      diagnostic.error_code = 0;
      diagnostic.error_name.clear();
      diagnostic.error_source.clear();
      diagnostic.error_msg.clear();
    } else {
      if (diagnostic.error_code == 0) {
        const bool canceled = final_bt_status == nav2_behavior_tree::BtStatus::CANCELED;
        diagnostic.error_code = canceled ? -1004 : -1002;
        diagnostic.error_name = canceled ? "DEMO_CANCELED" : "BT_FAILED";
        diagnostic.error_source = "bt_navigator";
        diagnostic.error_msg = canceled ? "Demo was canceled." :
          "Behavior tree failed without an action result; inspect the BT/MoveIt logs.";
      }
      result->error_code = diagnostic.error_code;
      result->error_name = diagnostic.error_name;
      result->error_source = diagnostic.error_source;
      result->error_msg = diagnostic.error_msg;
      result->failed_stage = diagnostic.current_stage;
      result->failed_node = diagnostic.current_node;
      RCLCPP_ERROR(logger_, "Demo failed at %s [%s]: %s (%d), source=%s: %s",
        result->failed_stage.c_str(), result->failed_node.c_str(),
        result->error_name.c_str(), result->error_code, result->error_source.c_str(),
        result->error_msg.c_str());
    }
    if (final_bt_status != nav2_behavior_tree::BtStatus::SUCCEEDED &&
      cleanup_client_ && cleanup_client_->service_is_ready())
    {
      (void)cleanup_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }
    // Failure can arrive on the last BT tick: always publish one final diagnostic feedback.
    bt_action_server_->publishFeedback(std::make_shared<ActionT::Feedback>(diagnostic));
  }

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cleanup_client_;
  std::string default_pick_bt_;
  std::string default_place_bt_;
};

}  // namespace rwh_beyond_navigator

PLUGINLIB_EXPORT_CLASS(
  rwh_beyond_navigator::DemoNavigator,
  nav2_core::NavigatorBase)
