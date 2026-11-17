#include <functional>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rwh_beyond_msgs/action/run_demo.hpp"
#include "xml_tree_filter.hpp"
#include "rwh_beyond_bt/diagnostics.hpp"

namespace rwh_beyond_navigator
{
using rwh_beyond_bt::ErrorCode;
using rwh_beyond_bt::execution_diagnostic;

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
    // Declare BT-owned perception/motion overrides before Nav2 copies parameters to
    // the private action-client node used by the tree plugins.
    for (const auto & item : node->get_node_parameters_interface()->get_parameter_overrides()) {
      if ((item.first.rfind("bt_perception.", 0) == 0 ||
        item.first.rfind("bt_motion.", 0) == 0 || item.first == "octomap_enabled") &&
        !node->has_parameter(item.first))
      {
        node->declare_parameter(item.first, item.second);
      }
    }
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
    return filtered_tree(default_pick_bt_);
  }

  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother>/*odom_smoother*/) override
  {
    auto node = parent_node.lock();
    if (!node) {
      RCLCPP_ERROR(logger_, "Unable to configure Groot2 monitoring: navigator node expired");
      return false;
    }

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
    auto board = bt_action_server_->getBlackboard();
    rwh_beyond_bt::reset_diagnostics(board);
    board->rootBlackboard()->set("target_text", goal->target_text);
    std::string tree = goal->behavior_tree;
    if (tree.empty()) {
      const auto mode = goal->mode == 0 ? ActionT::Goal::MODE_PICK : goal->mode;
      if (mode == ActionT::Goal::MODE_PICK) {
        tree = default_pick_bt_;
      } else if (mode == ActionT::Goal::MODE_PLACE) {
        tree = default_place_bt_;
      } else {
        return reject_request(ErrorCode::INVALID_MODE, "INVALID_MODE",
          "Unsupported RunDemo mode: " + std::to_string(goal->mode));
      }
    }
    try {
      if (!bt_action_server_->loadBehaviorTree(filtered_tree(tree))) {
        return reject_request(ErrorCode::INVALID_TREE, "INVALID_TREE",
          "Unable to load arm demo behavior tree: " + tree);
      }
    } catch (const std::exception & error) {
      return reject_request(ErrorCode::INVALID_TREE, "INVALID_TREE", error.what());
    }
    return true;
  }

  void onLoop() override
  {
    // Snapshot the BT-owned state; no ROS message is stored in the Blackboard.
    bt_action_server_->publishFeedback(make_feedback(
        execution_diagnostic(bt_action_server_->getBlackboard())));
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
    auto diagnostic = execution_diagnostic(bt_action_server_->getBlackboard());
    result->moveit_state = diagnostic.moveit_state;
    result->backend_stage = diagnostic.backend_stage;
    if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
      result->error_code = 0;
      result->error_name = "SUCCESS";
      result->error_msg = "arm demo completed";
      diagnostic.error_code = 0;
      diagnostic.error_domain.clear();
      diagnostic.error_name.clear();
      diagnostic.error_source.clear();
      diagnostic.error_msg.clear();
    } else {
      if (diagnostic.error_code == 0) {
        const bool canceled = final_bt_status == nav2_behavior_tree::BtStatus::CANCELED;
        diagnostic.error_code = canceled ? ErrorCode::CANCELED : ErrorCode::BT_FAILED;
        diagnostic.error_domain = "navigator";
        diagnostic.error_name = canceled ? "CANCELED" : "BT_FAILED";
        diagnostic.error_source = "/run_demo";
        diagnostic.error_msg = canceled ? "Demo was canceled." :
          "Behavior tree failed without a structured diagnostic.";
      }
      result->error_code = diagnostic.error_code;
      result->error_domain = diagnostic.error_domain;
      result->error_name = diagnostic.error_name;
      result->error_source = diagnostic.error_source;
      result->error_msg = diagnostic.error_msg;
      result->failed_stage = diagnostic.current_stage;
      result->failed_node = diagnostic.current_node;
      RCLCPP_ERROR(logger_, "Demo failed at %s [%s]: %s/%s (%d), source=%s: %s",
        result->failed_stage.c_str(), result->failed_node.c_str(),
        result->error_domain.c_str(), result->error_name.c_str(), result->error_code,
        result->error_source.c_str(), result->error_msg.c_str());
    }
    if (final_bt_status != nav2_behavior_tree::BtStatus::SUCCEEDED) {
      std::function<void()> cleanup;
      if (bt_action_server_->getBlackboard()->rootBlackboard()->get("bt_motion_cleanup", cleanup)) {
        try {cleanup();} catch (const std::exception & error) {
          RCLCPP_ERROR(logger_, "BT motion cleanup failed: %s", error.what());
        }
      }
    }
    // Failure can arrive on the last BT tick: always publish one final diagnostic feedback.
    bt_action_server_->publishFeedback(make_feedback(diagnostic));
  }

private:
  ActionT::Feedback::SharedPtr make_feedback(const rwh_beyond_bt::ExecutionDiagnostic & state)
  {
    auto feedback = std::make_shared<ActionT::Feedback>();
    feedback->current_stage = state.current_stage;
    feedback->current_node = state.current_node;
    feedback->moveit_state = state.moveit_state;
    feedback->backend_stage = state.backend_stage;
    feedback->error_code = state.error_code;
    feedback->error_domain = state.error_domain;
    feedback->error_name = state.error_name;
    feedback->error_source = state.error_source;
    feedback->error_msg = state.error_msg;
    return feedback;
  }

  bool reject_request(int32_t code, const std::string & name, const std::string & message)
  {
    // Jazzy's false goalReceived path aborts with a default Result. A built-in,
    // failure-only tree lets completion publish the structured error instead,
    // without ticking a previously loaded robot tree.
    const auto rejection_tree = ament_index_cpp::get_package_share_directory(
      "rwh_beyond_navigator") + "/behavior_trees/rejected_goal.xml";
    if (!bt_action_server_->loadBehaviorTree(rejection_tree)) {
      RCLCPP_FATAL(logger_, "Unable to load the installed request rejection tree");
      return false;
    }
    auto board = bt_action_server_->getBlackboard();
    rwh_beyond_bt::begin_stage(board, "request_validation", "DemoNavigator");
    rwh_beyond_bt::report_failure(board, code, name, "/run_demo", message, "navigator");
    return true;
  }

  std::string filtered_tree(const std::string & source)
  {
    const auto path = xml_filter_.prepare(source);
    if (xml_filter_.removed_attributes()) {
      RCLCPP_INFO(logger_, "BT XML filter: removed %zu empty attributes from %s (with includes)",
        xml_filter_.removed_attributes(), source.c_str());
    }
    return path;
  }

  XmlTreeFilter xml_filter_;
  std::string default_pick_bt_;
  std::string default_place_bt_;
};

}  // namespace rwh_beyond_navigator

PLUGINLIB_EXPORT_CLASS(
  rwh_beyond_navigator::DemoNavigator,
  nav2_core::NavigatorBase)
