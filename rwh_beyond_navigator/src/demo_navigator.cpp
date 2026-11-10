#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rwh_beyond_msgs/action/run_demo.hpp"

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
    const auto default_path = ament_index_cpp::get_package_share_directory(
      "rwh_beyond_bt") + "/behavior_trees/demo.xml";
    if (!node->has_parameter("default_demo_bt_xml")) {
      node->declare_parameter("default_demo_bt_xml", default_path);
    }
    return node->get_parameter("default_demo_bt_xml").as_string();
  }

  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override
  {
    if (!bt_action_server_->loadBehaviorTree(goal->behavior_tree)) {
      RCLCPP_ERROR(logger_, "Unable to load arm demo behavior tree");
      return false;
    }
    bt_action_server_->getBlackboard()->set("demo_stage", std::string("starting"));
    return true;
  }

  void onLoop() override
  {
    auto feedback = std::make_shared<ActionT::Feedback>();
    if (!bt_action_server_->getBlackboard()->get("demo_stage", feedback->current_stage)) {
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
    if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
      result->error_code = 0;
      result->error_msg = "arm demo completed";
    } else {
      result->error_msg = "arm demo failed";
    }
  }
};

}  // namespace rwh_beyond_navigator

PLUGINLIB_EXPORT_CLASS(
  rwh_beyond_navigator::DemoNavigator,
  nav2_core::NavigatorBase)
